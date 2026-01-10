using System.Numerics;
using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Events;
using Hexapod.Core.Services;
using Hexapod.Movement.Gait;
using Hexapod.Movement.Kinematics;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Movement.Services;

/// <summary>
/// Interface for the movement controller service.
/// </summary>
public interface IMovementController
{
    /// <summary>
    /// Gets the current movement state.
    /// </summary>
    MovementState CurrentState { get; }

    /// <summary>
    /// Gets the current gait type.
    /// </summary>
    GaitType CurrentGait { get; }

    /// <summary>
    /// Gets the current speed (0.0 to 1.0).
    /// </summary>
    double CurrentSpeed { get; }

    /// <summary>
    /// Starts walking in the specified direction.
    /// </summary>
    Task StartWalkingAsync(double direction, double speed, CancellationToken cancellationToken = default);

    /// <summary>
    /// Stops all movement and returns to idle.
    /// </summary>
    Task StopAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Performs an emergency stop (immediate halt).
    /// </summary>
    void EmergencyStop();

    /// <summary>
    /// Sets the turn rate while walking.
    /// </summary>
    void SetTurnRate(double turnRate);

    /// <summary>
    /// Changes the current gait pattern.
    /// </summary>
    Task ChangeGaitAsync(GaitType gaitType, CancellationToken cancellationToken = default);

    /// <summary>
    /// Rotates in place by the specified angle.
    /// </summary>
    Task RotateAsync(double angle, CancellationToken cancellationToken = default);

    /// <summary>
    /// Moves to a relative position.
    /// </summary>
    Task MoveToRelativeAsync(Vector3 targetPosition, CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets current joint positions for all legs.
    /// </summary>
    IReadOnlyList<LegJointState> GetJointStates();

    /// <summary>
    /// Updates the movement controller (called from main loop).
    /// </summary>
    Task UpdateAsync(TimeSpan deltaTime, CancellationToken cancellationToken = default);
}

/// <summary>
/// Represents the state of a leg's joints.
/// </summary>
public record LegJointState
{
    public required int LegId { get; init; }
    public required double CoxaAngle { get; init; }
    public required double FemurAngle { get; init; }
    public required double TibiaAngle { get; init; }
    public required Vector3 FootPosition { get; init; }
    public required bool IsGrounded { get; init; }
}

/// <summary>
/// Implementation of the movement controller.
/// </summary>
public sealed class MovementController : IMovementController, IDisposable
{
    private readonly IEventBus _eventBus;
    private readonly ILogger<MovementController> _logger;
    private readonly MovementConfiguration _config;
    private readonly HexapodBody _body;
    private readonly IServoController _servoController;
    
    private IGaitGenerator _currentGaitGenerator;
    private MovementState _currentState = MovementState.Idle;
    private GaitType _currentGait;
    private double _currentSpeed;
    private double _currentDirection;
    private double _currentTurnRate;
    private double _gaitPhase;
    private bool _emergencyStop;
    private readonly object _lock = new();

    public MovementController(
        IEventBus eventBus,
        IServoController servoController,
        IOptions<HexapodConfiguration> config,
        ILogger<MovementController> logger)
    {
        _eventBus = eventBus;
        _servoController = servoController;
        _logger = logger;
        _config = config.Value.Movement;

        _body = new HexapodBody(
            _config.LegDimensions.CoxaLength,
            _config.LegDimensions.FemurLength,
            _config.LegDimensions.TibiaLength);

        _currentGait = Enum.Parse<GaitType>(_config.DefaultGait);
        _currentGaitGenerator = GaitFactory.Create(_currentGait);
    }

    public MovementState CurrentState => _currentState;
    public GaitType CurrentGait => _currentGait;
    public double CurrentSpeed => _currentSpeed;

    public async Task StartWalkingAsync(double direction, double speed, CancellationToken cancellationToken = default)
    {
        if (_emergencyStop)
        {
            _logger.LogWarning("Cannot start walking - emergency stop active");
            return;
        }

        lock (_lock)
        {
            _currentDirection = direction;
            _currentSpeed = Math.Clamp(speed, 0.0, 1.0);
            
            if (_currentState != MovementState.Walking)
            {
                var previousState = _currentState;
                _currentState = MovementState.Walking;
                
                _eventBus.Publish(new MovementStateChangedEvent
                {
                    EventId = Guid.NewGuid().ToString(),
                    Timestamp = DateTimeOffset.UtcNow,
                    Source = nameof(MovementController),
                    PreviousState = previousState,
                    NewState = MovementState.Walking,
                    Gait = _currentGait
                });
            }
        }

        _logger.LogInformation("Started walking: direction={Direction}°, speed={Speed}", 
            direction * 180 / Math.PI, speed);

        await Task.CompletedTask;
    }

    public async Task StopAsync(CancellationToken cancellationToken = default)
    {
        lock (_lock)
        {
            _currentSpeed = 0;
            _currentTurnRate = 0;
            
            if (_currentState != MovementState.Idle)
            {
                var previousState = _currentState;
                _currentState = MovementState.Idle;

                _eventBus.Publish(new MovementStateChangedEvent
                {
                    EventId = Guid.NewGuid().ToString(),
                    Timestamp = DateTimeOffset.UtcNow,
                    Source = nameof(MovementController),
                    PreviousState = previousState,
                    NewState = MovementState.Idle,
                    Gait = _currentGait
                });
            }
        }

        // Move to neutral stance
        await MoveToNeutralStanceAsync(cancellationToken);
        
        _logger.LogInformation("Movement stopped");
    }

    public void EmergencyStop()
    {
        _emergencyStop = true;
        _currentSpeed = 0;
        _currentTurnRate = 0;
        _currentState = MovementState.Idle;
        
        _servoController.DisableAll();
        
        _logger.LogCritical("EMERGENCY STOP activated");
        
        _eventBus.Publish(new EmergencyEvent
        {
            EventId = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow,
            Source = nameof(MovementController),
            Type = EmergencyType.OperatorAlert,
            Description = "Emergency stop activated",
            RequiresImmediateAction = true
        });
    }

    public void SetTurnRate(double turnRate)
    {
        lock (_lock)
        {
            _currentTurnRate = Math.Clamp(turnRate, -1.0, 1.0);
        }
    }

    public async Task ChangeGaitAsync(GaitType gaitType, CancellationToken cancellationToken = default)
    {
        if (_currentGait == gaitType)
            return;

        _logger.LogInformation("Changing gait from {Old} to {New}", _currentGait, gaitType);

        lock (_lock)
        {
            _currentGait = gaitType;
            _currentGaitGenerator = GaitFactory.Create(gaitType);
            _gaitPhase = 0; // Reset phase for smooth transition
        }

        _eventBus.Publish(new MovementStateChangedEvent
        {
            EventId = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow,
            Source = nameof(MovementController),
            PreviousState = _currentState,
            NewState = _currentState,
            Gait = gaitType
        });

        await Task.CompletedTask;
    }

    public async Task RotateAsync(double angle, CancellationToken cancellationToken = default)
    {
        var previousState = _currentState;
        _currentState = MovementState.Turning;

        _eventBus.Publish(new MovementStateChangedEvent
        {
            EventId = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow,
            Source = nameof(MovementController),
            PreviousState = previousState,
            NewState = MovementState.Turning,
            Gait = _currentGait
        });

        // Implement rotation logic
        var direction = angle > 0 ? 1.0 : -1.0;
        var remainingAngle = Math.Abs(angle);
        var rotationSpeed = 0.5; // radians per second

        while (remainingAngle > 0.01 && !cancellationToken.IsCancellationRequested && !_emergencyStop)
        {
            var deltaAngle = Math.Min(remainingAngle, rotationSpeed * 0.02); // 50Hz update
            remainingAngle -= deltaAngle;
            
            _currentTurnRate = direction;
            _currentSpeed = 0.3;
            
            await Task.Delay(20, cancellationToken);
        }

        await StopAsync(cancellationToken);
    }

    public async Task MoveToRelativeAsync(Vector3 targetPosition, CancellationToken cancellationToken = default)
    {
        var distance = targetPosition.Length();
        var direction = Math.Atan2(targetPosition.Y, targetPosition.X);
        
        _logger.LogInformation("Moving to relative position: ({X}, {Y}), distance={Distance}m", 
            targetPosition.X, targetPosition.Y, distance);

        await StartWalkingAsync(direction, 0.5, cancellationToken);

        // Simple distance-based movement (would integrate with GPS in real implementation)
        var estimatedTime = distance / (_config.DefaultSpeed * 0.5);
        await Task.Delay(TimeSpan.FromSeconds(estimatedTime), cancellationToken);
        
        await StopAsync(cancellationToken);
    }

    public IReadOnlyList<LegJointState> GetJointStates()
    {
        var states = new List<LegJointState>();
        var legPhases = _currentGaitGenerator.GetLegPhases(_gaitPhase);

        foreach (var leg in _body.Legs)
        {
            states.Add(new LegJointState
            {
                LegId = leg.LegId,
                CoxaAngle = leg.CoxaAngle,
                FemurAngle = leg.FemurAngle,
                TibiaAngle = leg.TibiaAngle,
                FootPosition = leg.GetFootPosition(),
                IsGrounded = !legPhases[leg.LegId].IsSwingPhase
            });
        }

        return states.AsReadOnly();
    }

    public async Task UpdateAsync(TimeSpan deltaTime, CancellationToken cancellationToken = default)
    {
        if (_emergencyStop || _currentState == MovementState.Idle)
        {
            return;
        }

        lock (_lock)
        {
            // Advance gait phase
            var cycleTime = 1.0 / (_currentSpeed * 2 + 0.5); // Faster cycle at higher speeds
            _gaitPhase = (_gaitPhase + deltaTime.TotalSeconds / cycleTime) % 1.0;
        }

        // Generate foot positions
        var footPositions = _currentGaitGenerator.GenerateFootPositions(
            _gaitPhase,
            _currentDirection,
            _currentSpeed,
            _currentTurnRate,
            _config.StepLength,
            _config.StepHeight,
            _config.BodyHeight);

        // Calculate inverse kinematics
        _body.SetFootPositions(footPositions);

        // Update servos
        var jointStates = GetJointStates();
        await _servoController.SetPositionsAsync(jointStates, cancellationToken);
    }

    private async Task MoveToNeutralStanceAsync(CancellationToken cancellationToken)
    {
        var neutralPositions = _currentGaitGenerator.GenerateFootPositions(
            0, 0, 0, 0,
            _config.StepLength,
            _config.StepHeight,
            _config.BodyHeight);

        _body.SetFootPositions(neutralPositions);
        var jointStates = GetJointStates();
        await _servoController.SetPositionsAsync(jointStates, cancellationToken);
    }

    public void Dispose()
    {
        // Cleanup resources
    }
}

/// <summary>
/// Interface for servo controller hardware abstraction.
/// </summary>
public interface IServoController
{
    /// <summary>
    /// Sets servo positions for all joints.
    /// </summary>
    Task SetPositionsAsync(IReadOnlyList<LegJointState> jointStates, CancellationToken cancellationToken = default);

    /// <summary>
    /// Disables all servos (emergency).
    /// </summary>
    void DisableAll();

    /// <summary>
    /// Enables all servos.
    /// </summary>
    void EnableAll();
}

/// <summary>
/// PCA9685-based servo controller implementation.
/// </summary>
public class Pca9685ServoController : IServoController
{
    private readonly ILogger<Pca9685ServoController> _logger;
    private readonly int _i2cAddress;
    private bool _enabled = true;

    public Pca9685ServoController(
        IOptions<HexapodConfiguration> config,
        ILogger<Pca9685ServoController> logger)
    {
        _logger = logger;
        _i2cAddress = config.Value.Hardware.ServoControllerAddress;
    }

    public async Task SetPositionsAsync(IReadOnlyList<LegJointState> jointStates, CancellationToken cancellationToken = default)
    {
        if (!_enabled)
            return;

        // Convert angles to PWM values and send to hardware
        foreach (var state in jointStates)
        {
            var coxaPwm = AngleToPwm(state.CoxaAngle, state.LegId * 3);
            var femurPwm = AngleToPwm(state.FemurAngle, state.LegId * 3 + 1);
            var tibiaPwm = AngleToPwm(state.TibiaAngle, state.LegId * 3 + 2);

            // TODO: Actual I2C communication with PCA9685
            // await WriteRegisterAsync(channel, pwmValue);
        }

        await Task.CompletedTask;
    }

    public void DisableAll()
    {
        _enabled = false;
        _logger.LogWarning("All servos disabled");
        // TODO: Send PWM off command to all channels
    }

    public void EnableAll()
    {
        _enabled = true;
        _logger.LogInformation("All servos enabled");
    }

    private static int AngleToPwm(double angleRadians, int channel)
    {
        // Convert angle to PWM value (typical servo range 500-2500 microseconds)
        var angleDegrees = angleRadians * 180 / Math.PI;
        var normalized = (angleDegrees + 90) / 180; // Normalize to 0-1
        var pwmUs = 500 + normalized * 2000; // 500-2500 microseconds
        
        // Convert to PCA9685 12-bit value (4096 steps, 50Hz = 20ms period)
        return (int)(pwmUs / 20000 * 4096);
    }
}
