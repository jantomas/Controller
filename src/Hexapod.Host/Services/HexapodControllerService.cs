using System.Diagnostics;
using Hexapod.Autonomy.Confirmation;
using Hexapod.Autonomy.Decision;
using Hexapod.Autonomy.Mission;
using Hexapod.Communication;
using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Events;
using Hexapod.Core.Models;
using Hexapod.Core.Services;
using Hexapod.Core.StateMachine;
using Hexapod.Movement.Services;
using Hexapod.Sensors.Services;
using Hexapod.Telemetry.Collection;
using Hexapod.Telemetry.Upload;
using Hexapod.Vision.Abstractions;
using Microsoft.Extensions.Options;

namespace Hexapod.Host.Services;

/// <summary>
/// Main hosted service that orchestrates all hexapod subsystems.
/// </summary>
public sealed class HexapodControllerService : BackgroundService
{
    private readonly IEventBus _eventBus;
    private readonly IOperationStateMachine _stateMachine;
    private readonly ISensorManager _sensorManager;
    private readonly IMovementController _movementController;
    private readonly IObjectDetector _objectDetector;
    private readonly ITerrainClassifier _terrainClassifier;
    private readonly ICommunicationManager _communicationManager;
    private readonly IDecisionEngine _decisionEngine;
    private readonly AutonomyManager _autonomyManager;
    private readonly IMissionPlanner _missionPlanner;
    private readonly ITelemetryCollector _telemetryCollector;
    private readonly ITelemetryUploader _telemetryUploader;
    private readonly ILogger<HexapodControllerService> _logger;
    private readonly HexapodConfiguration _config;

    private readonly Stopwatch _cycleStopwatch = new();
    private SensorState? _lastSensorState;
    private int _cycleCount;

    public HexapodControllerService(
        IEventBus eventBus,
        IOperationStateMachine stateMachine,
        ISensorManager sensorManager,
        IMovementController movementController,
        IObjectDetector objectDetector,
        ITerrainClassifier terrainClassifier,
        ICommunicationManager communicationManager,
        IDecisionEngine decisionEngine,
        AutonomyManager autonomyManager,
        IMissionPlanner missionPlanner,
        ITelemetryCollector telemetryCollector,
        ITelemetryUploader telemetryUploader,
        IOptions<HexapodConfiguration> config,
        ILogger<HexapodControllerService> logger)
    {
        _eventBus = eventBus;
        _stateMachine = stateMachine;
        _sensorManager = sensorManager;
        _movementController = movementController;
        _objectDetector = objectDetector;
        _terrainClassifier = terrainClassifier;
        _communicationManager = communicationManager;
        _decisionEngine = decisionEngine;
        _autonomyManager = autonomyManager;
        _missionPlanner = missionPlanner;
        _telemetryCollector = telemetryCollector;
        _telemetryUploader = telemetryUploader;
        _config = config.Value;
        _logger = logger;
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        _logger.LogInformation("Hexapod Controller Service starting...");

        // Subscribe to events
        SubscribeToEvents();

        // Initialize subsystems
        await InitializeSubsystemsAsync(stoppingToken);

        // Start telemetry uploader
        await _telemetryUploader.StartAsync(stoppingToken);

        _logger.LogInformation("All subsystems initialized, entering main control loop");

        // Main control loop
        while (!stoppingToken.IsCancellationRequested)
        {
            _cycleStopwatch.Restart();

            try
            {
                await ExecuteControlCycleAsync(stoppingToken);
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                break;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error in control cycle");
                await Task.Delay(1000, stoppingToken);
            }

            _cycleStopwatch.Stop();
            _cycleCount++;

            // Maintain consistent cycle time (50Hz = 20ms)
            var targetCycleTime = TimeSpan.FromMilliseconds(20);
            var elapsed = _cycleStopwatch.Elapsed;
            if (elapsed < targetCycleTime)
            {
                await Task.Delay(targetCycleTime - elapsed, stoppingToken);
            }
            else if (_cycleCount % 100 == 0)
            {
                _logger.LogWarning(
                    "Control cycle exceeded target time: {Elapsed}ms > {Target}ms",
                    elapsed.TotalMilliseconds, targetCycleTime.TotalMilliseconds);
            }
        }

        _logger.LogInformation("Hexapod Controller Service stopping...");
        await ShutdownSubsystemsAsync();
    }

    private async Task InitializeSubsystemsAsync(CancellationToken cancellationToken)
    {
        _logger.LogInformation("Initializing sensors...");
        await _sensorManager.InitializeAsync(cancellationToken);

        _logger.LogInformation("Initializing communication channels (WiFi/LoRaWAN)...");
        await _communicationManager.InitializeAsync(cancellationToken);
        _logger.LogInformation("Active communication channel: {Channel}", _communicationManager.ActiveChannel);

        // Transition to SemiAutonomous state
        await _stateMachine.TransitionToAsync(OperationMode.SemiAutonomous, "Initialization complete");

        _eventBus.Publish(new SystemStartupEvent
        {
            EventId = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow,
            Source = nameof(HexapodControllerService),
            Version = "1.0.0",
            StartupMode = _config.Autonomy.DefaultMode
        });
    }

    private async Task ExecuteControlCycleAsync(CancellationToken cancellationToken)
    {
        // 1. Read sensors
        _lastSensorState = await _sensorManager.GetSensorStateAsync(cancellationToken);

        // 2. Check for obstacles or hazards using distance sensors
        var hasObstacles = _lastSensorState.DistanceReadings.Any(d => d.Distance < 0.3);
        if (hasObstacles && _movementController.CurrentState == MovementState.Walking)
        {
            var obstacles = _lastSensorState.DistanceReadings
                .Where(d => d.Distance < 0.3)
                .Select(d => d.SensorId.ToString())
                .ToList();
            await HandleObstaclesAsync(obstacles, cancellationToken);
        }

        // 3. Check power status
        if (_lastSensorState.PowerStatus?.BatteryPercentage < _config.Power.LowPowerThreshold)
        {
            await HandleLowPowerAsync(_lastSensorState.PowerStatus, cancellationToken);
        }

        // 4. Process vision (at lower frequency)
        if (_cycleCount % 5 == 0) // Every 5th cycle (10Hz)
        {
            await ProcessVisionAsync(cancellationToken);
        }

        // 5. Update mission progress
        var mission = _missionPlanner.GetCurrentMission();
        if (mission?.Status == "Active")
        {
            await UpdateMissionProgressAsync(mission, cancellationToken);
        }

        // 6. Update telemetry (at lower frequency)
        if (_cycleCount % 10 == 0) // Every 10th cycle (5Hz)
        {
            await _telemetryCollector.RecordSensorStateAsync(_lastSensorState);
        }

        // 7. Send periodic status via best available channel (every 30 seconds)
        if (_cycleCount % 1500 == 0)
        {
            await SendPeriodicStatusAsync(cancellationToken);
        }
    }

    private async Task HandleObstaclesAsync(IReadOnlyList<string> obstacles, CancellationToken cancellationToken)
    {
        _logger.LogWarning("Obstacles detected in directions: {Directions}", string.Join(", ", obstacles));

        // Pause movement
        await _movementController.StopAsync(cancellationToken);

        // Create decision context
        var context = new DecisionContext
        {
            Type = DecisionType.ObstacleAvoidance,
            SensorState = _lastSensorState!,
            SceneAnalysis = null,
            CurrentMission = _missionPlanner.GetCurrentMission(),
            OperationMode = _stateMachine.CurrentMode,
            DetectedObjects = null
        };

        // Process through autonomy manager
        var result = await _autonomyManager.ProcessSituationAsync(context, cancellationToken);

        if (result.Status == ActionStatus.Approved && result.SelectedOption != null)
        {
            _logger.LogInformation("Executing obstacle avoidance: {Option}", result.SelectedOption.Description);
            // Execute the selected option
            await ExecuteAvoidanceOptionAsync(result.SelectedOption, cancellationToken);
        }
    }

    private async Task HandleLowPowerAsync(PowerStatus powerStatus, CancellationToken cancellationToken)
    {
        _logger.LogWarning("Low battery: {Percent}%", powerStatus.BatteryPercentage);

        if (powerStatus.BatteryPercentage < 10)
        {
            _logger.LogCritical("Critical battery level, initiating emergency stop");
            await _stateMachine.ForceEmergencyStopAsync("Critical battery", cancellationToken);
            _movementController.EmergencyStop();
        }
        else if (powerStatus.BatteryPercentage < 20)
        {
            // Switch to safe mode
            await _stateMachine.ForceSafeModeAsync("Low battery", cancellationToken);
            
            // Cancel current mission
            var mission = _missionPlanner.GetCurrentMission();
            if (mission != null)
            {
                await _missionPlanner.CancelMissionAsync(mission.MissionId, "Low battery", cancellationToken);
            }
        }
    }

    private async Task ProcessVisionAsync(CancellationToken cancellationToken)
    {
        // In a full implementation, this would capture camera frame and run inference
        // For now, we simulate the process
        await Task.CompletedTask;
    }

    private async Task UpdateMissionProgressAsync(MissionStatus mission, CancellationToken cancellationToken)
    {
        if (_lastSensorState?.Position == null)
            return;

        var nextWaypoint = _missionPlanner.GetNextWaypoint();
        if (nextWaypoint == null)
            return;

        var distance = CalculateDistance(_lastSensorState.Position, nextWaypoint.Position);
        var tolerance = nextWaypoint.ToleranceMeters ?? 2.0;

        if (distance <= tolerance)
        {
            _logger.LogInformation("Reached waypoint {Id}", nextWaypoint.WaypointId);
            await _missionPlanner.CompleteWaypointAsync(nextWaypoint.WaypointId, cancellationToken);
        }
    }

    private async Task SendPeriodicStatusAsync(CancellationToken cancellationToken)
    {
        var position = _lastSensorState?.Position;
        if (position != null)
        {
            await _communicationManager.SendPositionAsync(position, cancellationToken);
        }

        var status = new StatusReport
        {
            BatteryPercent = (byte)(_lastSensorState?.PowerStatus?.BatteryPercentage ?? 0),
            CurrentMode = _stateMachine.CurrentMode,
            CurrentState = _movementController.CurrentState,
            Flags = BuildStatusFlags()
        };

        await _communicationManager.SendStatusAsync(status, cancellationToken);
        
        _logger.LogDebug("Status sent via {Channel}", _communicationManager.ActiveChannel);
    }

    private async Task ExecuteAvoidanceOptionAsync(DecisionOption option, CancellationToken cancellationToken)
    {
        switch (option.OptionId)
        {
            case "avoid_left":
                await _movementController.RotateAsync(-45, cancellationToken);
                break;
            case "avoid_right":
                await _movementController.RotateAsync(45, cancellationToken);
                break;
            case "reverse":
                await _movementController.StartWalkingAsync(Math.PI, 0.5, cancellationToken); // Walk backward
                break;
            case "stop":
            default:
                // Already stopped
                break;
        }
    }

    private StatusFlags BuildStatusFlags()
    {
        var flags = StatusFlags.None;
        
        if (_communicationManager.IsWifiAvailable)
            flags |= StatusFlags.CloudConnected;
        
        if (_missionPlanner.GetCurrentMission()?.Status == "Active")
            flags |= StatusFlags.MissionActive;
        
        // Check for obstacles using distance sensors
        if (_lastSensorState?.DistanceReadings.Any(d => d.Distance < 0.3) == true)
            flags |= StatusFlags.ObstacleDetected;
        
        if (_lastSensorState?.PowerStatus?.BatteryPercentage < 20)
            flags |= StatusFlags.LowBattery;

        return flags;
    }

    private static double CalculateDistance(GeoPosition pos1, GeoPosition pos2)
    {
        // Haversine formula for distance between two coordinates
        const double R = 6371000; // Earth radius in meters
        
        var lat1Rad = pos1.Latitude * Math.PI / 180;
        var lat2Rad = pos2.Latitude * Math.PI / 180;
        var deltaLat = (pos2.Latitude - pos1.Latitude) * Math.PI / 180;
        var deltaLon = (pos2.Longitude - pos1.Longitude) * Math.PI / 180;

        var a = Math.Sin(deltaLat / 2) * Math.Sin(deltaLat / 2) +
                Math.Cos(lat1Rad) * Math.Cos(lat2Rad) *
                Math.Sin(deltaLon / 2) * Math.Sin(deltaLon / 2);
        
        var c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));

        return R * c;
    }

    private void SubscribeToEvents()
    {
        // Emergency stop handler
        _eventBus.Subscribe<EmergencyStopEvent>()
            .Subscribe(async e =>
            {
                _logger.LogCritical("EMERGENCY STOP: {Reason}", e.Reason);
                _movementController.EmergencyStop();
                await _stateMachine.ForceEmergencyStopAsync(e.Reason);
            });

        // Mode change handler
        _eventBus.Subscribe<ModeChangedEvent>()
            .Subscribe(e =>
            {
                _logger.LogInformation("Mode changed to {Mode}", e.NewMode);
            });

        // Remote command handler
        _eventBus.Subscribe<OperatorCommandReceivedEvent>()
            .Subscribe(async e =>
            {
                _logger.LogInformation("Command received: {Type}", e.CommandType);
                await HandleOperatorCommandAsync(e);
            });
    }

    private async Task HandleOperatorCommandAsync(OperatorCommandReceivedEvent command)
    {
        switch (command.CommandType)
        {
            case "EmergencyStop":
                _movementController.EmergencyStop();
                await _stateMachine.ForceEmergencyStopAsync("Operator command");
                break;

            case "SwitchMode":
                if (command.Parameters.TryGetValue("mode", out var modeStr) &&
                    Enum.TryParse<OperationMode>(modeStr?.ToString(), out var mode))
                {
                    await _stateMachine.TransitionToAsync(mode, "Operator command");
                }
                break;

            case "StartMission":
                // Would parse mission definition from parameters
                break;

            case "PauseMission":
                var mission = _missionPlanner.GetCurrentMission();
                if (mission != null)
                {
                    await _missionPlanner.PauseMissionAsync(mission.MissionId);
                }
                break;

            case "ResumeMission":
                mission = _missionPlanner.GetCurrentMission();
                if (mission != null)
                {
                    await _missionPlanner.ResumeMissionAsync(mission.MissionId);
                }
                break;
        }
    }

    private async Task ShutdownSubsystemsAsync()
    {
        _logger.LogInformation("Shutting down subsystems...");

        await _telemetryUploader.StopAsync();
        await _movementController.StopAsync();
        await _communicationManager.DisconnectAsync();

        _eventBus.Publish(new SystemShutdownEvent
        {
            EventId = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow,
            Source = nameof(HexapodControllerService),
            Reason = "Normal shutdown"
        });
    }
}

/// <summary>
/// System startup event.
/// </summary>
public record SystemStartupEvent : HexapodEvent
{
    public required string Version { get; init; }
    public required string StartupMode { get; init; }
}

/// <summary>
/// System shutdown event.
/// </summary>
public record SystemShutdownEvent : HexapodEvent
{
    public required string Reason { get; init; }
}
