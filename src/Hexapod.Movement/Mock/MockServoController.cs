using Hexapod.Core.Configuration;
using Hexapod.Movement.Services;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Movement.Mock;

/// <summary>
/// Mock servo controller for development and testing without hardware.
/// Simulates servo behavior and logs all operations for debugging.
/// </summary>
public sealed class MockServoController : IServoController, IDisposable
{
    private readonly ILogger<MockServoController> _logger;
    private readonly MockModeConfiguration _mockConfig;
    private readonly HardwareConfiguration _hardwareConfig;
    private readonly Random _random = new();
    private bool _enabled = true;
    private bool _disposed;

    /// <summary>
    /// Current simulated positions for all 18 servos (in degrees).
    /// </summary>
    private readonly double[] _currentPositions = new double[18];

    /// <summary>
    /// Target positions for all 18 servos (in degrees).
    /// </summary>
    private readonly double[] _targetPositions = new double[18];

    /// <summary>
    /// Event raised when servo positions change (for debugging/visualization).
    /// </summary>
    public event EventHandler<ServoPositionsChangedEventArgs>? PositionsChanged;

    public MockServoController(
        IOptions<HexapodConfiguration> config,
        ILogger<MockServoController> logger)
    {
        _logger = logger;
        _mockConfig = config.Value.MockMode;
        _hardwareConfig = config.Value.Hardware;

        // Initialize all servos to neutral position (0 degrees)
        for (int i = 0; i < 18; i++)
        {
            _currentPositions[i] = 0;
            _targetPositions[i] = 0;
        }

        _logger.LogInformation("🎮 Mock Servo Controller initialized - No hardware connected");
        _logger.LogInformation("   Simulating {LegCount} legs × {JointsPerLeg} joints = {Total} servos",
            _hardwareConfig.LegCount,
            _hardwareConfig.JointsPerLeg,
            _hardwareConfig.LegCount * _hardwareConfig.JointsPerLeg);
    }

    /// <inheritdoc/>
    public async Task SetPositionsAsync(IReadOnlyList<LegJointState> jointStates, CancellationToken cancellationToken = default)
    {
        if (!_enabled)
        {
            if (_mockConfig.VerboseLogging)
                _logger.LogDebug("Mock servos disabled, ignoring position update");
            return;
        }

        // Simulate servo delay
        if (_mockConfig.SimulatedServoDelayMs > 0)
        {
            await Task.Delay(_mockConfig.SimulatedServoDelayMs, cancellationToken);
        }

        foreach (var state in jointStates)
        {
            int baseChannel = state.LegId * 3;

            // Apply positions with optional noise
            _targetPositions[baseChannel] = ApplyNoise(state.CoxaAngle);
            _targetPositions[baseChannel + 1] = ApplyNoise(state.FemurAngle);
            _targetPositions[baseChannel + 2] = ApplyNoise(state.TibiaAngle);

            // Update current positions (instant in mock mode)
            _currentPositions[baseChannel] = _targetPositions[baseChannel];
            _currentPositions[baseChannel + 1] = _targetPositions[baseChannel + 1];
            _currentPositions[baseChannel + 2] = _targetPositions[baseChannel + 2];

            if (_mockConfig.VerboseLogging)
            {
                _logger.LogDebug(
                    "🦿 Leg {LegId}: Coxa={Coxa:F1}° Femur={Femur:F1}° Tibia={Tibia:F1}°",
                    state.LegId,
                    _currentPositions[baseChannel],
                    _currentPositions[baseChannel + 1],
                    _currentPositions[baseChannel + 2]);
            }
        }

        // Raise event for external listeners (e.g., visualizers)
        PositionsChanged?.Invoke(this, new ServoPositionsChangedEventArgs(
            (double[])_currentPositions.Clone(),
            DateTimeOffset.UtcNow));
    }

    /// <inheritdoc/>
    public void DisableAll()
    {
        _enabled = false;
        _logger.LogWarning("🔴 Mock servos DISABLED (emergency stop simulated)");

        // Reset all positions to 0 (limp)
        for (int i = 0; i < 18; i++)
        {
            _currentPositions[i] = 0;
            _targetPositions[i] = 0;
        }
    }

    /// <inheritdoc/>
    public void EnableAll()
    {
        _enabled = true;
        _logger.LogInformation("🟢 Mock servos ENABLED");
    }

    /// <summary>
    /// Gets the current simulated position for a specific channel.
    /// </summary>
    public double GetPosition(int channel)
    {
        if (channel < 0 || channel >= 18)
            throw new ArgumentOutOfRangeException(nameof(channel), "Channel must be 0-17");

        return _currentPositions[channel];
    }

    /// <summary>
    /// Gets all current simulated positions.
    /// </summary>
    public IReadOnlyList<double> GetAllPositions() => _currentPositions;

    /// <summary>
    /// Gets whether servos are currently enabled.
    /// </summary>
    public bool IsEnabled => _enabled;

    private double ApplyNoise(double value)
    {
        if (!_mockConfig.SimulateNoise || _mockConfig.NoiseAmplitude <= 0)
            return value;

        // Apply small random noise to simulate real servo jitter
        double noise = (_random.NextDouble() - 0.5) * 2 * _mockConfig.NoiseAmplitude * 10; // ±0.2° typical
        return value + noise;
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;

        _logger.LogInformation("🎮 Mock Servo Controller disposed");
    }
}

/// <summary>
/// Event args for servo position changes.
/// </summary>
public class ServoPositionsChangedEventArgs : EventArgs
{
    public double[] Positions { get; }
    public DateTimeOffset Timestamp { get; }

    public ServoPositionsChangedEventArgs(double[] positions, DateTimeOffset timestamp)
    {
        Positions = positions;
        Timestamp = timestamp;
    }
}
