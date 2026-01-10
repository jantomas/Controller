using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Models;
using Hexapod.Sensors.Abstractions;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Sensors.Mock;

/// <summary>
/// Mock GPS sensor for development and testing without hardware.
/// </summary>
public sealed class MockGpsSensor : IGpsSensor
{
    private readonly ILogger<MockGpsSensor> _logger;
    private readonly MockModeConfiguration _mockConfig;
    private readonly Random _random = new();
    private GeoPosition _currentPosition;
    private bool _initialized;

    public MockGpsSensor(
        IOptions<HexapodConfiguration> config,
        ILogger<MockGpsSensor> logger)
    {
        _logger = logger;
        _mockConfig = config.Value.MockMode;
        
        _currentPosition = new GeoPosition
        {
            Latitude = _mockConfig.SimulatedLatitude,
            Longitude = _mockConfig.SimulatedLongitude,
            Altitude = 100.0,
            Accuracy = 2.5,
            Timestamp = DateTimeOffset.UtcNow
        };
    }

    public SensorType Type => SensorType.Gps;
    public string Name => "Mock-GPS";
    public bool IsEnabled => true;
    public HealthStatus Health => _initialized ? HealthStatus.Healthy : HealthStatus.Unknown;
    public bool HasFix => true;
    public int SatellitesInView => 12;
    public double Hdop => 1.2;

    public Task InitializeAsync(CancellationToken cancellationToken = default)
    {
        _initialized = true;
        _logger.LogInformation("🛰️ Mock GPS sensor initialized at {Lat:F6}, {Lon:F6}",
            _currentPosition.Latitude, _currentPosition.Longitude);
        return Task.CompletedTask;
    }

    public Task ShutdownAsync(CancellationToken cancellationToken = default)
    {
        _initialized = false;
        _logger.LogInformation("🛰️ Mock GPS sensor shutdown");
        return Task.CompletedTask;
    }

    public Task<bool> SelfTestAsync(CancellationToken cancellationToken = default)
    {
        _logger.LogDebug("🛰️ Mock GPS self-test passed");
        return Task.FromResult(true);
    }

    public Task<GeoPosition?> GetPositionAsync(CancellationToken cancellationToken = default)
    {
        // Simulate small position drift
        if (_mockConfig.SimulateNoise)
        {
            var latDrift = (_random.NextDouble() - 0.5) * 0.00001 * _mockConfig.NoiseAmplitude;
            var lonDrift = (_random.NextDouble() - 0.5) * 0.00001 * _mockConfig.NoiseAmplitude;
            
            _currentPosition = _currentPosition with
            {
                Latitude = _mockConfig.SimulatedLatitude + latDrift,
                Longitude = _mockConfig.SimulatedLongitude + lonDrift,
                Timestamp = DateTimeOffset.UtcNow
            };
        }

        if (_mockConfig.VerboseLogging)
        {
            _logger.LogDebug("🛰️ GPS Position: {Lat:F6}, {Lon:F6}",
                _currentPosition.Latitude, _currentPosition.Longitude);
        }

        return Task.FromResult<GeoPosition?>(_currentPosition);
    }
}

/// <summary>
/// Mock IMU sensor for development and testing without hardware.
/// </summary>
public sealed class MockImuSensor : IImuSensor
{
    private readonly ILogger<MockImuSensor> _logger;
    private readonly MockModeConfiguration _mockConfig;
    private readonly Random _random = new();
    private bool _initialized;
    private bool _calibrated;

    // Simulated orientation state
    private double _roll;
    private double _pitch;
    private double _yaw;

    public MockImuSensor(
        IOptions<HexapodConfiguration> config,
        ILogger<MockImuSensor> logger)
    {
        _logger = logger;
        _mockConfig = config.Value.MockMode;
    }

    public SensorType Type => SensorType.Imu;
    public string Name => "Mock-IMU";
    public bool IsEnabled => true;
    public HealthStatus Health => _initialized ? HealthStatus.Healthy : HealthStatus.Unknown;
    public bool IsCalibrated => _calibrated;

    public Task InitializeAsync(CancellationToken cancellationToken = default)
    {
        _initialized = true;
        _logger.LogInformation("📐 Mock IMU sensor initialized");
        return Task.CompletedTask;
    }

    public Task ShutdownAsync(CancellationToken cancellationToken = default)
    {
        _initialized = false;
        _logger.LogInformation("📐 Mock IMU sensor shutdown");
        return Task.CompletedTask;
    }

    public Task<bool> SelfTestAsync(CancellationToken cancellationToken = default)
    {
        _logger.LogDebug("📐 Mock IMU self-test passed");
        return Task.FromResult(true);
    }

    public Task<bool> CalibrateAsync(CancellationToken cancellationToken = default)
    {
        _calibrated = true;
        _logger.LogInformation("📐 Mock IMU calibration completed");
        return Task.FromResult(true);
    }

    public Task<Orientation> GetOrientationAsync(CancellationToken cancellationToken = default)
    {
        // Add small noise to simulate sensor drift
        if (_mockConfig.SimulateNoise)
        {
            _roll += (_random.NextDouble() - 0.5) * _mockConfig.NoiseAmplitude * 2;
            _pitch += (_random.NextDouble() - 0.5) * _mockConfig.NoiseAmplitude * 2;
            _yaw += (_random.NextDouble() - 0.5) * _mockConfig.NoiseAmplitude;

            // Keep values in reasonable bounds
            _roll = Math.Clamp(_roll, -5, 5);
            _pitch = Math.Clamp(_pitch, -5, 5);
            _yaw = (_yaw + 360) % 360;
        }

        var orientation = new Orientation
        {
            Roll = _roll,
            Pitch = _pitch,
            Yaw = _yaw,
            Timestamp = DateTimeOffset.UtcNow
        };

        if (_mockConfig.VerboseLogging)
        {
            _logger.LogDebug("📐 Orientation: Roll={Roll:F2}° Pitch={Pitch:F2}° Yaw={Yaw:F2}°",
                _roll, _pitch, _yaw);
        }

        return Task.FromResult(orientation);
    }

    public Task<Acceleration> GetAccelerationAsync(CancellationToken cancellationToken = default)
    {
        var noise = _mockConfig.SimulateNoise ? _mockConfig.NoiseAmplitude : 0;
        
        var acceleration = new Acceleration
        {
            X = (_random.NextDouble() - 0.5) * noise,
            Y = (_random.NextDouble() - 0.5) * noise,
            Z = 9.81 + (_random.NextDouble() - 0.5) * noise,
            Timestamp = DateTimeOffset.UtcNow
        };

        return Task.FromResult(acceleration);
    }

    public Task<Acceleration> GetAngularVelocityAsync(CancellationToken cancellationToken = default)
    {
        var noise = _mockConfig.SimulateNoise ? _mockConfig.NoiseAmplitude * 10 : 0;
        
        var angularVelocity = new Acceleration
        {
            X = (_random.NextDouble() - 0.5) * noise,
            Y = (_random.NextDouble() - 0.5) * noise,
            Z = (_random.NextDouble() - 0.5) * noise,
            Timestamp = DateTimeOffset.UtcNow
        };

        return Task.FromResult(angularVelocity);
    }

    /// <summary>
    /// Sets the simulated orientation (for testing scenarios).
    /// </summary>
    public void SetSimulatedOrientation(double roll, double pitch, double yaw)
    {
        _roll = roll;
        _pitch = pitch;
        _yaw = yaw;
        _logger.LogDebug("📐 Orientation set to Roll={Roll}° Pitch={Pitch}° Yaw={Yaw}°",
            roll, pitch, yaw);
    }
}

/// <summary>
/// Mock power sensor for development and testing without hardware.
/// </summary>
public sealed class MockPowerSensor : IPowerSensor
{
    private readonly ILogger<MockPowerSensor> _logger;
    private readonly MockModeConfiguration _mockConfig;
    private readonly Random _random = new();
    private bool _initialized;
    private double _batteryPercent;
    private double _voltage;
    private readonly DateTimeOffset _startTime = DateTimeOffset.UtcNow;

    public MockPowerSensor(
        IOptions<HexapodConfiguration> config,
        ILogger<MockPowerSensor> logger)
    {
        _logger = logger;
        _mockConfig = config.Value.MockMode;
        _batteryPercent = _mockConfig.SimulatedBatteryPercent;
        _voltage = _mockConfig.SimulatedBatteryVoltage;
    }

    public SensorType Type => SensorType.Current;
    public string Name => "Mock-Power";
    public bool IsEnabled => true;
    public HealthStatus Health => _initialized ? HealthStatus.Healthy : HealthStatus.Unknown;
    public double CurrentVoltage => _voltage;
    public double CurrentAmperage => 1.5 + (_random.NextDouble() - 0.5) * 0.3;
    public TimeSpan EstimatedRuntime => TimeSpan.FromMinutes(_batteryPercent * 1.2);

    public Task InitializeAsync(CancellationToken cancellationToken = default)
    {
        _initialized = true;
        _logger.LogInformation("🔋 Mock Power sensor initialized - Battery: {Percent:F1}%",
            _batteryPercent);
        return Task.CompletedTask;
    }

    public Task ShutdownAsync(CancellationToken cancellationToken = default)
    {
        _initialized = false;
        _logger.LogInformation("🔋 Mock Power sensor shutdown");
        return Task.CompletedTask;
    }

    public Task<bool> SelfTestAsync(CancellationToken cancellationToken = default)
    {
        _logger.LogDebug("🔋 Mock Power self-test passed");
        return Task.FromResult(true);
    }

    public Task<PowerStatus> GetStatusAsync(CancellationToken cancellationToken = default)
    {
        // Simulate slow battery drain (0.01% per second in mock)
        var elapsedSeconds = (DateTimeOffset.UtcNow - _startTime).TotalSeconds;
        _batteryPercent = Math.Max(0, _mockConfig.SimulatedBatteryPercent - elapsedSeconds * 0.01);
        
        // Voltage correlates with battery percentage
        _voltage = 9.0 + (_batteryPercent / 100.0) * 3.6;

        if (_mockConfig.SimulateNoise)
        {
            _voltage += (_random.NextDouble() - 0.5) * 0.1;
        }

        var status = new PowerStatus
        {
            BatteryVoltage = _voltage,
            BatteryCurrent = CurrentAmperage,
            BatteryPercentage = _batteryPercent,
            PowerConsumption = _voltage * CurrentAmperage,
            EstimatedRuntime = EstimatedRuntime,
            IsCharging = false,
            Timestamp = DateTimeOffset.UtcNow
        };

        if (_mockConfig.VerboseLogging)
        {
            _logger.LogDebug("🔋 Battery: {Percent:F1}% @ {Voltage:F2}V",
                _batteryPercent, _voltage);
        }

        return Task.FromResult(status);
    }

    /// <summary>
    /// Sets the simulated battery level (for testing scenarios).
    /// </summary>
    public void SetBatteryLevel(double percentage, double voltage)
    {
        _batteryPercent = Math.Clamp(percentage, 0, 100);
        _voltage = voltage;
        _logger.LogDebug("🔋 Battery level set to {Percent}% @ {Voltage}V",
            percentage, voltage);
    }
}

/// <summary>
/// Mock distance sensor array for development and testing without hardware.
/// </summary>
public sealed class MockDistanceSensorArray : IDistanceSensorArray
{
    private readonly ILogger<MockDistanceSensorArray> _logger;
    private readonly MockModeConfiguration _mockConfig;
    private readonly Random _random = new();
    private readonly double[] _distances = new double[6];
    private readonly string[] _sensorNames = { "Front-Left", "Front", "Front-Right", "Rear-Right", "Rear", "Rear-Left" };

    public MockDistanceSensorArray(
        IOptions<HexapodConfiguration> config,
        ILogger<MockDistanceSensorArray> logger)
    {
        _logger = logger;
        _mockConfig = config.Value.MockMode;

        // Initialize with "clear" readings (2 meters)
        for (int i = 0; i < 6; i++)
            _distances[i] = 2.0;

        _logger.LogInformation("📏 Mock Distance Sensor Array initialized - 6 sensors");
    }

    public Task<IReadOnlyList<DistanceReading>> GetAllReadingsAsync(CancellationToken cancellationToken = default)
    {
        var readings = new List<DistanceReading>();

        for (int i = 0; i < 6; i++)
        {
            // Add noise to distance readings
            var noise = _mockConfig.SimulateNoise
                ? (_random.NextDouble() - 0.5) * _mockConfig.NoiseAmplitude * 0.1
                : 0;

            readings.Add(new DistanceReading
            {
                SensorId = i,
                SensorName = _sensorNames[i],
                Distance = Math.Max(0.01, _distances[i] + noise),
                Confidence = 0.95 + _random.NextDouble() * 0.05,
                Timestamp = DateTimeOffset.UtcNow
            });
        }

        if (_mockConfig.VerboseLogging)
        {
            _logger.LogDebug("📏 Distances: {Distances}",
                string.Join(", ", readings.Select(r => $"{r.SensorName}={r.Distance:F2}m")));
        }

        return Task.FromResult<IReadOnlyList<DistanceReading>>(readings);
    }

    public async Task<DistanceReading?> GetMinimumDistanceAsync(CancellationToken cancellationToken = default)
    {
        var readings = await GetAllReadingsAsync(cancellationToken);
        return readings.MinBy(r => r.Distance);
    }

    public async Task<bool> IsObstacleWithinAsync(double distance, CancellationToken cancellationToken = default)
    {
        var min = await GetMinimumDistanceAsync(cancellationToken);
        return min != null && min.Distance < distance;
    }

    /// <summary>
    /// Sets simulated obstacle distance for a specific sensor (for testing).
    /// </summary>
    public void SetObstacleDistance(int sensorId, double distance)
    {
        if (sensorId >= 0 && sensorId < 6)
        {
            _distances[sensorId] = distance;
            _logger.LogDebug("📏 Set {Sensor} distance to {Distance:F2}m",
                _sensorNames[sensorId], distance);
        }
    }

    /// <summary>
    /// Simulates an obstacle approaching from the front (for testing).
    /// </summary>
    public void SimulateFrontObstacle(double distance)
    {
        _distances[0] = distance * 1.2; // Front-Left
        _distances[1] = distance;        // Front
        _distances[2] = distance * 1.2; // Front-Right
        _logger.LogInformation("📏 Simulating front obstacle at {Distance:F2}m", distance);
    }
}

/// <summary>
/// Mock touch sensor array for development and testing without hardware.
/// </summary>
public sealed class MockTouchSensorArray : ITouchSensorArray
{
    private readonly ILogger<MockTouchSensorArray> _logger;
    private readonly MockModeConfiguration _mockConfig;
    private readonly Random _random = new();
    private readonly bool[] _grounded = new bool[6];
    private readonly double[] _forces = new double[6];

    public MockTouchSensorArray(
        IOptions<HexapodConfiguration> config,
        ILogger<MockTouchSensorArray> logger)
    {
        _logger = logger;
        _mockConfig = config.Value.MockMode;

        // Initialize with all legs grounded (standing position)
        for (int i = 0; i < 6; i++)
        {
            _grounded[i] = true;
            _forces[i] = 2.5; // ~2.5N per leg when standing
        }

        _logger.LogInformation("👆 Mock Touch Sensor Array initialized - 6 sensors (all grounded)");
    }

    public Task<IReadOnlyList<TouchReading>> GetAllReadingsAsync(CancellationToken cancellationToken = default)
    {
        var readings = new List<TouchReading>();

        for (int i = 0; i < 6; i++)
        {
            var noise = _mockConfig.SimulateNoise
                ? (_random.NextDouble() - 0.5) * _mockConfig.NoiseAmplitude * 0.5
                : 0;

            readings.Add(new TouchReading
            {
                LegId = i,
                Force = Math.Max(0, _grounded[i] ? _forces[i] + noise : 0),
                IsGrounded = _grounded[i],
                Timestamp = DateTimeOffset.UtcNow
            });
        }

        return Task.FromResult<IReadOnlyList<TouchReading>>(readings);
    }

    public async Task<int> GetGroundedLegCountAsync(CancellationToken cancellationToken = default)
    {
        var readings = await GetAllReadingsAsync(cancellationToken);
        return readings.Count(r => r.IsGrounded);
    }

    public async Task<IReadOnlyList<int>> GetGroundedLegIdsAsync(CancellationToken cancellationToken = default)
    {
        var readings = await GetAllReadingsAsync(cancellationToken);
        return readings.Where(r => r.IsGrounded).Select(r => r.LegId).ToList();
    }

    /// <summary>
    /// Sets the grounded state for a specific leg (for testing gait patterns).
    /// </summary>
    public void SetLegGrounded(int legId, bool grounded, double force = 2.5)
    {
        if (legId >= 0 && legId < 6)
        {
            _grounded[legId] = grounded;
            _forces[legId] = grounded ? force : 0;
            _logger.LogDebug("👆 Leg {LegId} set to {State}", legId, grounded ? "GROUNDED" : "RAISED");
        }
    }

    /// <summary>
    /// Simulates tripod gait stance (alternating legs).
    /// </summary>
    public void SimulateTripodStance(bool phase)
    {
        // Phase A: legs 0, 2, 4 grounded
        // Phase B: legs 1, 3, 5 grounded
        for (int i = 0; i < 6; i++)
        {
            bool shouldBeGrounded = phase ? (i % 2 == 0) : (i % 2 == 1);
            _grounded[i] = shouldBeGrounded;
            _forces[i] = shouldBeGrounded ? 3.5 : 0; // Higher force when fewer legs on ground
        }
        _logger.LogDebug("👆 Tripod stance phase {Phase}: {Grounded}",
            phase ? "A" : "B",
            string.Join(", ", Enumerable.Range(0, 6).Where(i => _grounded[i])));
    }
}
