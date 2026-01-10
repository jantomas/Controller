using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Events;
using Hexapod.Core.Models;
using Hexapod.Core.Services;
using Hexapod.Sensors.Abstractions;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Sensors.Services;

/// <summary>
/// Central service that coordinates all sensors and provides unified sensor data.
/// </summary>
public interface ISensorManager
{
    /// <summary>
    /// Gets the current aggregated sensor state.
    /// </summary>
    Task<SensorState> GetSensorStateAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Initializes all sensors.
    /// </summary>
    Task InitializeAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Shuts down all sensors.
    /// </summary>
    Task ShutdownAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the health status of all sensors.
    /// </summary>
    IReadOnlyDictionary<string, HealthStatus> GetHealthStatus();

    /// <summary>
    /// Runs self-tests on all sensors.
    /// </summary>
    Task<IReadOnlyDictionary<string, bool>> RunSelfTestsAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the GPS sensor.
    /// </summary>
    IGpsSensor? Gps { get; }

    /// <summary>
    /// Gets the IMU sensor.
    /// </summary>
    IImuSensor? Imu { get; }

    /// <summary>
    /// Gets the power sensor.
    /// </summary>
    IPowerSensor? Power { get; }

    /// <summary>
    /// Gets the distance sensor array.
    /// </summary>
    IDistanceSensorArray? DistanceSensors { get; }

    /// <summary>
    /// Gets the touch sensor array.
    /// </summary>
    ITouchSensorArray? TouchSensors { get; }
}

/// <summary>
/// Implementation of the sensor manager.
/// </summary>
public sealed class SensorManager : ISensorManager, IDisposable
{
    private readonly ILogger<SensorManager> _logger;
    private readonly IEventBus _eventBus;
    private readonly SensorConfiguration _config;
    private readonly Dictionary<string, ISensor> _sensors = new();
    private readonly CancellationTokenSource _cts = new();
    private Task? _pollingTask;
    private SensorState? _lastState;

    public IGpsSensor? Gps { get; private set; }
    public IImuSensor? Imu { get; private set; }
    public IPowerSensor? Power { get; private set; }
    public IDistanceSensorArray? DistanceSensors { get; private set; }
    public ITouchSensorArray? TouchSensors { get; private set; }

    public SensorManager(
        IEventBus eventBus,
        IOptions<HexapodConfiguration> config,
        IGpsSensor? gps,
        IImuSensor? imu,
        IPowerSensor? power,
        IDistanceSensorArray? distanceSensors,
        ITouchSensorArray? touchSensors,
        ILogger<SensorManager> logger)
    {
        _eventBus = eventBus;
        _config = config.Value.Sensors;
        _logger = logger;

        Gps = gps;
        Imu = imu;
        Power = power;
        DistanceSensors = distanceSensors;
        TouchSensors = touchSensors;

        // Register sensors
        if (gps != null) _sensors[gps.Name] = gps;
        if (imu != null) _sensors[imu.Name] = imu;
        if (power != null) _sensors[power.Name] = power;
    }

    public async Task InitializeAsync(CancellationToken cancellationToken = default)
    {
        _logger.LogInformation("Initializing sensor manager...");

        var initTasks = _sensors.Values
            .Where(s => s.IsEnabled)
            .Select(async sensor =>
            {
                try
                {
                    await sensor.InitializeAsync(cancellationToken);
                    _logger.LogInformation("Sensor {Name} initialized", sensor.Name);
                }
                catch (Exception ex)
                {
                    _logger.LogError(ex, "Failed to initialize sensor {Name}", sensor.Name);
                }
            });

        await Task.WhenAll(initTasks);

        // Start sensor polling
        _pollingTask = Task.Run(() => PollSensorsAsync(_cts.Token), cancellationToken);

        _logger.LogInformation("Sensor manager initialized with {Count} sensors", _sensors.Count);
    }

    public async Task ShutdownAsync(CancellationToken cancellationToken = default)
    {
        _logger.LogInformation("Shutting down sensor manager...");

        _cts.Cancel();

        if (_pollingTask != null)
        {
            try
            {
                await _pollingTask.WaitAsync(TimeSpan.FromSeconds(5), cancellationToken);
            }
            catch (TimeoutException)
            {
                _logger.LogWarning("Sensor polling task did not complete in time");
            }
        }

        foreach (var sensor in _sensors.Values)
        {
            try
            {
                await sensor.ShutdownAsync(cancellationToken);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error shutting down sensor {Name}", sensor.Name);
            }
        }

        _logger.LogInformation("Sensor manager shut down");
    }

    public async Task<SensorState> GetSensorStateAsync(CancellationToken cancellationToken = default)
    {
        var tasks = new List<Task>();
        
        GeoPosition? position = null;
        Orientation? orientation = null;
        Acceleration? acceleration = null;
        PowerStatus? powerStatus = null;
        IReadOnlyList<DistanceReading> distanceReadings = Array.Empty<DistanceReading>();
        IReadOnlyList<TouchReading> touchReadings = Array.Empty<TouchReading>();

        // Gather sensor data in parallel
        if (Gps != null)
        {
            tasks.Add(Task.Run(async () => position = await Gps.GetPositionAsync(cancellationToken), cancellationToken));
        }

        if (Imu != null)
        {
            tasks.Add(Task.Run(async () =>
            {
                orientation = await Imu.GetOrientationAsync(cancellationToken);
                acceleration = await Imu.GetAccelerationAsync(cancellationToken);
            }, cancellationToken));
        }

        if (Power != null)
        {
            tasks.Add(Task.Run(async () => powerStatus = await Power.GetStatusAsync(cancellationToken), cancellationToken));
        }

        if (DistanceSensors != null)
        {
            tasks.Add(Task.Run(async () => distanceReadings = await DistanceSensors.GetAllReadingsAsync(cancellationToken), cancellationToken));
        }

        if (TouchSensors != null)
        {
            tasks.Add(Task.Run(async () => touchReadings = await TouchSensors.GetAllReadingsAsync(cancellationToken), cancellationToken));
        }

        await Task.WhenAll(tasks);

        _lastState = new SensorState
        {
            Position = position,
            Orientation = orientation,
            Acceleration = acceleration,
            PowerStatus = powerStatus,
            DistanceReadings = distanceReadings,
            TouchReadings = touchReadings,
            Timestamp = DateTimeOffset.UtcNow
        };

        return _lastState;
    }

    public IReadOnlyDictionary<string, HealthStatus> GetHealthStatus()
    {
        return _sensors.ToDictionary(
            kvp => kvp.Key,
            kvp => kvp.Value.Health);
    }

    public async Task<IReadOnlyDictionary<string, bool>> RunSelfTestsAsync(CancellationToken cancellationToken = default)
    {
        var results = new Dictionary<string, bool>();

        foreach (var sensor in _sensors.Values)
        {
            try
            {
                var result = await sensor.SelfTestAsync(cancellationToken);
                results[sensor.Name] = result;
                _logger.LogInformation("Sensor {Name} self-test: {Result}", 
                    sensor.Name, result ? "PASS" : "FAIL");
            }
            catch (Exception ex)
            {
                results[sensor.Name] = false;
                _logger.LogError(ex, "Sensor {Name} self-test failed with exception", sensor.Name);
            }
        }

        return results;
    }

    private async Task PollSensorsAsync(CancellationToken cancellationToken)
    {
        var pollInterval = TimeSpan.FromMilliseconds(50); // 20 Hz base polling rate
        var lastPowerCheck = DateTimeOffset.MinValue;
        var powerCheckInterval = TimeSpan.FromSeconds(5);

        while (!cancellationToken.IsCancellationRequested)
        {
            try
            {
                var state = await GetSensorStateAsync(cancellationToken);

                // Publish sensor updates
                if (state.Orientation != null)
                {
                    _eventBus.Publish(new SensorDataUpdatedEvent
                    {
                        EventId = Guid.NewGuid().ToString(),
                        Timestamp = DateTimeOffset.UtcNow,
                        Source = nameof(SensorManager),
                        SensorType = SensorType.Imu,
                        Data = state.Orientation
                    });
                }

                // Check power status periodically
                if (state.PowerStatus != null && 
                    DateTimeOffset.UtcNow - lastPowerCheck > powerCheckInterval)
                {
                    lastPowerCheck = DateTimeOffset.UtcNow;
                    CheckPowerStatus(state.PowerStatus);
                }

                // Check for obstacles
                await CheckForObstaclesAsync(state.DistanceReadings, cancellationToken);

                await Task.Delay(pollInterval, cancellationToken);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error in sensor polling loop");
                await Task.Delay(TimeSpan.FromSeconds(1), cancellationToken);
            }
        }
    }

    private void CheckPowerStatus(PowerStatus status)
    {
        var isLow = status.BatteryPercentage < _config.PowerMonitor.LowBatteryThreshold;
        var isCritical = status.BatteryPercentage < _config.PowerMonitor.CriticalBatteryThreshold;

        if (isLow || isCritical)
        {
            _eventBus.Publish(new PowerStatusChangedEvent
            {
                EventId = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow,
                Source = nameof(SensorManager),
                Status = status,
                IsLowPower = isLow,
                IsCritical = isCritical
            });

            if (isCritical)
            {
                _eventBus.Publish(new EmergencyEvent
                {
                    EventId = Guid.NewGuid().ToString(),
                    Timestamp = DateTimeOffset.UtcNow,
                    Source = nameof(SensorManager),
                    Type = EmergencyType.CriticalBattery,
                    Description = $"Critical battery level: {status.BatteryPercentage:F1}%",
                    RequiresImmediateAction = true
                });
            }
        }
    }

    private async Task CheckForObstaclesAsync(
        IReadOnlyList<DistanceReading> readings, 
        CancellationToken cancellationToken)
    {
        const double warningDistance = 0.3; // 30cm warning threshold

        foreach (var reading in readings)
        {
            if (reading.Distance < warningDistance && reading.Confidence > 0.8)
            {
                _eventBus.Publish(new ObstacleDetectedEvent
                {
                    EventId = Guid.NewGuid().ToString(),
                    Timestamp = DateTimeOffset.UtcNow,
                    Source = nameof(SensorManager),
                    Obstacle = new DetectedObject
                    {
                        Label = "Obstacle",
                        Confidence = reading.Confidence,
                        BoundingBox = new BoundingBox { X = 0, Y = 0, Width = 0, Height = 0 },
                        EstimatedDistance = reading.Distance,
                        IsObstacle = true,
                        IsHazard = reading.Distance < 0.15,
                        Timestamp = reading.Timestamp
                    },
                    Distance = reading.Distance,
                    Bearing = reading.SensorId * 60 // Approximate bearing based on sensor position
                });
            }
        }

        await Task.CompletedTask;
    }

    public void Dispose()
    {
        _cts.Cancel();
        _cts.Dispose();
        
        foreach (var sensor in _sensors.Values)
        {
            if (sensor is IDisposable disposable)
            {
                disposable.Dispose();
            }
        }
    }
}
