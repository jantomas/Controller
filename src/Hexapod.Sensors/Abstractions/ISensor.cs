using Hexapod.Core.Enums;
using Hexapod.Core.Models;

namespace Hexapod.Sensors.Abstractions;

/// <summary>
/// Base interface for all sensors.
/// </summary>
public interface ISensor
{
    /// <summary>
    /// Gets the sensor type.
    /// </summary>
    SensorType Type { get; }

    /// <summary>
    /// Gets the sensor name.
    /// </summary>
    string Name { get; }

    /// <summary>
    /// Gets whether the sensor is enabled.
    /// </summary>
    bool IsEnabled { get; }

    /// <summary>
    /// Gets the current health status of the sensor.
    /// </summary>
    HealthStatus Health { get; }

    /// <summary>
    /// Initializes the sensor hardware.
    /// </summary>
    Task InitializeAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Shuts down the sensor.
    /// </summary>
    Task ShutdownAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Performs a self-test on the sensor.
    /// </summary>
    Task<bool> SelfTestAsync(CancellationToken cancellationToken = default);
}

/// <summary>
/// Interface for GPS sensor.
/// </summary>
public interface IGpsSensor : ISensor
{
    /// <summary>
    /// Gets the current GPS position.
    /// </summary>
    Task<GeoPosition?> GetPositionAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets whether the GPS has a valid fix.
    /// </summary>
    bool HasFix { get; }

    /// <summary>
    /// Gets the number of satellites in view.
    /// </summary>
    int SatellitesInView { get; }

    /// <summary>
    /// Gets the horizontal dilution of precision.
    /// </summary>
    double Hdop { get; }
}

/// <summary>
/// Interface for IMU (Inertial Measurement Unit) sensor.
/// </summary>
public interface IImuSensor : ISensor
{
    /// <summary>
    /// Gets the current orientation.
    /// </summary>
    Task<Orientation> GetOrientationAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the current acceleration.
    /// </summary>
    Task<Acceleration> GetAccelerationAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the angular velocity (gyroscope).
    /// </summary>
    Task<Acceleration> GetAngularVelocityAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Calibrates the IMU.
    /// </summary>
    Task<bool> CalibrateAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets whether the sensor is calibrated.
    /// </summary>
    bool IsCalibrated { get; }
}

/// <summary>
/// Interface for distance sensors (ToF, ultrasonic, etc.).
/// </summary>
public interface IDistanceSensor : ISensor
{
    /// <summary>
    /// Gets the sensor ID in the array.
    /// </summary>
    int SensorId { get; }

    /// <summary>
    /// Gets the current distance reading.
    /// </summary>
    Task<DistanceReading> GetDistanceAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the maximum range of the sensor.
    /// </summary>
    double MaxRange { get; }
}

/// <summary>
/// Interface for touch/force sensors.
/// </summary>
public interface ITouchSensor : ISensor
{
    /// <summary>
    /// Gets the leg ID this sensor is attached to.
    /// </summary>
    int LegId { get; }

    /// <summary>
    /// Gets the current touch reading.
    /// </summary>
    Task<TouchReading> GetReadingAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Sets the force threshold for ground detection.
    /// </summary>
    void SetThreshold(double threshold);
}

/// <summary>
/// Interface for power monitoring sensor.
/// </summary>
public interface IPowerSensor : ISensor
{
    /// <summary>
    /// Gets the current power status.
    /// </summary>
    Task<PowerStatus> GetStatusAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the current voltage.
    /// </summary>
    double CurrentVoltage { get; }

    /// <summary>
    /// Gets the current amperage.
    /// </summary>
    double CurrentAmperage { get; }

    /// <summary>
    /// Estimates the remaining runtime.
    /// </summary>
    TimeSpan EstimatedRuntime { get; }
}

/// <summary>
/// Interface for the distance sensor array manager.
/// </summary>
public interface IDistanceSensorArray
{
    /// <summary>
    /// Gets readings from all distance sensors.
    /// </summary>
    Task<IReadOnlyList<DistanceReading>> GetAllReadingsAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the minimum distance detected.
    /// </summary>
    Task<DistanceReading?> GetMinimumDistanceAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets whether any obstacle is within the specified distance.
    /// </summary>
    Task<bool> IsObstacleWithinAsync(double distance, CancellationToken cancellationToken = default);
}

/// <summary>
/// Interface for the touch sensor array manager.
/// </summary>
public interface ITouchSensorArray
{
    /// <summary>
    /// Gets readings from all touch sensors.
    /// </summary>
    Task<IReadOnlyList<TouchReading>> GetAllReadingsAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the number of legs currently on the ground.
    /// </summary>
    Task<int> GetGroundedLegCountAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the IDs of grounded legs.
    /// </summary>
    Task<IReadOnlyList<int>> GetGroundedLegIdsAsync(CancellationToken cancellationToken = default);
}
