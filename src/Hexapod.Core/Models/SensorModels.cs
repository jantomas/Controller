using System.Numerics;

namespace Hexapod.Core.Models;

/// <summary>
/// Represents a geographic position with GPS coordinates.
/// </summary>
public record GeoPosition
{
    public double Latitude { get; init; }
    public double Longitude { get; init; }
    public double Altitude { get; init; }
    public double Accuracy { get; init; }
    public DateTimeOffset Timestamp { get; init; }

    /// <summary>
    /// Calculates the distance in meters to another position using Haversine formula.
    /// </summary>
    public double DistanceTo(GeoPosition other)
    {
        const double R = 6371000; // Earth's radius in meters
        var dLat = ToRadians(other.Latitude - Latitude);
        var dLon = ToRadians(other.Longitude - Longitude);
        var a = Math.Sin(dLat / 2) * Math.Sin(dLat / 2) +
                Math.Cos(ToRadians(Latitude)) * Math.Cos(ToRadians(other.Latitude)) *
                Math.Sin(dLon / 2) * Math.Sin(dLon / 2);
        var c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
        return R * c;
    }

    private static double ToRadians(double degrees) => degrees * Math.PI / 180.0;
}

/// <summary>
/// Represents the orientation of the device using Euler angles.
/// </summary>
public record Orientation
{
    /// <summary>Rotation around X-axis (side-to-side tilt) in degrees.</summary>
    public double Roll { get; init; }
    
    /// <summary>Rotation around Y-axis (forward-backward tilt) in degrees.</summary>
    public double Pitch { get; init; }
    
    /// <summary>Rotation around Z-axis (compass heading) in degrees.</summary>
    public double Yaw { get; init; }

    public DateTimeOffset Timestamp { get; init; }

    /// <summary>
    /// Converts to a quaternion representation.
    /// </summary>
    public Quaternion ToQuaternion()
    {
        var cy = Math.Cos(ToRadians(Yaw) * 0.5);
        var sy = Math.Sin(ToRadians(Yaw) * 0.5);
        var cp = Math.Cos(ToRadians(Pitch) * 0.5);
        var sp = Math.Sin(ToRadians(Pitch) * 0.5);
        var cr = Math.Cos(ToRadians(Roll) * 0.5);
        var sr = Math.Sin(ToRadians(Roll) * 0.5);

        return new Quaternion(
            (float)(sr * cp * cy - cr * sp * sy),
            (float)(cr * sp * cy + sr * cp * sy),
            (float)(cr * cp * sy - sr * sp * cy),
            (float)(cr * cp * cy + sr * sp * sy));
    }

    private static double ToRadians(double degrees) => degrees * Math.PI / 180.0;
}

/// <summary>
/// Represents 3D acceleration data from the IMU.
/// </summary>
public record Acceleration
{
    public double X { get; init; }
    public double Y { get; init; }
    public double Z { get; init; }
    public DateTimeOffset Timestamp { get; init; }

    public double Magnitude => Math.Sqrt(X * X + Y * Y + Z * Z);
}

/// <summary>
/// Represents power status of the device.
/// </summary>
public record PowerStatus
{
    public double BatteryVoltage { get; init; }
    public double BatteryCurrent { get; init; }
    public double BatteryPercentage { get; init; }
    public double PowerConsumption { get; init; }
    public TimeSpan EstimatedRuntime { get; init; }
    public bool IsCharging { get; init; }
    public DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Represents a distance reading from a ToF sensor.
/// </summary>
public record DistanceReading
{
    public int SensorId { get; init; }
    public string SensorName { get; init; } = string.Empty;
    public double Distance { get; init; } // in meters
    public double Confidence { get; init; } // 0.0 to 1.0
    public DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Represents a touch/force sensor reading.
/// </summary>
public record TouchReading
{
    public int LegId { get; init; }
    public double Force { get; init; } // in Newtons
    public bool IsGrounded { get; init; }
    public DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Represents the complete sensor state at a point in time.
/// </summary>
public record SensorState
{
    public GeoPosition? Position { get; init; }
    public Orientation? Orientation { get; init; }
    public Acceleration? Acceleration { get; init; }
    public PowerStatus? PowerStatus { get; init; }
    public IReadOnlyList<DistanceReading> DistanceReadings { get; init; } = Array.Empty<DistanceReading>();
    public IReadOnlyList<TouchReading> TouchReadings { get; init; } = Array.Empty<TouchReading>();
    public DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Represents the position of a single leg joint.
/// </summary>
public record JointPosition
{
    public int LegId { get; init; }
    public int JointId { get; init; }
    public double Angle { get; init; } // in degrees
    public double TargetAngle { get; init; }
    public double Velocity { get; init; }
    public double Torque { get; init; }
}

/// <summary>
/// Represents a leg endpoint position in 3D space.
/// </summary>
public record LegEndpoint
{
    public int LegId { get; init; }
    public Vector3 Position { get; init; }
    public bool IsGrounded { get; init; }
}

/// <summary>
/// Compact status report for communication.
/// </summary>
public record StatusReport
{
    public required byte BatteryPercent { get; init; }
    public required Hexapod.Core.Enums.OperationMode CurrentMode { get; init; }
    public required Hexapod.Core.Enums.MovementState CurrentState { get; init; }
    public required StatusFlags Flags { get; init; }
}

/// <summary>
/// Status flags for compact status reporting.
/// </summary>
[Flags]
public enum StatusFlags : byte
{
    None = 0,
    CloudConnected = 1,
    MissionActive = 2,
    ObstacleDetected = 4,
    LowBattery = 8,
    Charging = 16,
    ErrorState = 32,
    EmergencyStop = 64,
    GpsLocked = 128
}
