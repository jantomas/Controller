namespace Hexapod.Core.Enums;

/// <summary>
/// Defines the operational modes of the hexapod device.
/// </summary>
public enum OperationMode
{
    /// <summary>
    /// System is initializing and performing self-checks.
    /// </summary>
    Initializing = 0,

    /// <summary>
    /// Full autonomous operation with predefined mission parameters.
    /// All decisions are made by the onboard AI without operator intervention.
    /// </summary>
    Autonomous = 1,

    /// <summary>
    /// Default mode. Autonomous navigation with operator confirmation required
    /// for irreversible actions, high-risk maneuvers, and uncertain situations.
    /// </summary>
    SemiAutonomous = 2,

    /// <summary>
    /// Direct operator control via remote connection.
    /// Used for emergency situations or complex manual operations.
    /// </summary>
    RemoteControl = 3,

    /// <summary>
    /// Minimal operation mode when critical errors occur or communication is lost.
    /// Device maintains position and waits for recovery.
    /// </summary>
    SafeMode = 4,

    /// <summary>
    /// Device is in low-power standby mode.
    /// </summary>
    Standby = 5,

    /// <summary>
    /// Emergency stop - all movement halted immediately.
    /// </summary>
    EmergencyStop = 6
}

/// <summary>
/// Defines the movement states of the hexapod.
/// </summary>
public enum MovementState
{
    Idle = 0,
    Walking = 1,
    Turning = 2,
    Climbing = 3,
    Descending = 4,
    Recovering = 5,
    Positioning = 6
}

/// <summary>
/// Defines the gait patterns available for locomotion.
/// </summary>
public enum GaitType
{
    /// <summary>
    /// Tripod gait - fastest, three legs move simultaneously.
    /// Best for flat terrain and speed priority.
    /// </summary>
    Tripod = 0,

    /// <summary>
    /// Wave gait - most stable, legs move in sequence.
    /// Best for rough terrain and stability priority.
    /// </summary>
    Wave = 1,

    /// <summary>
    /// Ripple gait - balanced between speed and stability.
    /// Good general-purpose gait.
    /// </summary>
    Ripple = 2,

    /// <summary>
    /// Metachronal gait - smooth, energy-efficient.
    /// Best for sustained travel and energy conservation.
    /// </summary>
    Metachronal = 3
}

/// <summary>
/// Defines risk levels for autonomous decisions.
/// </summary>
public enum RiskLevel
{
    /// <summary>
    /// No risk - action can proceed automatically.
    /// </summary>
    None = 0,

    /// <summary>
    /// Low risk - log action, no confirmation needed.
    /// </summary>
    Low = 1,

    /// <summary>
    /// Medium risk - confirmation may be requested based on settings.
    /// </summary>
    Medium = 2,

    /// <summary>
    /// High risk - operator confirmation required in semi-autonomous mode.
    /// </summary>
    High = 3,

    /// <summary>
    /// Critical risk - always requires operator confirmation or abort.
    /// </summary>
    Critical = 4
}

/// <summary>
/// Defines the health status of system components.
/// </summary>
public enum HealthStatus
{
    Unknown = 0,
    Healthy = 1,
    Degraded = 2,
    Unhealthy = 3,
    Critical = 4,
    Offline = 5
}

/// <summary>
/// Defines sensor types available in the system.
/// </summary>
public enum SensorType
{
    Camera = 0,
    Gps = 1,
    Imu = 2,
    Distance = 3,
    Touch = 4,
    Current = 5,
    Temperature = 6,
    Voltage = 7
}

/// <summary>
/// Defines terrain types that can be detected.
/// </summary>
public enum TerrainType
{
    Unknown = 0,
    Flat = 1,
    Rough = 2,
    Slope = 3,
    Stairs = 4,
    Water = 5,
    Obstacle = 6,
    Vegetation = 7,
    Sand = 8
}
