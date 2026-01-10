using Hexapod.Core.Enums;
using Hexapod.Core.Models;

namespace Hexapod.Core.Events;

/// <summary>
/// Base class for all system events.
/// </summary>
public abstract record HexapodEvent
{
    public required string EventId { get; init; }
    public required DateTimeOffset Timestamp { get; init; }
    public required string Source { get; init; }
}

/// <summary>
/// Event raised when the operation mode changes.
/// </summary>
public record OperationModeChangedEvent : HexapodEvent
{
    public required OperationMode PreviousMode { get; init; }
    public required OperationMode NewMode { get; init; }
    public required string Reason { get; init; }
}

/// <summary>
/// Event raised when the movement state changes.
/// </summary>
public record MovementStateChangedEvent : HexapodEvent
{
    public required MovementState PreviousState { get; init; }
    public required MovementState NewState { get; init; }
    public required GaitType Gait { get; init; }
}

/// <summary>
/// Event raised when sensor data is updated.
/// </summary>
public record SensorDataUpdatedEvent : HexapodEvent
{
    public required SensorType SensorType { get; init; }
    public required object Data { get; init; }
}

/// <summary>
/// Event raised when an obstacle is detected.
/// </summary>
public record ObstacleDetectedEvent : HexapodEvent
{
    public required DetectedObject Obstacle { get; init; }
    public required double Distance { get; init; }
    public required double Bearing { get; init; }
}

/// <summary>
/// Event raised when a decision requires operator confirmation.
/// </summary>
public record ConfirmationRequiredEvent : HexapodEvent
{
    public required Decision Decision { get; init; }
    public required TimeSpan Timeout { get; init; }
}

/// <summary>
/// Event raised when an operator responds to a confirmation request.
/// </summary>
public record OperatorResponseEvent : HexapodEvent
{
    public required string DecisionId { get; init; }
    public required string SelectedOptionId { get; init; }
    public string? Notes { get; init; }
}

/// <summary>
/// Event raised when the device reaches a waypoint.
/// </summary>
public record WaypointReachedEvent : HexapodEvent
{
    public required Waypoint Waypoint { get; init; }
    public required int RemainingWaypoints { get; init; }
}

/// <summary>
/// Event raised when mission status changes.
/// </summary>
public record MissionStatusChangedEvent : HexapodEvent
{
    public required string MissionId { get; init; }
    public MissionPhase PreviousPhase { get; init; }
    public MissionPhase NewPhase { get; init; }
    public string? PreviousStatus { get; init; }
    public string? NewStatus { get; init; }
    public double Progress { get; init; }
}

/// <summary>
/// Event raised when a command is received from the cloud.
/// </summary>
public record CommandReceivedEvent : HexapodEvent
{
    public required string CommandType { get; init; }
    public required string Payload { get; init; }
    public required string CorrelationId { get; init; }
}

/// <summary>
/// Event raised when the health status of a component changes.
/// </summary>
public record HealthStatusChangedEvent : HexapodEvent
{
    public required string ComponentName { get; init; }
    public required HealthStatus PreviousStatus { get; init; }
    public required HealthStatus NewStatus { get; init; }
    public string? Details { get; init; }
}

/// <summary>
/// Event raised when power status changes significantly.
/// </summary>
public record PowerStatusChangedEvent : HexapodEvent
{
    public required PowerStatus Status { get; init; }
    public required bool IsLowPower { get; init; }
    public required bool IsCritical { get; init; }
}

/// <summary>
/// Event raised for emergency situations.
/// </summary>
public record EmergencyEvent : HexapodEvent
{
    public required EmergencyType Type { get; init; }
    public required string Description { get; init; }
    public required bool RequiresImmediateAction { get; init; }
}

/// <summary>
/// Defines emergency types.
/// </summary>
public enum EmergencyType
{
    HardwareFault,
    CommunicationLost,
    LowBattery,
    CriticalBattery,
    CollisionImminent,
    TipOverRisk,
    EnvironmentalHazard,
    SecurityBreach,
    OperatorAlert
}

/// <summary>
/// Event raised when connectivity status changes.
/// </summary>
public record ConnectivityChangedEvent : HexapodEvent
{
    public required bool IsConnected { get; init; }
    public required ConnectionType ConnectionType { get; init; }
    public int? SignalStrength { get; init; }
}

/// <summary>
/// Defines connection types.
/// </summary>
public enum ConnectionType
{
    None,
    LoRaWAN,
    WiFi,
    Cellular
}

/// <summary>
/// Event raised when terrain classification changes.
/// </summary>
public record TerrainChangedEvent : HexapodEvent
{
    public required TerrainType PreviousTerrain { get; init; }
    public required TerrainType NewTerrain { get; init; }
    public required GaitType RecommendedGait { get; init; }
}

/// <summary>
/// Alias event for mode changes (backwards compatibility).
/// </summary>
public record ModeChangedEvent : HexapodEvent
{
    public required OperationMode PreviousMode { get; init; }
    public required OperationMode NewMode { get; init; }
    public string? Reason { get; init; }
}

/// <summary>
/// Alias event for state changes (backwards compatibility).
/// </summary>
public record StateChangedEvent : HexapodEvent
{
    public required MovementState PreviousState { get; init; }
    public required MovementState NewState { get; init; }
}

/// <summary>
/// Event raised when emergency stop is triggered.
/// </summary>
public record EmergencyStopEvent : HexapodEvent
{
    public required string Reason { get; init; }
    public required bool IsHardwareTriggered { get; init; }
}

/// <summary>
/// Event raised when an operator command is received.
/// </summary>
public record OperatorCommandReceivedEvent : HexapodEvent
{
    public required string CommandId { get; init; }
    public required string CommandType { get; init; }
    public CommandPriority Priority { get; init; } = CommandPriority.Normal;
    public Dictionary<string, object?> Parameters { get; init; } = new();
}

/// <summary>
/// Command priority levels.
/// </summary>
public enum CommandPriority
{
    Low,
    Normal,
    High,
    Critical
}

/// <summary>
/// Event raised when an object is detected.
/// </summary>
public record ObjectDetectedEvent : HexapodEvent
{
    public required DetectedObject Object { get; init; }
    public double Distance { get; init; }
    public double Bearing { get; init; }
}
