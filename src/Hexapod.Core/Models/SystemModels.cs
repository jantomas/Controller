using Hexapod.Core.Enums;

namespace Hexapod.Core.Models;

/// <summary>
/// Represents the complete system state at a point in time.
/// </summary>
public record SystemState
{
    public required string DeviceId { get; init; }
    public required OperationMode OperationMode { get; init; }
    public required MovementState MovementState { get; init; }
    public required GaitType CurrentGait { get; init; }
    public required HealthStatus OverallHealth { get; init; }
    public required SensorState SensorState { get; init; }
    public required MissionStatus? CurrentMission { get; init; }
    public required DateTimeOffset Timestamp { get; init; }
    public required bool IsConnected { get; init; }
    public required TimeSpan Uptime { get; init; }
}

/// <summary>
/// Represents the status of the current mission.
/// </summary>
public record MissionStatus
{
    public required string MissionId { get; init; }
    public required string MissionType { get; init; }
    public string MissionName { get; init; } = string.Empty;
    public MissionPhase Phase { get; init; }
    public string Status { get; init; } = string.Empty;
    public required double Progress { get; init; } // 0.0 to 1.0
    public GeoPosition? TargetPosition { get; init; }
    public GeoPosition? CurrentPosition { get; init; }
    public DateTimeOffset StartTime { get; init; }
    public TimeSpan EstimatedTimeRemaining { get; init; }
    public DateTimeOffset? CompletedAt { get; init; }
    public int CompletedWaypoints { get; init; }
    public int TotalWaypoints { get; init; }
    public IReadOnlyList<Waypoint> Waypoints { get; init; } = Array.Empty<Waypoint>();
    public int CurrentWaypointIndex { get; init; }
}

/// <summary>
/// Defines mission execution phases.
/// </summary>
public enum MissionPhase
{
    Pending,
    Starting,
    InProgress,
    Paused,
    AwaitingConfirmation,
    Completing,
    Completed,
    Aborted,
    Failed
}

/// <summary>
/// Represents a navigation waypoint.
/// </summary>
public record Waypoint
{
    public required int Index { get; init; }
    public required GeoPosition Position { get; init; }
    public required WaypointType Type { get; init; }
    public double? Heading { get; init; }
    public TimeSpan? WaitTime { get; init; }
    public string? Action { get; init; }
    public bool IsCompleted { get; init; }
}

/// <summary>
/// Defines waypoint types.
/// </summary>
public enum WaypointType
{
    Navigation,
    Inspection,
    DataCollection,
    Charging,
    OperatorCheckpoint
}

/// <summary>
/// Represents a decision that requires evaluation or confirmation.
/// </summary>
public record Decision
{
    public required string DecisionId { get; init; }
    public required string Description { get; init; }
    public required RiskLevel RiskLevel { get; init; }
    public required DecisionType Type { get; init; }
    public required IReadOnlyList<DecisionOption> Options { get; init; }
    public required DateTimeOffset CreatedAt { get; init; }
    public DateTimeOffset? ExpiresAt { get; init; }
    public DecisionOption? SelectedOption { get; init; }
    public string? OperatorNotes { get; init; }
    public bool RequiresConfirmation { get; init; }
}

/// <summary>
/// Defines decision types.
/// </summary>
public enum DecisionType
{
    PathSelection,
    ObstacleAvoidance,
    TerrainTraversal,
    BoundaryViolation,
    MissionModification,
    EmergencyAction,
    ResourceManagement
}

/// <summary>
/// Represents an option for a decision.
/// </summary>
public record DecisionOption
{
    public required string OptionId { get; init; }
    public required string Description { get; init; }
    public required double Confidence { get; init; }
    public required RiskLevel Risk { get; init; }
    public double? EnergyCost { get; init; }
    public TimeSpan? TimeCost { get; init; }
}

/// <summary>
/// Represents a detected object from the vision system.
/// </summary>
public record DetectedObject
{
    public required string Label { get; init; }
    public required double Confidence { get; init; }
    public required BoundingBox BoundingBox { get; init; }
    public double? EstimatedDistance { get; init; }
    public string? Category { get; init; }
    public bool IsObstacle { get; init; }
    public bool IsHazard { get; init; }
    public DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Represents a bounding box for object detection.
/// </summary>
public record BoundingBox
{
    public required double X { get; init; }
    public required double Y { get; init; }
    public required double Width { get; init; }
    public required double Height { get; init; }
}

/// <summary>
/// Represents the result of scene analysis.
/// </summary>
public record SceneAnalysis
{
    public required TerrainType PrimaryTerrain { get; init; }
    public required double TerrainConfidence { get; init; }
    public required IReadOnlyList<DetectedObject> DetectedObjects { get; init; }
    public required bool IsPathClear { get; init; }
    public required double TraversabilityScore { get; init; }
    public IReadOnlyList<HazardZone> HazardZones { get; init; } = Array.Empty<HazardZone>();
    public DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Represents a hazardous zone detected in the scene.
/// </summary>
public record HazardZone
{
    public required string HazardType { get; init; }
    public required BoundingBox Area { get; init; }
    public required RiskLevel Risk { get; init; }
    public string? AvoidanceRecommendation { get; init; }
}
