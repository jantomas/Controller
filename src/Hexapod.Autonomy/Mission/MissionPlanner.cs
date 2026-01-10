using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Events;
using Hexapod.Core.Models;
using Hexapod.Core.Services;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Autonomy.Mission;

/// <summary>
/// Interface for mission planning and execution.
/// </summary>
public interface IMissionPlanner
{
    /// <summary>
    /// Creates a new mission.
    /// </summary>
    Task<MissionStatus> CreateMissionAsync(MissionDefinition definition, CancellationToken cancellationToken = default);

    /// <summary>
    /// Starts mission execution.
    /// </summary>
    Task StartMissionAsync(string missionId, CancellationToken cancellationToken = default);

    /// <summary>
    /// Pauses mission execution.
    /// </summary>
    Task PauseMissionAsync(string missionId, CancellationToken cancellationToken = default);

    /// <summary>
    /// Resumes mission execution.
    /// </summary>
    Task ResumeMissionAsync(string missionId, CancellationToken cancellationToken = default);

    /// <summary>
    /// Cancels mission.
    /// </summary>
    Task CancelMissionAsync(string missionId, string reason, CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets current mission status.
    /// </summary>
    MissionStatus? GetCurrentMission();

    /// <summary>
    /// Gets the next waypoint.
    /// </summary>
    Waypoint? GetNextWaypoint();

    /// <summary>
    /// Marks a waypoint as completed.
    /// </summary>
    Task CompleteWaypointAsync(string waypointId, CancellationToken cancellationToken = default);
}

/// <summary>
/// Mission definition from operator.
/// </summary>
public record MissionDefinition
{
    public required string Name { get; init; }
    public string? Description { get; init; }
    public required MissionType Type { get; init; }
    public required IReadOnlyList<Waypoint> Waypoints { get; init; }
    public GeoBoundary? OperationalBoundary { get; init; }
    public TimeSpan? MaxDuration { get; init; }
    public DateTimeOffset? ScheduledStart { get; init; }
    public MissionPriority Priority { get; init; } = MissionPriority.Normal;
}

/// <summary>
/// Waypoint in a mission path.
/// </summary>
public record Waypoint
{
    public required string WaypointId { get; init; }
    public required GeoPosition Position { get; init; }
    public WaypointType Type { get; init; } = WaypointType.NavigateTo;
    public IReadOnlyList<WaypointAction>? Actions { get; init; }
    public TimeSpan? WaitTime { get; init; }
    public double? ToleranceMeters { get; init; }
}

/// <summary>
/// Geographical boundary for mission.
/// </summary>
public record GeoBoundary
{
    public required IReadOnlyList<GeoPosition> Vertices { get; init; }
    public double MaxAltitude { get; init; }
    public double MinAltitude { get; init; }
}

/// <summary>
/// Action to perform at a waypoint.
/// </summary>
public record WaypointAction
{
    public required string ActionType { get; init; }
    public IReadOnlyDictionary<string, object>? Parameters { get; init; }
}

/// <summary>
/// Types of waypoints.
/// </summary>
public enum WaypointType
{
    NavigateTo,
    Patrol,
    Inspect,
    Wait,
    ReturnHome
}

/// <summary>
/// Types of missions.
/// </summary>
public enum MissionType
{
    Navigation,
    Patrol,
    Inspection,
    Exploration,
    Survey,
    Custom
}

/// <summary>
/// Mission priority levels.
/// </summary>
public enum MissionPriority
{
    Low,
    Normal,
    High,
    Critical
}

/// <summary>
/// State of a waypoint.
/// </summary>
public enum WaypointState
{
    Pending,
    Current,
    Completed,
    Skipped,
    Failed
}

/// <summary>
/// Implementation of mission planning and execution.
/// </summary>
public sealed class MissionPlanner : IMissionPlanner
{
    private readonly IEventBus _eventBus;
    private readonly ILogger<MissionPlanner> _logger;
    private readonly AutonomyConfiguration _config;
    
    private MissionStatus? _currentMission;
    private int _currentWaypointIndex;
    private readonly Dictionary<string, WaypointState> _waypointStates = new();
    private readonly object _lock = new();

    public MissionPlanner(
        IEventBus eventBus,
        IOptions<HexapodConfiguration> config,
        ILogger<MissionPlanner> logger)
    {
        _eventBus = eventBus;
        _logger = logger;
        _config = config.Value.Autonomy;
    }

    public Task<MissionStatus> CreateMissionAsync(
        MissionDefinition definition, 
        CancellationToken cancellationToken = default)
    {
        lock (_lock)
        {
            if (_currentMission != null && 
                _currentMission.Status != "Completed" && 
                _currentMission.Status != "Cancelled")
            {
                throw new InvalidOperationException("A mission is already active");
            }

            var missionId = Guid.NewGuid().ToString();
            
            // Initialize waypoint states
            _waypointStates.Clear();
            foreach (var wp in definition.Waypoints)
            {
                _waypointStates[wp.WaypointId] = WaypointState.Pending;
            }

            _currentMission = new MissionStatus
            {
                MissionId = missionId,
                MissionName = definition.Name,
                MissionType = definition.Type.ToString(),
                Status = "Created",
                Progress = 0.0,
                CompletedWaypoints = 0,
                TotalWaypoints = definition.Waypoints.Count,
                StartTime = DateTimeOffset.MinValue,
                TargetPosition = definition.Waypoints.FirstOrDefault()?.Position
            };

            _currentWaypointIndex = 0;

            _logger.LogInformation(
                "Mission created: {Id} - {Name} with {Count} waypoints",
                missionId, definition.Name, definition.Waypoints.Count);

            _eventBus.Publish(new MissionStatusChangedEvent
            {
                EventId = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow,
                Source = nameof(MissionPlanner),
                MissionId = missionId,
                PreviousStatus = null,
                NewStatus = "Created",
                Progress = 0
            });

            return Task.FromResult(_currentMission);
        }
    }

    public Task StartMissionAsync(string missionId, CancellationToken cancellationToken = default)
    {
        lock (_lock)
        {
            ValidateMission(missionId, "Created", "Paused");

            var previousStatus = _currentMission!.Status;
            _currentMission = _currentMission with
            {
                Status = "Active",
                StartTime = _currentMission.StartTime == DateTimeOffset.MinValue 
                    ? DateTimeOffset.UtcNow 
                    : _currentMission.StartTime
            };

            // Mark first waypoint as current
            if (_waypointStates.Any())
            {
                var firstPending = _waypointStates.FirstOrDefault(kv => kv.Value == WaypointState.Pending);
                if (firstPending.Key != null)
                {
                    _waypointStates[firstPending.Key] = WaypointState.Current;
                }
            }

            _logger.LogInformation("Mission started: {Id}", missionId);

            _eventBus.Publish(new MissionStatusChangedEvent
            {
                EventId = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow,
                Source = nameof(MissionPlanner),
                MissionId = missionId,
                PreviousStatus = previousStatus,
                NewStatus = "Active",
                Progress = CalculateProgress()
            });

            return Task.CompletedTask;
        }
    }

    public Task PauseMissionAsync(string missionId, CancellationToken cancellationToken = default)
    {
        lock (_lock)
        {
            ValidateMission(missionId, "Active");

            _currentMission = _currentMission with { Status = "Paused" };

            _logger.LogInformation("Mission paused: {Id}", missionId);

            _eventBus.Publish(new MissionStatusChangedEvent
            {
                EventId = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow,
                Source = nameof(MissionPlanner),
                MissionId = missionId,
                PreviousStatus = "Active",
                NewStatus = "Paused",
                Progress = CalculateProgress()
            });

            return Task.CompletedTask;
        }
    }

    public Task ResumeMissionAsync(string missionId, CancellationToken cancellationToken = default)
    {
        lock (_lock)
        {
            ValidateMission(missionId, "Paused");

            _currentMission = _currentMission with { Status = "Active" };

            _logger.LogInformation("Mission resumed: {Id}", missionId);

            _eventBus.Publish(new MissionStatusChangedEvent
            {
                EventId = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow,
                Source = nameof(MissionPlanner),
                MissionId = missionId,
                PreviousStatus = "Paused",
                NewStatus = "Active",
                Progress = CalculateProgress()
            });

            return Task.CompletedTask;
        }
    }

    public Task CancelMissionAsync(string missionId, string reason, CancellationToken cancellationToken = default)
    {
        lock (_lock)
        {
            if (_currentMission == null || _currentMission.MissionId != missionId)
            {
                throw new InvalidOperationException($"Mission {missionId} not found");
            }

            var previousStatus = _currentMission.Status;
            _currentMission = _currentMission with { Status = "Cancelled" };

            _logger.LogInformation("Mission cancelled: {Id}, Reason: {Reason}", missionId, reason);

            _eventBus.Publish(new MissionStatusChangedEvent
            {
                EventId = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow,
                Source = nameof(MissionPlanner),
                MissionId = missionId,
                PreviousStatus = previousStatus,
                NewStatus = "Cancelled",
                Progress = CalculateProgress()
            });

            return Task.CompletedTask;
        }
    }

    public MissionStatus? GetCurrentMission()
    {
        lock (_lock)
        {
            return _currentMission;
        }
    }

    public Waypoint? GetNextWaypoint()
    {
        lock (_lock)
        {
            if (_currentMission == null || _currentMission.Status != "Active")
                return null;

            // Return current waypoint from stored definition
            var currentWpId = _waypointStates
                .FirstOrDefault(kv => kv.Value == WaypointState.Current)
                .Key;

            // In a full implementation, we'd store the waypoints
            return null;
        }
    }

    public Task CompleteWaypointAsync(string waypointId, CancellationToken cancellationToken = default)
    {
        lock (_lock)
        {
            if (_currentMission == null || _currentMission.Status != "Active")
            {
                throw new InvalidOperationException("No active mission");
            }

            if (!_waypointStates.ContainsKey(waypointId))
            {
                throw new ArgumentException($"Waypoint {waypointId} not found", nameof(waypointId));
            }

            _waypointStates[waypointId] = WaypointState.Completed;
            var completed = _waypointStates.Count(kv => kv.Value == WaypointState.Completed);

            // Find next pending waypoint
            var nextWpId = _waypointStates
                .FirstOrDefault(kv => kv.Value == WaypointState.Pending)
                .Key;

            if (nextWpId != null)
            {
                _waypointStates[nextWpId] = WaypointState.Current;
            }

            _currentMission = _currentMission with
            {
                CompletedWaypoints = completed
            };

            _logger.LogInformation(
                "Waypoint completed: {WpId}, Progress: {Progress}%",
                waypointId, CalculateProgress());

            // Check if mission is complete
            if (completed == _currentMission.TotalWaypoints)
            {
                _currentMission = _currentMission with { Status = "Completed" };
                
                _eventBus.Publish(new MissionStatusChangedEvent
                {
                    EventId = Guid.NewGuid().ToString(),
                    Timestamp = DateTimeOffset.UtcNow,
                    Source = nameof(MissionPlanner),
                    MissionId = _currentMission.MissionId,
                    PreviousStatus = "Active",
                    NewStatus = "Completed",
                    Progress = 100
                });

                _logger.LogInformation("Mission completed: {Id}", _currentMission.MissionId);
            }

            _eventBus.Publish(new WaypointReachedEvent
            {
                EventId = Guid.NewGuid().ToString(),
                Timestamp = DateTimeOffset.UtcNow,
                Source = nameof(MissionPlanner),
                WaypointId = waypointId,
                MissionId = _currentMission.MissionId,
                RemainingWaypoints = _currentMission.TotalWaypoints - completed
            });

            return Task.CompletedTask;
        }
    }

    private void ValidateMission(string missionId, params string[] validStatuses)
    {
        if (_currentMission == null || _currentMission.MissionId != missionId)
        {
            throw new InvalidOperationException($"Mission {missionId} not found");
        }

        if (!validStatuses.Contains(_currentMission.Status))
        {
            throw new InvalidOperationException(
                $"Mission is in state {_currentMission.Status}, expected one of: {string.Join(", ", validStatuses)}");
        }
    }

    private int CalculateProgress()
    {
        if (_currentMission == null || _currentMission.TotalWaypoints == 0)
            return 0;

        return (int)((_currentMission.CompletedWaypoints * 100.0) / _currentMission.TotalWaypoints);
    }
}

/// <summary>
/// Event when a waypoint is reached.
/// </summary>
public record WaypointReachedEvent : HexapodEvent
{
    public required string WaypointId { get; init; }
    public required string MissionId { get; init; }
    public required int RemainingWaypoints { get; init; }
}
