using System.Text.Json;
using Hexapod.Core.Configuration;
using Hexapod.Core.Events;
using Hexapod.Core.Models;
using Hexapod.Core.Services;
using LiteDB;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Telemetry.Logging;

/// <summary>
/// Interface for activity logging service.
/// </summary>
public interface IActivityLogger
{
    /// <summary>
    /// Logs an activity.
    /// </summary>
    Task LogActivityAsync(ActivityRecord activity, CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets recent activities.
    /// </summary>
    Task<IReadOnlyList<ActivityRecord>> GetRecentActivitiesAsync(
        int count, 
        CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets activities by type.
    /// </summary>
    Task<IReadOnlyList<ActivityRecord>> GetActivitiesByTypeAsync(
        ActivityType type, 
        DateTimeOffset? since = null,
        CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets activities pending upload.
    /// </summary>
    Task<IReadOnlyList<ActivityRecord>> GetPendingUploadAsync(
        int maxCount,
        CancellationToken cancellationToken = default);

    /// <summary>
    /// Marks activities as uploaded.
    /// </summary>
    Task MarkUploadedAsync(
        IEnumerable<string> activityIds, 
        CancellationToken cancellationToken = default);

    /// <summary>
    /// Prunes old activities.
    /// </summary>
    Task PruneOldActivitiesAsync(TimeSpan maxAge, CancellationToken cancellationToken = default);
}

/// <summary>
/// Type of activity.
/// </summary>
public enum ActivityType
{
    SystemStartup,
    SystemShutdown,
    ModeChange,
    StateChange,
    Movement,
    SensorReading,
    Decision,
    OperatorCommand,
    Alert,
    Error,
    MissionEvent,
    Communication,
    PowerEvent,
    Detection,
    Custom
}

/// <summary>
/// Record of an activity.
/// </summary>
public class ActivityRecord
{
    [BsonId]
    public string ActivityId { get; set; } = Guid.NewGuid().ToString();
    
    public required ActivityType Type { get; set; }
    public required string Description { get; set; }
    public required DateTimeOffset Timestamp { get; set; }
    public string? Source { get; set; }
    public string? CorrelationId { get; set; }
    public GeoPosition? Position { get; set; }
    public Dictionary<string, object>? Metadata { get; set; }
    public bool IsUploaded { get; set; }
    public DateTimeOffset? UploadedAt { get; set; }
}

/// <summary>
/// Implementation of activity logging with local storage.
/// </summary>
public sealed class ActivityLogger : IActivityLogger, IDisposable
{
    private readonly IEventBus _eventBus;
    private readonly ILogger<ActivityLogger> _logger;
    private readonly TelemetryConfiguration _config;
    private readonly LiteDatabase _database;
    private readonly ILiteCollection<ActivityRecord> _activities;
    private readonly SemaphoreSlim _dbLock = new(1, 1);
    private bool _disposed;

    public ActivityLogger(
        IEventBus eventBus,
        IOptions<HexapodConfiguration> config,
        ILogger<ActivityLogger> logger)
    {
        _eventBus = eventBus;
        _logger = logger;
        _config = config.Value.Telemetry;

        var dbPath = Path.Combine(_config.LocalStoragePath, "activities.db");
        Directory.CreateDirectory(_config.LocalStoragePath);
        
        _database = new LiteDatabase(dbPath);
        _activities = _database.GetCollection<ActivityRecord>("activities");
        _activities.EnsureIndex(x => x.Timestamp);
        _activities.EnsureIndex(x => x.Type);
        _activities.EnsureIndex(x => x.IsUploaded);

        // Subscribe to system events for automatic logging
        SubscribeToEvents();

        _logger.LogInformation("Activity logger initialized with storage at {Path}", dbPath);
    }

    public async Task LogActivityAsync(ActivityRecord activity, CancellationToken cancellationToken = default)
    {
        await _dbLock.WaitAsync(cancellationToken);
        try
        {
            _activities.Insert(activity);
            
            _logger.LogDebug(
                "Activity logged: {Type} - {Description}", 
                activity.Type, activity.Description);
        }
        finally
        {
            _dbLock.Release();
        }
    }

    public async Task<IReadOnlyList<ActivityRecord>> GetRecentActivitiesAsync(
        int count, 
        CancellationToken cancellationToken = default)
    {
        await _dbLock.WaitAsync(cancellationToken);
        try
        {
            return _activities
                .Query()
                .OrderByDescending(x => x.Timestamp)
                .Limit(count)
                .ToList()
                .AsReadOnly();
        }
        finally
        {
            _dbLock.Release();
        }
    }

    public async Task<IReadOnlyList<ActivityRecord>> GetActivitiesByTypeAsync(
        ActivityType type, 
        DateTimeOffset? since = null,
        CancellationToken cancellationToken = default)
    {
        await _dbLock.WaitAsync(cancellationToken);
        try
        {
            var query = _activities.Query().Where(x => x.Type == type);

            if (since.HasValue)
            {
                query = query.Where(x => x.Timestamp >= since.Value);
            }

            return query
                .OrderByDescending(x => x.Timestamp)
                .ToList()
                .AsReadOnly();
        }
        finally
        {
            _dbLock.Release();
        }
    }

    public async Task<IReadOnlyList<ActivityRecord>> GetPendingUploadAsync(
        int maxCount,
        CancellationToken cancellationToken = default)
    {
        await _dbLock.WaitAsync(cancellationToken);
        try
        {
            return _activities
                .Query()
                .Where(x => !x.IsUploaded)
                .OrderBy(x => x.Timestamp)
                .Limit(maxCount)
                .ToList()
                .AsReadOnly();
        }
        finally
        {
            _dbLock.Release();
        }
    }

    public async Task MarkUploadedAsync(
        IEnumerable<string> activityIds, 
        CancellationToken cancellationToken = default)
    {
        await _dbLock.WaitAsync(cancellationToken);
        try
        {
            foreach (var id in activityIds)
            {
                var activity = _activities.FindById(id);
                if (activity != null)
                {
                    activity.IsUploaded = true;
                    activity.UploadedAt = DateTimeOffset.UtcNow;
                    _activities.Update(activity);
                }
            }
        }
        finally
        {
            _dbLock.Release();
        }
    }

    public async Task PruneOldActivitiesAsync(TimeSpan maxAge, CancellationToken cancellationToken = default)
    {
        await _dbLock.WaitAsync(cancellationToken);
        try
        {
            var cutoff = DateTimeOffset.UtcNow - maxAge;
            var deleted = _activities.DeleteMany(x => x.Timestamp < cutoff && x.IsUploaded);
            
            if (deleted > 0)
            {
                _logger.LogInformation("Pruned {Count} old activities", deleted);
            }
        }
        finally
        {
            _dbLock.Release();
        }
    }

    private void SubscribeToEvents()
    {
        // Mode changes
        _eventBus.Subscribe<ModeChangedEvent>()
            .Subscribe(e => LogActivityAsync(new ActivityRecord
            {
                Type = ActivityType.ModeChange,
                Description = $"Mode changed: {e.PreviousMode} -> {e.NewMode}",
                Timestamp = e.Timestamp,
                Source = e.Source,
                Metadata = new Dictionary<string, object>
                {
                    ["PreviousMode"] = e.PreviousMode.ToString(),
                    ["NewMode"] = e.NewMode.ToString(),
                    ["Reason"] = e.Reason ?? string.Empty
                }
            }).GetAwaiter().GetResult());

        // State changes
        _eventBus.Subscribe<StateChangedEvent>()
            .Subscribe(e => LogActivityAsync(new ActivityRecord
            {
                Type = ActivityType.StateChange,
                Description = $"State changed: {e.PreviousState} -> {e.NewState}",
                Timestamp = e.Timestamp,
                Source = e.Source,
                Metadata = new Dictionary<string, object>
                {
                    ["PreviousState"] = e.PreviousState.ToString(),
                    ["NewState"] = e.NewState.ToString()
                }
            }).GetAwaiter().GetResult());

        // Emergency events
        _eventBus.Subscribe<EmergencyStopEvent>()
            .Subscribe(e => LogActivityAsync(new ActivityRecord
            {
                Type = ActivityType.Alert,
                Description = $"EMERGENCY STOP: {e.Reason}",
                Timestamp = e.Timestamp,
                Source = e.Source,
                Metadata = new Dictionary<string, object>
                {
                    ["Reason"] = e.Reason
                }
            }).GetAwaiter().GetResult());

        // Operator commands
        _eventBus.Subscribe<OperatorCommandReceivedEvent>()
            .Subscribe(e => LogActivityAsync(new ActivityRecord
            {
                Type = ActivityType.OperatorCommand,
                Description = $"Command received: {e.CommandType}",
                Timestamp = e.Timestamp,
                Source = e.Source,
                CorrelationId = e.CommandId,
                Metadata = new Dictionary<string, object>
                {
                    ["CommandType"] = e.CommandType,
                    ["Priority"] = e.Priority.ToString()
                }
            }).GetAwaiter().GetResult());

        // Mission events
        _eventBus.Subscribe<MissionStatusChangedEvent>()
            .Subscribe(e => LogActivityAsync(new ActivityRecord
            {
                Type = ActivityType.MissionEvent,
                Description = $"Mission {e.MissionId}: {e.PreviousStatus} -> {e.NewStatus}",
                Timestamp = e.Timestamp,
                Source = e.Source,
                Metadata = new Dictionary<string, object>
                {
                    ["MissionId"] = e.MissionId,
                    ["NewStatus"] = e.NewStatus,
                    ["Progress"] = e.Progress
                }
            }).GetAwaiter().GetResult());

        // Object detection
        _eventBus.Subscribe<ObjectDetectedEvent>()
            .Subscribe(e => LogActivityAsync(new ActivityRecord
            {
                Type = ActivityType.Detection,
                Description = $"Object detected: {e.Object.Label} ({e.Object.Confidence:P0})",
                Timestamp = e.Timestamp,
                Source = e.Source,
                Metadata = new Dictionary<string, object>
                {
                    ["Label"] = e.Object.Label,
                    ["Confidence"] = e.Object.Confidence,
                    ["IsObstacle"] = e.Object.IsObstacle,
                    ["IsHazard"] = e.Object.IsHazard
                }
            }).GetAwaiter().GetResult());
    }

    public void Dispose()
    {
        if (!_disposed)
        {
            _dbLock.Dispose();
            _database.Dispose();
            _disposed = true;
        }
    }
}
