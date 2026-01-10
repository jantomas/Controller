using System.Text.Json;
using Hexapod.Core.Configuration;
using Hexapod.Core.Events;
using Hexapod.Core.Services;
using Hexapod.Telemetry.Logging;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Telemetry.Upload;

/// <summary>
/// Interface for telemetry upload coordination.
/// </summary>
public interface ITelemetryUploader
{
    /// <summary>
    /// Starts the upload service.
    /// </summary>
    Task StartAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Stops the upload service.
    /// </summary>
    Task StopAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Forces an immediate upload attempt.
    /// </summary>
    Task ForceUploadAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets upload status.
    /// </summary>
    UploadStatus GetStatus();
}

/// <summary>
/// Interface for upload destinations.
/// </summary>
public interface IUploadDestination
{
    /// <summary>
    /// Uploads telemetry batches.
    /// </summary>
    Task<UploadResult> UploadTelemetryAsync(
        IReadOnlyList<TelemetryBatch> batches,
        CancellationToken cancellationToken = default);

    /// <summary>
    /// Uploads activity records.
    /// </summary>
    Task<UploadResult> UploadActivitiesAsync(
        IReadOnlyList<ActivityRecord> activities,
        CancellationToken cancellationToken = default);

    /// <summary>
    /// Checks if destination is available.
    /// </summary>
    Task<bool> IsAvailableAsync(CancellationToken cancellationToken = default);
}

/// <summary>
/// Result of an upload operation.
/// </summary>
public record UploadResult
{
    public required bool Success { get; init; }
    public required int ItemsUploaded { get; init; }
    public IReadOnlyList<string>? SuccessfulIds { get; init; }
    public IReadOnlyList<string>? FailedIds { get; init; }
    public string? ErrorMessage { get; init; }
    public required TimeSpan Duration { get; init; }
}

/// <summary>
/// Current upload status.
/// </summary>
public record UploadStatus
{
    public required bool IsRunning { get; init; }
    public required bool IsConnected { get; init; }
    public required DateTimeOffset? LastSuccessfulUpload { get; init; }
    public required DateTimeOffset? LastAttempt { get; init; }
    public required int PendingBatches { get; init; }
    public required int PendingActivities { get; init; }
    public required int ConsecutiveFailures { get; init; }
}

/// <summary>
/// Telemetry batch for upload.
/// </summary>
public record TelemetryBatch
{
    public required string BatchId { get; init; }
    public required string DeviceId { get; init; }
    public required DateTimeOffset CreatedAt { get; init; }
    public required IReadOnlyList<object> DataPoints { get; init; }
    public int SizeBytes { get; init; }
}

/// <summary>
/// Implementation of telemetry upload coordinator.
/// </summary>
public sealed class TelemetryUploader : ITelemetryUploader, IDisposable
{
    private readonly IUploadDestination _destination;
    private readonly IActivityLogger _activityLogger;
    private readonly IEventBus _eventBus;
    private readonly ILogger<TelemetryUploader> _logger;
    private readonly TelemetryConfiguration _config;
    
    private readonly SemaphoreSlim _uploadLock = new(1, 1);
    private Timer? _uploadTimer;
    private CancellationTokenSource? _cts;
    
    private bool _isRunning;
    private bool _isConnected;
    private DateTimeOffset? _lastSuccessfulUpload;
    private DateTimeOffset? _lastAttempt;
    private int _consecutiveFailures;
    private bool _disposed;

    public TelemetryUploader(
        IUploadDestination destination,
        IActivityLogger activityLogger,
        IEventBus eventBus,
        IOptions<HexapodConfiguration> config,
        ILogger<TelemetryUploader> logger)
    {
        _destination = destination;
        _activityLogger = activityLogger;
        _eventBus = eventBus;
        _logger = logger;
        _config = config.Value.Telemetry;
    }

    public Task StartAsync(CancellationToken cancellationToken = default)
    {
        if (_isRunning)
            return Task.CompletedTask;

        _cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        
        _uploadTimer = new Timer(
            UploadCallback,
            null,
            TimeSpan.FromSeconds(_config.UploadIntervalSeconds),
            TimeSpan.FromSeconds(_config.UploadIntervalSeconds));

        _isRunning = true;
        _logger.LogInformation(
            "Telemetry uploader started with interval: {Interval}s",
            _config.UploadIntervalSeconds);

        return Task.CompletedTask;
    }

    public async Task StopAsync(CancellationToken cancellationToken = default)
    {
        if (!_isRunning)
            return;

        _cts?.Cancel();
        
        if (_uploadTimer != null)
        {
            await _uploadTimer.DisposeAsync();
            _uploadTimer = null;
        }

        _isRunning = false;
        _logger.LogInformation("Telemetry uploader stopped");
    }

    public async Task ForceUploadAsync(CancellationToken cancellationToken = default)
    {
        await PerformUploadAsync(cancellationToken);
    }

    public UploadStatus GetStatus()
    {
        return new UploadStatus
        {
            IsRunning = _isRunning,
            IsConnected = _isConnected,
            LastSuccessfulUpload = _lastSuccessfulUpload,
            LastAttempt = _lastAttempt,
            PendingBatches = 0, // Would be populated from collector
            PendingActivities = 0, // Would be populated from logger
            ConsecutiveFailures = _consecutiveFailures
        };
    }

    private void UploadCallback(object? state)
    {
        if (_cts?.IsCancellationRequested == true)
            return;

        _ = PerformUploadAsync(_cts?.Token ?? CancellationToken.None);
    }

    private async Task PerformUploadAsync(CancellationToken cancellationToken)
    {
        if (!await _uploadLock.WaitAsync(0, cancellationToken))
        {
            _logger.LogDebug("Upload already in progress, skipping");
            return;
        }

        try
        {
            _lastAttempt = DateTimeOffset.UtcNow;

            // Check connectivity
            _isConnected = await _destination.IsAvailableAsync(cancellationToken);
            if (!_isConnected)
            {
                _consecutiveFailures++;
                _logger.LogWarning(
                    "Upload destination unavailable, consecutive failures: {Count}",
                    _consecutiveFailures);
                return;
            }

            // Upload activities
            var activities = await _activityLogger.GetPendingUploadAsync(
                _config.MaxBatchSize, cancellationToken);

            if (activities.Count > 0)
            {
                var activityResult = await _destination.UploadActivitiesAsync(
                    activities, cancellationToken);

                if (activityResult.Success && activityResult.SuccessfulIds != null)
                {
                    await _activityLogger.MarkUploadedAsync(
                        activityResult.SuccessfulIds, cancellationToken);
                    
                    _logger.LogDebug(
                        "Uploaded {Count} activities in {Duration}ms",
                        activityResult.ItemsUploaded,
                        activityResult.Duration.TotalMilliseconds);
                }
            }

            _lastSuccessfulUpload = DateTimeOffset.UtcNow;
            _consecutiveFailures = 0;
        }
        catch (Exception ex)
        {
            _consecutiveFailures++;
            _logger.LogError(ex, 
                "Upload failed, consecutive failures: {Count}",
                _consecutiveFailures);
        }
        finally
        {
            _uploadLock.Release();
        }
    }

    public void Dispose()
    {
        if (!_disposed)
        {
            _uploadTimer?.Dispose();
            _uploadLock.Dispose();
            _cts?.Dispose();
            _disposed = true;
        }
    }
}

/// <summary>
/// Upload destination using Azure IoT Hub.
/// </summary>
public sealed class AzureIoTUploadDestination : IUploadDestination
{
    private readonly IEventBus _eventBus;
    private readonly ILogger<AzureIoTUploadDestination> _logger;
    private Func<Task<bool>>? _connectionChecker;
    private Func<object, CancellationToken, Task>? _messageSender;

    public AzureIoTUploadDestination(
        IEventBus eventBus,
        ILogger<AzureIoTUploadDestination> logger)
    {
        _eventBus = eventBus;
        _logger = logger;
    }

    /// <summary>
    /// Configures the connection checker delegate.
    /// </summary>
    public void ConfigureConnectionChecker(Func<Task<bool>> checker)
    {
        _connectionChecker = checker;
    }

    /// <summary>
    /// Configures the message sender delegate.
    /// </summary>
    public void ConfigureMessageSender(Func<object, CancellationToken, Task> sender)
    {
        _messageSender = sender;
    }

    public async Task<UploadResult> UploadTelemetryAsync(
        IReadOnlyList<TelemetryBatch> batches,
        CancellationToken cancellationToken = default)
    {
        var startTime = DateTimeOffset.UtcNow;
        var successIds = new List<string>();
        var failedIds = new List<string>();

        foreach (var batch in batches)
        {
            try
            {
                if (_messageSender != null)
                {
                    await _messageSender(new
                    {
                        type = "telemetry",
                        batchId = batch.BatchId,
                        deviceId = batch.DeviceId,
                        timestamp = batch.CreatedAt,
                        dataPoints = batch.DataPoints
                    }, cancellationToken);

                    successIds.Add(batch.BatchId);
                }
                else
                {
                    failedIds.Add(batch.BatchId);
                }
            }
            catch (Exception ex)
            {
                _logger.LogWarning(ex, "Failed to upload batch {Id}", batch.BatchId);
                failedIds.Add(batch.BatchId);
            }
        }

        return new UploadResult
        {
            Success = failedIds.Count == 0,
            ItemsUploaded = successIds.Count,
            SuccessfulIds = successIds.AsReadOnly(),
            FailedIds = failedIds.Count > 0 ? failedIds.AsReadOnly() : null,
            Duration = DateTimeOffset.UtcNow - startTime
        };
    }

    public async Task<UploadResult> UploadActivitiesAsync(
        IReadOnlyList<ActivityRecord> activities,
        CancellationToken cancellationToken = default)
    {
        var startTime = DateTimeOffset.UtcNow;
        var successIds = new List<string>();
        var failedIds = new List<string>();

        if (_messageSender != null)
        {
            try
            {
                // Send as batch
                await _messageSender(new
                {
                    type = "activities",
                    count = activities.Count,
                    activities = activities.Select(a => new
                    {
                        activityId = a.ActivityId,
                        type = a.Type.ToString(),
                        description = a.Description,
                        timestamp = a.Timestamp,
                        source = a.Source,
                        correlationId = a.CorrelationId,
                        position = a.Position,
                        metadata = a.Metadata
                    })
                }, cancellationToken);

                successIds.AddRange(activities.Select(a => a.ActivityId));
            }
            catch (Exception ex)
            {
                _logger.LogWarning(ex, "Failed to upload activities batch");
                failedIds.AddRange(activities.Select(a => a.ActivityId));
            }
        }
        else
        {
            failedIds.AddRange(activities.Select(a => a.ActivityId));
        }

        return new UploadResult
        {
            Success = failedIds.Count == 0,
            ItemsUploaded = successIds.Count,
            SuccessfulIds = successIds.AsReadOnly(),
            FailedIds = failedIds.Count > 0 ? failedIds.AsReadOnly() : null,
            Duration = DateTimeOffset.UtcNow - startTime
        };
    }

    public async Task<bool> IsAvailableAsync(CancellationToken cancellationToken = default)
    {
        if (_connectionChecker != null)
        {
            return await _connectionChecker();
        }
        return false;
    }
}
