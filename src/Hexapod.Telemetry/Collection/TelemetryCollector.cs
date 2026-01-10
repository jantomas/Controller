using System.Text.Json;
using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Events;
using Hexapod.Core.Models;
using Hexapod.Core.Services;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Telemetry.Collection;

/// <summary>
/// Interface for telemetry collection service.
/// </summary>
public interface ITelemetryCollector
{
    /// <summary>
    /// Records a telemetry data point.
    /// </summary>
    Task RecordAsync<T>(string metricName, T value, Dictionary<string, string>? tags = null) 
        where T : notnull;

    /// <summary>
    /// Records sensor state telemetry.
    /// </summary>
    Task RecordSensorStateAsync(SensorState state);

    /// <summary>
    /// Records system health telemetry.
    /// </summary>
    Task RecordHealthAsync(SystemHealth health);

    /// <summary>
    /// Gets buffered telemetry for upload.
    /// </summary>
    Task<IReadOnlyList<TelemetryBatch>> GetBufferedTelemetryAsync(
        int maxBatches,
        CancellationToken cancellationToken = default);

    /// <summary>
    /// Clears uploaded telemetry.
    /// </summary>
    Task ClearUploadedAsync(IEnumerable<string> batchIds, CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets current statistics.
    /// </summary>
    TelemetryStatistics GetStatistics();
}

/// <summary>
/// Telemetry data point.
/// </summary>
public record TelemetryDataPoint
{
    public required string MetricName { get; init; }
    public required object Value { get; init; }
    public required DateTimeOffset Timestamp { get; init; }
    public Dictionary<string, string>? Tags { get; init; }
}

/// <summary>
/// Batch of telemetry data.
/// </summary>
public record TelemetryBatch
{
    public required string BatchId { get; init; }
    public required string DeviceId { get; init; }
    public required DateTimeOffset CreatedAt { get; init; }
    public required IReadOnlyList<TelemetryDataPoint> DataPoints { get; init; }
    public int SizeBytes { get; init; }
}

/// <summary>
/// Statistics about telemetry collection.
/// </summary>
public record TelemetryStatistics
{
    public required long TotalPointsCollected { get; init; }
    public required long TotalPointsUploaded { get; init; }
    public required int BufferedBatches { get; init; }
    public required long BufferSizeBytes { get; init; }
    public required DateTimeOffset? LastUploadTime { get; init; }
    public required TimeSpan? AverageUploadLatency { get; init; }
}

/// <summary>
/// Implementation of telemetry collection with buffering.
/// </summary>
public sealed class TelemetryCollector : ITelemetryCollector, IDisposable
{
    private readonly IEventBus _eventBus;
    private readonly ILogger<TelemetryCollector> _logger;
    private readonly TelemetryConfiguration _config;
    private readonly string _deviceId;
    
    private readonly List<TelemetryDataPoint> _currentBuffer = new();
    private readonly List<TelemetryBatch> _batchBuffer = new();
    private readonly SemaphoreSlim _bufferLock = new(1, 1);
    private readonly Timer _batchTimer;
    
    private long _totalCollected;
    private long _totalUploaded;
    private DateTimeOffset? _lastUpload;
    private TimeSpan? _avgUploadLatency;
    private bool _disposed;

    public TelemetryCollector(
        IEventBus eventBus,
        IOptions<HexapodConfiguration> config,
        ILogger<TelemetryCollector> logger)
    {
        _eventBus = eventBus;
        _logger = logger;
        _config = config.Value.Telemetry;
        _deviceId = config.Value.DeviceId;

        // Start batch creation timer
        _batchTimer = new Timer(
            CreateBatchCallback, 
            null, 
            TimeSpan.FromSeconds(_config.BatchIntervalSeconds),
            TimeSpan.FromSeconds(_config.BatchIntervalSeconds));

        _logger.LogInformation(
            "Telemetry collector initialized with batch interval: {Interval}s",
            _config.BatchIntervalSeconds);
    }

    public async Task RecordAsync<T>(string metricName, T value, Dictionary<string, string>? tags = null)
        where T : notnull
    {
        var dataPoint = new TelemetryDataPoint
        {
            MetricName = metricName,
            Value = value,
            Timestamp = DateTimeOffset.UtcNow,
            Tags = tags
        };

        await _bufferLock.WaitAsync();
        try
        {
            _currentBuffer.Add(dataPoint);
            Interlocked.Increment(ref _totalCollected);

            // Check if we need to create a batch early
            if (_currentBuffer.Count >= _config.MaxBatchSize)
            {
                await CreateBatchAsync();
            }
        }
        finally
        {
            _bufferLock.Release();
        }
    }

    public Task RecordSensorStateAsync(SensorState state)
    {
        var tasks = new List<Task>();

        if (state.Position != null)
        {
            tasks.Add(RecordAsync("position.latitude", state.Position.Latitude));
            tasks.Add(RecordAsync("position.longitude", state.Position.Longitude));
            tasks.Add(RecordAsync("position.altitude", state.Position.Altitude));
        }

        if (state.Orientation != null)
        {
            tasks.Add(RecordAsync("orientation.roll", state.Orientation.Roll));
            tasks.Add(RecordAsync("orientation.pitch", state.Orientation.Pitch));
            tasks.Add(RecordAsync("orientation.yaw", state.Orientation.Yaw));
        }

        if (state.Acceleration != null)
        {
            tasks.Add(RecordAsync("acceleration.x", state.Acceleration.X));
            tasks.Add(RecordAsync("acceleration.y", state.Acceleration.Y));
            tasks.Add(RecordAsync("acceleration.z", state.Acceleration.Z));
        }

        if (state.PowerStatus != null)
        {
            tasks.Add(RecordAsync("power.battery_percent", state.PowerStatus.BatteryPercentage));
            tasks.Add(RecordAsync("power.voltage", state.PowerStatus.BatteryVoltage));
            tasks.Add(RecordAsync("power.current", state.PowerStatus.BatteryCurrent));
            tasks.Add(RecordAsync("power.charging", state.PowerStatus.IsCharging ? 1 : 0));
        }

        if (state.DistanceReadings != null)
        {
            foreach (var reading in state.DistanceReadings)
            {
                tasks.Add(RecordAsync($"distance.sensor_{reading.SensorId}", reading.Distance,
                    new Dictionary<string, string> { ["sensor_name"] = reading.SensorName }));
            }
        }

        return Task.WhenAll(tasks);
    }

    public Task RecordHealthAsync(SystemHealth health)
    {
        var tasks = new List<Task>
        {
            RecordAsync("health.status", (int)health.OverallStatus),
            RecordAsync("health.cpu_usage", health.CpuUsagePercent),
            RecordAsync("health.memory_usage", health.MemoryUsagePercent),
            RecordAsync("health.storage_used", health.StorageUsedBytes),
            RecordAsync("health.storage_available", health.StorageAvailableBytes),
            RecordAsync("health.temperature_cpu", health.CpuTemperatureCelsius),
            RecordAsync("health.uptime_seconds", health.UptimeSeconds)
        };

        return Task.WhenAll(tasks);
    }

    public async Task<IReadOnlyList<TelemetryBatch>> GetBufferedTelemetryAsync(
        int maxBatches,
        CancellationToken cancellationToken = default)
    {
        await _bufferLock.WaitAsync(cancellationToken);
        try
        {
            return _batchBuffer.Take(maxBatches).ToList().AsReadOnly();
        }
        finally
        {
            _bufferLock.Release();
        }
    }

    public async Task ClearUploadedAsync(IEnumerable<string> batchIds, CancellationToken cancellationToken = default)
    {
        var idSet = batchIds.ToHashSet();
        
        await _bufferLock.WaitAsync(cancellationToken);
        try
        {
            var removed = _batchBuffer.RemoveAll(b => idSet.Contains(b.BatchId));
            Interlocked.Add(ref _totalUploaded, removed);
            _lastUpload = DateTimeOffset.UtcNow;
            
            _logger.LogDebug("Cleared {Count} uploaded batches", removed);
        }
        finally
        {
            _bufferLock.Release();
        }
    }

    public TelemetryStatistics GetStatistics()
    {
        return new TelemetryStatistics
        {
            TotalPointsCollected = _totalCollected,
            TotalPointsUploaded = _totalUploaded,
            BufferedBatches = _batchBuffer.Count,
            BufferSizeBytes = _batchBuffer.Sum(b => b.SizeBytes),
            LastUploadTime = _lastUpload,
            AverageUploadLatency = _avgUploadLatency
        };
    }

    private void CreateBatchCallback(object? state)
    {
        _ = CreateBatchAsync();
    }

    private async Task CreateBatchAsync()
    {
        if (_currentBuffer.Count == 0)
            return;

        await _bufferLock.WaitAsync();
        try
        {
            if (_currentBuffer.Count == 0)
                return;

            var dataPoints = _currentBuffer.ToList();
            _currentBuffer.Clear();

            var batch = new TelemetryBatch
            {
                BatchId = Guid.NewGuid().ToString(),
                DeviceId = _deviceId,
                CreatedAt = DateTimeOffset.UtcNow,
                DataPoints = dataPoints.AsReadOnly(),
                SizeBytes = EstimateBatchSize(dataPoints)
            };

            _batchBuffer.Add(batch);

            // Check buffer limit
            while (_batchBuffer.Count > _config.MaxBufferedBatches)
            {
                var oldest = _batchBuffer[0];
                _batchBuffer.RemoveAt(0);
                _logger.LogWarning(
                    "Dropped oldest batch {Id} due to buffer limit", 
                    oldest.BatchId);
            }

            _logger.LogDebug(
                "Created telemetry batch {Id} with {Count} points",
                batch.BatchId, dataPoints.Count);
        }
        finally
        {
            _bufferLock.Release();
        }
    }

    private static int EstimateBatchSize(List<TelemetryDataPoint> dataPoints)
    {
        // Rough estimate: 100 bytes per data point average
        return dataPoints.Count * 100;
    }

    public void Dispose()
    {
        if (!_disposed)
        {
            _batchTimer.Dispose();
            _bufferLock.Dispose();
            _disposed = true;
        }
    }
}

/// <summary>
/// System health information.
/// </summary>
public record SystemHealth
{
    public required HealthStatus OverallStatus { get; init; }
    public required double CpuUsagePercent { get; init; }
    public required double MemoryUsagePercent { get; init; }
    public required long StorageUsedBytes { get; init; }
    public required long StorageAvailableBytes { get; init; }
    public required double CpuTemperatureCelsius { get; init; }
    public required long UptimeSeconds { get; init; }
    public required DateTimeOffset Timestamp { get; init; }
}
