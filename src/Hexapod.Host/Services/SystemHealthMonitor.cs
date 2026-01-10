using System.Diagnostics;
using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Telemetry.Collection;
using Microsoft.Extensions.Options;

namespace Hexapod.Host.Services;

/// <summary>
/// Background service for monitoring system health.
/// </summary>
public sealed class SystemHealthMonitor : BackgroundService
{
    private readonly ITelemetryCollector _telemetryCollector;
    private readonly ILogger<SystemHealthMonitor> _logger;
    private readonly HexapodConfiguration _config;
    
    private readonly Stopwatch _uptimeStopwatch = Stopwatch.StartNew();

    public SystemHealthMonitor(
        ITelemetryCollector telemetryCollector,
        IOptions<HexapodConfiguration> config,
        ILogger<SystemHealthMonitor> logger)
    {
        _telemetryCollector = telemetryCollector;
        _config = config.Value;
        _logger = logger;
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        _logger.LogInformation("System health monitor starting...");

        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                var health = await CollectHealthMetricsAsync();
                await _telemetryCollector.RecordHealthAsync(health);

                if (health.OverallStatus >= HealthStatus.Degraded)
                {
                    _logger.LogWarning(
                        "System health degraded: CPU={Cpu}%, Mem={Mem}%, Temp={Temp}°C",
                        health.CpuUsagePercent,
                        health.MemoryUsagePercent,
                        health.CpuTemperatureCelsius);
                }
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error collecting health metrics");
            }

            await Task.Delay(TimeSpan.FromSeconds(10), stoppingToken);
        }
    }

    private Task<SystemHealth> CollectHealthMetricsAsync()
    {
        var process = Process.GetCurrentProcess();
        
        // Get CPU usage
        var cpuUsage = GetCpuUsage();
        
        // Get memory usage
        var totalMemory = GC.GetGCMemoryInfo().TotalAvailableMemoryBytes;
        var usedMemory = process.WorkingSet64;
        var memoryPercent = (double)usedMemory / totalMemory * 100;

        // Get storage info
        var storagePath = _config.Telemetry.LocalStoragePath;
        var driveInfo = new DriveInfo(Path.GetPathRoot(storagePath) ?? "/");
        
        // Get CPU temperature (Raspberry Pi specific)
        var cpuTemp = GetCpuTemperature();

        // Determine overall status
        var status = HealthStatus.Healthy;
        
        if (cpuUsage > 90 || memoryPercent > 90 || cpuTemp > 80)
            status = HealthStatus.Critical;
        else if (cpuUsage > 70 || memoryPercent > 70 || cpuTemp > 70)
            status = HealthStatus.Degraded;

        return Task.FromResult(new SystemHealth
        {
            OverallStatus = status,
            CpuUsagePercent = cpuUsage,
            MemoryUsagePercent = memoryPercent,
            StorageUsedBytes = driveInfo.TotalSize - driveInfo.AvailableFreeSpace,
            StorageAvailableBytes = driveInfo.AvailableFreeSpace,
            CpuTemperatureCelsius = cpuTemp,
            UptimeSeconds = (long)_uptimeStopwatch.Elapsed.TotalSeconds,
            Timestamp = DateTimeOffset.UtcNow
        });
    }

    private static double GetCpuUsage()
    {
        // On Linux, read from /proc/stat
        try
        {
            if (OperatingSystem.IsLinux())
            {
                var cpuLine = File.ReadLines("/proc/stat").First();
                var values = cpuLine.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                
                if (values.Length >= 5)
                {
                    var user = double.Parse(values[1]);
                    var nice = double.Parse(values[2]);
                    var system = double.Parse(values[3]);
                    var idle = double.Parse(values[4]);
                    
                    var total = user + nice + system + idle;
                    var used = user + nice + system;
                    
                    return (used / total) * 100;
                }
            }
        }
        catch
        {
            // Ignore errors
        }

        return 0;
    }

    private static double GetCpuTemperature()
    {
        // On Raspberry Pi, read from thermal zone
        try
        {
            if (OperatingSystem.IsLinux() && File.Exists("/sys/class/thermal/thermal_zone0/temp"))
            {
                var temp = File.ReadAllText("/sys/class/thermal/thermal_zone0/temp").Trim();
                if (double.TryParse(temp, out var milliCelsius))
                {
                    return milliCelsius / 1000.0;
                }
            }
        }
        catch
        {
            // Ignore errors
        }

        return 0;
    }
}
