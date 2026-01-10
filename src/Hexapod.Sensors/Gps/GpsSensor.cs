using System.IO.Ports;
using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Models;
using Hexapod.Sensors.Abstractions;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Sensors.Gps;

/// <summary>
/// GPS sensor implementation for u-blox NEO-M9N module using NMEA protocol.
/// </summary>
public sealed class GpsSensor : IGpsSensor, IDisposable
{
    private readonly ILogger<GpsSensor> _logger;
    private readonly GpsConfig _config;
    private SerialPort? _serialPort;
    private GeoPosition? _lastPosition;
    private bool _hasFix;
    private int _satellitesInView;
    private double _hdop;
    private HealthStatus _health = HealthStatus.Unknown;
    private readonly CancellationTokenSource _cts = new();
    private Task? _readTask;

    public GpsSensor(IOptions<HexapodConfiguration> config, ILogger<GpsSensor> logger)
    {
        _logger = logger;
        _config = config.Value.Sensors.Gps;
    }

    public SensorType Type => SensorType.Gps;
    public string Name => "GPS-NEO-M9N";
    public bool IsEnabled => _config.Enabled;
    public HealthStatus Health => _health;
    public bool HasFix => _hasFix;
    public int SatellitesInView => _satellitesInView;
    public double Hdop => _hdop;

    public async Task InitializeAsync(CancellationToken cancellationToken = default)
    {
        if (!_config.Enabled)
        {
            _logger.LogInformation("GPS sensor is disabled in configuration");
            return;
        }

        try
        {
            _serialPort = new SerialPort(_config.SerialPort, _config.BaudRate)
            {
                ReadTimeout = 1000,
                WriteTimeout = 1000,
                NewLine = "\r\n"
            };

            _serialPort.Open();
            _health = HealthStatus.Healthy;

            // Start reading NMEA sentences
            _readTask = Task.Run(() => ReadNmeaLoop(_cts.Token), cancellationToken);

            _logger.LogInformation("GPS sensor initialized on {Port}", _config.SerialPort);
        }
        catch (Exception ex)
        {
            _health = HealthStatus.Unhealthy;
            _logger.LogError(ex, "Failed to initialize GPS sensor");
            throw;
        }

        await Task.CompletedTask;
    }

    public async Task ShutdownAsync(CancellationToken cancellationToken = default)
    {
        _cts.Cancel();
        
        if (_readTask != null)
        {
            try
            {
                await _readTask.WaitAsync(TimeSpan.FromSeconds(5), cancellationToken);
            }
            catch (TimeoutException)
            {
                _logger.LogWarning("GPS read task did not complete in time");
            }
        }

        _serialPort?.Close();
        _serialPort?.Dispose();
        _health = HealthStatus.Offline;
        
        _logger.LogInformation("GPS sensor shut down");
    }

    public async Task<bool> SelfTestAsync(CancellationToken cancellationToken = default)
    {
        // Wait for at least one valid position
        var timeout = DateTimeOffset.UtcNow.AddSeconds(10);
        
        while (DateTimeOffset.UtcNow < timeout && !cancellationToken.IsCancellationRequested)
        {
            if (_lastPosition != null)
            {
                return true;
            }
            await Task.Delay(500, cancellationToken);
        }

        return false;
    }

    public Task<GeoPosition?> GetPositionAsync(CancellationToken cancellationToken = default)
    {
        return Task.FromResult(_lastPosition);
    }

    private async Task ReadNmeaLoop(CancellationToken cancellationToken)
    {
        while (!cancellationToken.IsCancellationRequested && _serialPort?.IsOpen == true)
        {
            try
            {
                var line = _serialPort.ReadLine();
                ProcessNmeaSentence(line);
            }
            catch (TimeoutException)
            {
                // Normal timeout, continue
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error reading GPS data");
                _health = HealthStatus.Degraded;
                await Task.Delay(1000, cancellationToken);
            }
        }
    }

    private void ProcessNmeaSentence(string sentence)
    {
        if (string.IsNullOrEmpty(sentence) || !sentence.StartsWith('$'))
            return;

        try
        {
            var parts = sentence.Split(',');
            var sentenceType = parts[0];

            switch (sentenceType)
            {
                case "$GPGGA" or "$GNGGA":
                    ParseGga(parts);
                    break;
                case "$GPGSV" or "$GNGSV":
                    ParseGsv(parts);
                    break;
                case "$GPRMC" or "$GNRMC":
                    ParseRmc(parts);
                    break;
            }
        }
        catch (Exception ex)
        {
            _logger.LogDebug(ex, "Failed to parse NMEA sentence: {Sentence}", sentence);
        }
    }

    private void ParseGga(string[] parts)
    {
        // GGA - Global Positioning System Fix Data
        if (parts.Length < 15)
            return;

        var fixQuality = int.Parse(parts[6]);
        _hasFix = fixQuality > 0;

        if (!_hasFix)
            return;

        var latitude = ParseCoordinate(parts[2], parts[3]);
        var longitude = ParseCoordinate(parts[4], parts[5]);
        var altitude = double.TryParse(parts[9], out var alt) ? alt : 0;
        _hdop = double.TryParse(parts[8], out var hdop) ? hdop : 99.9;

        _lastPosition = new GeoPosition
        {
            Latitude = latitude,
            Longitude = longitude,
            Altitude = altitude,
            Accuracy = _hdop * 2.5, // Rough estimate of accuracy
            Timestamp = DateTimeOffset.UtcNow
        };

        _health = HealthStatus.Healthy;
    }

    private void ParseGsv(string[] parts)
    {
        // GSV - Satellites in View
        if (parts.Length >= 4 && int.TryParse(parts[3], out var satellites))
        {
            _satellitesInView = satellites;
        }
    }

    private void ParseRmc(string[] parts)
    {
        // RMC - Recommended Minimum Navigation Information
        // Can be used for timestamp and speed information
    }

    private static double ParseCoordinate(string value, string direction)
    {
        if (string.IsNullOrEmpty(value))
            return 0;

        // Format: DDDMM.MMMM or DDMM.MMMM
        var dotIndex = value.IndexOf('.');
        var degreeLength = dotIndex - 2;
        
        var degrees = double.Parse(value[..degreeLength]);
        var minutes = double.Parse(value[degreeLength..]);
        
        var result = degrees + minutes / 60;
        
        if (direction == "S" || direction == "W")
            result = -result;

        return result;
    }

    public void Dispose()
    {
        _cts.Cancel();
        _cts.Dispose();
        _serialPort?.Dispose();
    }
}
