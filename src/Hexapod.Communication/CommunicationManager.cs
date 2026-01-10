using System.Net.NetworkInformation;
using Hexapod.Communication.AzureIoT;
using Hexapod.Communication.LoRaWAN;
using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Events;
using Hexapod.Core.Models;
using Hexapod.Core.Services;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Communication;

/// <summary>
/// Interface for unified communication management.
/// </summary>
public interface ICommunicationManager
{
    /// <summary>
    /// Gets whether any communication channel is available.
    /// </summary>
    bool IsConnected { get; }

    /// <summary>
    /// Gets the current active communication channel.
    /// </summary>
    CommunicationChannel ActiveChannel { get; }

    /// <summary>
    /// Gets whether WiFi is currently available.
    /// </summary>
    bool IsWifiAvailable { get; }

    /// <summary>
    /// Gets whether LoRaWAN is joined.
    /// </summary>
    bool IsLoRaWanJoined { get; }

    /// <summary>
    /// Initializes all communication channels.
    /// </summary>
    Task InitializeAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Sends telemetry using the best available channel.
    /// </summary>
    Task<bool> SendTelemetryAsync<T>(T telemetry, CancellationToken cancellationToken = default) where T : class;

    /// <summary>
    /// Sends position update using the best available channel.
    /// </summary>
    Task<bool> SendPositionAsync(GeoPosition position, CancellationToken cancellationToken = default);

    /// <summary>
    /// Sends status report using the best available channel.
    /// </summary>
    Task<bool> SendStatusAsync(StatusReport status, CancellationToken cancellationToken = default);

    /// <summary>
    /// Sends an alert using all available channels.
    /// </summary>
    Task SendAlertAsync(string alertType, string message, CancellationToken cancellationToken = default);

    /// <summary>
    /// Forces a connectivity check and channel switch if needed.
    /// </summary>
    Task RefreshConnectivityAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Disconnects all communication channels.
    /// </summary>
    Task DisconnectAsync(CancellationToken cancellationToken = default);
}

/// <summary>
/// Available communication channels.
/// </summary>
public enum CommunicationChannel
{
    None,
    Wifi,       // Azure IoT Hub over WiFi
    LoRaWan     // LoRaWAN for long-range low-bandwidth
}

/// <summary>
/// Manages communication channels with WiFi priority and LoRaWAN fallback.
/// </summary>
public sealed class CommunicationManager : ICommunicationManager, IDisposable
{
    private readonly IAzureIoTHubService _iotHub;
    private readonly ILoRaWanService _loRaWan;
    private readonly IEventBus _eventBus;
    private readonly ILogger<CommunicationManager> _logger;
    private readonly CommunicationConfiguration _config;
    
    private readonly Timer _connectivityCheckTimer;
    private readonly SemaphoreSlim _channelSwitchLock = new(1, 1);
    
    private CommunicationChannel _activeChannel = CommunicationChannel.None;
    private bool _isWifiAvailable;
    private bool _wifiWasAvailable;
    private bool _disposed;

    public bool IsConnected => _activeChannel != CommunicationChannel.None;
    public CommunicationChannel ActiveChannel => _activeChannel;
    public bool IsWifiAvailable => _isWifiAvailable;
    public bool IsLoRaWanJoined => _loRaWan.IsJoined;

    public CommunicationManager(
        IAzureIoTHubService iotHub,
        ILoRaWanService loRaWan,
        IEventBus eventBus,
        IOptions<HexapodConfiguration> config,
        ILogger<CommunicationManager> logger)
    {
        _iotHub = iotHub;
        _loRaWan = loRaWan;
        _eventBus = eventBus;
        _logger = logger;
        _config = config.Value.Communication;

        // Check connectivity every 30 seconds
        _connectivityCheckTimer = new Timer(
            OnConnectivityCheckTimer,
            null,
            TimeSpan.FromSeconds(30),
            TimeSpan.FromSeconds(30));
    }

    public async Task InitializeAsync(CancellationToken cancellationToken = default)
    {
        _logger.LogInformation("Initializing communication channels...");

        // Check initial WiFi state
        _isWifiAvailable = await CheckWifiConnectivityAsync(cancellationToken);
        _wifiWasAvailable = _isWifiAvailable;

        // Initialize LoRaWAN in background (doesn't block WiFi)
        _ = InitializeLoRaWanAsync(cancellationToken);

        // Try WiFi first
        if (_isWifiAvailable)
        {
            try
            {
                await _iotHub.ConnectAsync(cancellationToken);
                if (_iotHub.IsConnected)
                {
                    _activeChannel = CommunicationChannel.Wifi;
                    _logger.LogInformation("Primary channel established: WiFi (Azure IoT Hub)");
                    PublishChannelChangedEvent(CommunicationChannel.None, CommunicationChannel.Wifi);
                    return;
                }
            }
            catch (Exception ex)
            {
                _logger.LogWarning(ex, "Failed to connect to Azure IoT Hub");
            }
        }

        // Fall back to LoRaWAN if available
        if (_loRaWan.IsJoined)
        {
            _activeChannel = CommunicationChannel.LoRaWan;
            _logger.LogInformation("Fallback channel established: LoRaWAN");
            PublishChannelChangedEvent(CommunicationChannel.None, CommunicationChannel.LoRaWan);
        }
        else
        {
            _logger.LogWarning("No communication channel available");
        }
    }

    public async Task<bool> SendTelemetryAsync<T>(T telemetry, CancellationToken cancellationToken = default) 
        where T : class
    {
        await EnsureBestChannelAsync(cancellationToken);

        switch (_activeChannel)
        {
            case CommunicationChannel.Wifi:
                try
                {
                    await _iotHub.SendTelemetryAsync("telemetry", telemetry, cancellationToken);
                    return true;
                }
                catch (Exception ex)
                {
                    _logger.LogWarning(ex, "Failed to send telemetry via WiFi, attempting fallback");
                    await TryFallbackToLoRaWanAsync(cancellationToken);
                    // Telemetry is too large for LoRaWAN, queue for later
                    return false;
                }

            case CommunicationChannel.LoRaWan:
                // LoRaWAN has limited bandwidth, skip large telemetry
                _logger.LogDebug("Skipping large telemetry on LoRaWAN channel");
                return false;

            default:
                _logger.LogWarning("No channel available for telemetry");
                return false;
        }
    }

    public async Task<bool> SendPositionAsync(GeoPosition position, CancellationToken cancellationToken = default)
    {
        await EnsureBestChannelAsync(cancellationToken);

        switch (_activeChannel)
        {
            case CommunicationChannel.Wifi:
                try
                {
                    await _iotHub.SendTelemetryAsync("position", new
                    {
                        type = "position",
                        latitude = position.Latitude,
                        longitude = position.Longitude,
                        altitude = position.Altitude,
                        timestamp = DateTimeOffset.UtcNow
                    }, cancellationToken);
                    return true;
                }
                catch (Exception ex)
                {
                    _logger.LogWarning(ex, "Failed to send position via WiFi, trying LoRaWAN");
                    await TryFallbackToLoRaWanAsync(cancellationToken);
                    return await SendPositionViaLoRaWanAsync(position, cancellationToken);
                }

            case CommunicationChannel.LoRaWan:
                return await SendPositionViaLoRaWanAsync(position, cancellationToken);

            default:
                _logger.LogWarning("No channel available for position update");
                return false;
        }
    }

    public async Task<bool> SendStatusAsync(StatusReport status, CancellationToken cancellationToken = default)
    {
        await EnsureBestChannelAsync(cancellationToken);

        switch (_activeChannel)
        {
            case CommunicationChannel.Wifi:
                try
                {
                    await _iotHub.SendTelemetryAsync("status", new
                    {
                        type = "status",
                        battery = status.BatteryPercent,
                        mode = status.CurrentMode.ToString(),
                        state = status.CurrentState.ToString(),
                        flags = (int)status.Flags,
                        timestamp = DateTimeOffset.UtcNow
                    }, cancellationToken);
                    return true;
                }
                catch (Exception ex)
                {
                    _logger.LogWarning(ex, "Failed to send status via WiFi, trying LoRaWAN");
                    await TryFallbackToLoRaWanAsync(cancellationToken);
                    return await SendStatusViaLoRaWanAsync(status, cancellationToken);
                }

            case CommunicationChannel.LoRaWan:
                return await SendStatusViaLoRaWanAsync(status, cancellationToken);

            default:
                _logger.LogWarning("No channel available for status update");
                return false;
        }
    }

    public async Task SendAlertAsync(string alertType, string message, CancellationToken cancellationToken = default)
    {
        _logger.LogWarning("Sending alert: {Type} - {Message}", alertType, message);

        var tasks = new List<Task>();

        // Send via WiFi if available
        if (_iotHub.IsConnected)
        {
            tasks.Add(Task.Run(async () =>
            {
                try
                {
                    await _iotHub.SendTelemetryAsync("alert", new
                    {
                        type = "alert",
                        alertType,
                        message,
                        priority = "high",
                        timestamp = DateTimeOffset.UtcNow
                    }, cancellationToken);
                }
                catch (Exception ex)
                {
                    _logger.LogError(ex, "Failed to send alert via WiFi");
                }
            }, cancellationToken));
        }

        // Always try to send via LoRaWAN for critical alerts
        if (_loRaWan.IsJoined)
        {
            tasks.Add(Task.Run(async () =>
            {
                try
                {
                    // Send compact alert format for LoRaWAN
                    var alertData = $"A:{alertType}:{message.Substring(0, Math.Min(message.Length, 50))}";
                    await _loRaWan.SendAsync(alertData, port: 10, confirmed: true, cancellationToken);
                }
                catch (Exception ex)
                {
                    _logger.LogError(ex, "Failed to send alert via LoRaWAN");
                }
            }, cancellationToken));
        }

        if (tasks.Count > 0)
        {
            await Task.WhenAll(tasks);
        }
        else
        {
            _logger.LogError("No channel available to send alert!");
        }
    }

    public async Task RefreshConnectivityAsync(CancellationToken cancellationToken = default)
    {
        await _channelSwitchLock.WaitAsync(cancellationToken);
        try
        {
            var previousChannel = _activeChannel;
            _isWifiAvailable = await CheckWifiConnectivityAsync(cancellationToken);

            // WiFi became available - switch to it
            if (_isWifiAvailable && !_wifiWasAvailable)
            {
                _logger.LogInformation("WiFi connectivity restored, switching from LoRaWAN");
                await SwitchToWifiAsync(cancellationToken);
            }
            // WiFi lost - fall back to LoRaWAN
            else if (!_isWifiAvailable && _wifiWasAvailable)
            {
                _logger.LogWarning("WiFi connectivity lost, switching to LoRaWAN");
                await TryFallbackToLoRaWanAsync(cancellationToken);
            }
            // WiFi available but IoT Hub disconnected - try to reconnect
            else if (_isWifiAvailable && !_iotHub.IsConnected && _activeChannel == CommunicationChannel.Wifi)
            {
                _logger.LogInformation("Attempting to reconnect to Azure IoT Hub");
                await SwitchToWifiAsync(cancellationToken);
            }

            _wifiWasAvailable = _isWifiAvailable;

            if (previousChannel != _activeChannel)
            {
                PublishChannelChangedEvent(previousChannel, _activeChannel);
            }
        }
        finally
        {
            _channelSwitchLock.Release();
        }
    }

    private async Task EnsureBestChannelAsync(CancellationToken cancellationToken)
    {
        // Quick check if we should switch channels
        if (_activeChannel == CommunicationChannel.LoRaWan && _isWifiAvailable && _iotHub.IsConnected)
        {
            await _channelSwitchLock.WaitAsync(cancellationToken);
            try
            {
                if (_iotHub.IsConnected)
                {
                    var previous = _activeChannel;
                    _activeChannel = CommunicationChannel.Wifi;
                    _logger.LogInformation("Switched back to WiFi channel");
                    PublishChannelChangedEvent(previous, _activeChannel);
                }
            }
            finally
            {
                _channelSwitchLock.Release();
            }
        }
    }

    private async Task SwitchToWifiAsync(CancellationToken cancellationToken)
    {
        try
        {
            if (!_iotHub.IsConnected)
            {
                await _iotHub.ConnectAsync(cancellationToken);
            }

            if (_iotHub.IsConnected)
            {
                _activeChannel = CommunicationChannel.Wifi;
                _logger.LogInformation("Switched to WiFi channel (Azure IoT Hub)");
            }
        }
        catch (Exception ex)
        {
            _logger.LogWarning(ex, "Failed to switch to WiFi, staying on current channel");
        }
    }

    private async Task TryFallbackToLoRaWanAsync(CancellationToken cancellationToken)
    {
        if (_loRaWan.IsJoined)
        {
            _activeChannel = CommunicationChannel.LoRaWan;
            _logger.LogInformation("Switched to LoRaWAN fallback channel");
        }
        else
        {
            // Try to join if not already joined
            try
            {
                await _loRaWan.JoinNetworkAsync(cancellationToken);
                if (_loRaWan.IsJoined)
                {
                    _activeChannel = CommunicationChannel.LoRaWan;
                    _logger.LogInformation("Joined LoRaWAN network, switched to fallback channel");
                }
                else
                {
                    _activeChannel = CommunicationChannel.None;
                    _logger.LogError("No communication channel available");
                }
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to join LoRaWAN network");
                _activeChannel = CommunicationChannel.None;
            }
        }
    }

    private async Task<bool> SendPositionViaLoRaWanAsync(GeoPosition position, CancellationToken cancellationToken)
    {
        if (!_loRaWan.IsJoined)
            return false;

        try
        {
            // Compact position format for LoRaWAN: lat(4 bytes) + lon(4 bytes) + alt(2 bytes) = 10 bytes
            var data = new byte[10];
            var lat = (int)(position.Latitude * 1_000_000);
            var lon = (int)(position.Longitude * 1_000_000);
            var alt = (short)position.Altitude;
            
            BitConverter.GetBytes(lat).CopyTo(data, 0);
            BitConverter.GetBytes(lon).CopyTo(data, 4);
            BitConverter.GetBytes(alt).CopyTo(data, 8);
            
            return await _loRaWan.SendAsync(data, port: 2, confirmed: false, cancellationToken);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Failed to send position via LoRaWAN");
            return false;
        }
    }

    private async Task<bool> SendStatusViaLoRaWanAsync(StatusReport status, CancellationToken cancellationToken)
    {
        if (!_loRaWan.IsJoined)
            return false;

        try
        {
            // Compact status format for LoRaWAN: battery(1) + mode(1) + state(1) + flags(1) = 4 bytes
            var data = new byte[4];
            data[0] = status.BatteryPercent;
            data[1] = (byte)status.CurrentMode;
            data[2] = (byte)status.CurrentState;
            data[3] = (byte)status.Flags;
            
            return await _loRaWan.SendAsync(data, port: 3, confirmed: false, cancellationToken);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Failed to send status via LoRaWAN");
            return false;
        }
    }

    private async Task InitializeLoRaWanAsync(CancellationToken cancellationToken)
    {
        try
        {
            await _loRaWan.JoinNetworkAsync(cancellationToken);
            if (_loRaWan.IsJoined)
            {
                _logger.LogInformation("LoRaWAN network joined (backup channel ready)");
                
                // If WiFi isn't working, switch to LoRaWAN
                if (_activeChannel == CommunicationChannel.None)
                {
                    _activeChannel = CommunicationChannel.LoRaWan;
                    PublishChannelChangedEvent(CommunicationChannel.None, CommunicationChannel.LoRaWan);
                }
            }
        }
        catch (Exception ex)
        {
            _logger.LogWarning(ex, "Failed to join LoRaWAN network");
        }
    }

    private async Task<bool> CheckWifiConnectivityAsync(CancellationToken cancellationToken)
    {
        try
        {
            // Check if any network interface is up with internet
            var interfaces = NetworkInterface.GetAllNetworkInterfaces()
                .Where(ni => ni.OperationalStatus == OperationalStatus.Up)
                .Where(ni => ni.NetworkInterfaceType == NetworkInterfaceType.Wireless80211 ||
                             ni.NetworkInterfaceType == NetworkInterfaceType.Ethernet);

            if (!interfaces.Any())
                return false;

            // Try to ping a known endpoint
            using var ping = new Ping();
            var reply = await ping.SendPingAsync("8.8.8.8", 3000);
            return reply.Status == IPStatus.Success;
        }
        catch
        {
            return false;
        }
    }

    private void OnConnectivityCheckTimer(object? state)
    {
        _ = RefreshConnectivityAsync(CancellationToken.None);
    }

    private void PublishChannelChangedEvent(CommunicationChannel previous, CommunicationChannel current)
    {
        _eventBus.Publish(new CommunicationChannelChangedEvent
        {
            EventId = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow,
            Source = nameof(CommunicationManager),
            PreviousChannel = previous,
            CurrentChannel = current,
            IsWifiAvailable = _isWifiAvailable,
            IsLoRaWanJoined = _loRaWan.IsJoined
        });
    }

    /// <inheritdoc/>
    public async Task DisconnectAsync(CancellationToken cancellationToken = default)
    {
        _logger.LogInformation("Disconnecting all communication channels...");

        try
        {
            if (_iotHub.IsConnected)
            {
                await _iotHub.DisconnectAsync(cancellationToken);
            }
        }
        catch (Exception ex)
        {
            _logger.LogWarning(ex, "Error disconnecting Azure IoT Hub");
        }

        _activeChannel = CommunicationChannel.None;
        _logger.LogInformation("Communication channels disconnected");
    }

    public void Dispose()
    {
        if (!_disposed)
        {
            _connectivityCheckTimer.Dispose();
            _channelSwitchLock.Dispose();
            _disposed = true;
        }
    }
}

/// <summary>
/// Event when communication channel changes.
/// </summary>
public record CommunicationChannelChangedEvent : HexapodEvent
{
    public required CommunicationChannel PreviousChannel { get; init; }
    public required CommunicationChannel CurrentChannel { get; init; }
    public required bool IsWifiAvailable { get; init; }
    public required bool IsLoRaWanJoined { get; init; }
}
