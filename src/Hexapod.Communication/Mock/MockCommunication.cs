using System.Text.Json;
using Hexapod.Communication.AzureIoT;
using Hexapod.Communication.LoRaWAN;
using Hexapod.Core.Configuration;
using Hexapod.Core.Models;
using Microsoft.Azure.Devices.Client;
using Microsoft.Azure.Devices.Shared;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Communication.Mock;

/// <summary>
/// Mock Azure IoT Hub service for development and testing without cloud connectivity.
/// </summary>
public sealed class MockAzureIoTHubService : IAzureIoTHubService, IDisposable
{
    private readonly ILogger<MockAzureIoTHubService> _logger;
    private readonly MockModeConfiguration _mockConfig;
    private bool _isConnected;
    private readonly Dictionary<string, Func<MethodRequest, Task<MethodResponse>>> _methodHandlers = new();
    private Func<TwinCollection, Task>? _desiredPropertyHandler;
    private TwinCollection _reportedProperties = new();
    private readonly List<MockTelemetryMessage> _telemetryLog = new();
    private int _messageCount;

    public MockAzureIoTHubService(
        IOptions<HexapodConfiguration> config,
        ILogger<MockAzureIoTHubService> logger)
    {
        _logger = logger;
        _mockConfig = config.Value.MockMode;
    }

    public bool IsConnected => _isConnected;

    /// <summary>
    /// Gets all telemetry messages sent during this session (for debugging).
    /// </summary>
    public IReadOnlyList<MockTelemetryMessage> TelemetryLog => _telemetryLog;

    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        _isConnected = true;
        _logger.LogInformation("☁️ Mock Azure IoT Hub connected (simulated)");
        return Task.CompletedTask;
    }

    public Task DisconnectAsync(CancellationToken cancellationToken = default)
    {
        _isConnected = false;
        _logger.LogInformation("☁️ Mock Azure IoT Hub disconnected");
        return Task.CompletedTask;
    }

    public Task SendTelemetryAsync<T>(string topic, T data, CancellationToken cancellationToken = default)
    {
        _messageCount++;
        var json = JsonSerializer.Serialize(data, new JsonSerializerOptions { WriteIndented = false });
        
        var message = new MockTelemetryMessage
        {
            MessageId = _messageCount,
            Topic = topic,
            Payload = json,
            Timestamp = DateTimeOffset.UtcNow
        };
        _telemetryLog.Add(message);

        // Keep only last 1000 messages to prevent memory issues
        if (_telemetryLog.Count > 1000)
            _telemetryLog.RemoveAt(0);

        if (_mockConfig.VerboseLogging)
        {
            _logger.LogDebug("☁️ IoT Telemetry [{Topic}]: {Json}", topic, 
                json.Length > 200 ? json[..200] + "..." : json);
        }

        return Task.CompletedTask;
    }

    public async Task SendTelemetryBatchAsync<T>(string topic, IEnumerable<T> data, CancellationToken cancellationToken = default)
    {
        foreach (var item in data)
        {
            await SendTelemetryAsync(topic, item, cancellationToken);
        }
        
        _logger.LogDebug("☁️ Mock IoT batch sent to topic '{Topic}'", topic);
    }

    public Task UpdateReportedPropertiesAsync(TwinCollection properties, CancellationToken cancellationToken = default)
    {
        foreach (KeyValuePair<string, object> prop in properties)
        {
            _reportedProperties[prop.Key] = prop.Value;
        }

        if (_mockConfig.VerboseLogging)
        {
            _logger.LogDebug("☁️ Mock Twin properties updated: {Props}",
                JsonSerializer.Serialize(properties));
        }

        return Task.CompletedTask;
    }

    public Task<Twin> GetTwinAsync(CancellationToken cancellationToken = default)
    {
        // Create a mock twin - note: Twin class has internal constructors,
        // so we return a mock structure via the reported properties
        _logger.LogDebug("☁️ Mock Twin requested");
        
        // Twin cannot be easily instantiated - return via exception with meaningful message
        // In real testing scenarios, consider using a wrapper interface
        throw new NotImplementedException(
            "Mock Twin not available. Use GetMockReportedProperties() instead for testing.");
    }

    /// <summary>
    /// Gets the mock reported properties (for testing).
    /// </summary>
    public TwinCollection GetMockReportedProperties() => _reportedProperties;

    public void RegisterMethodHandler(string methodName, Func<MethodRequest, Task<MethodResponse>> handler)
    {
        _methodHandlers[methodName] = handler;
        _logger.LogInformation("☁️ Mock method handler registered: {Method}", methodName);
    }

    public void RegisterDesiredPropertyHandler(Func<TwinCollection, Task> handler)
    {
        _desiredPropertyHandler = handler;
        _logger.LogInformation("☁️ Mock desired property handler registered");
    }

    /// <summary>
    /// Simulates receiving a direct method call (for testing).
    /// </summary>
    public async Task<MethodResponse?> SimulateMethodCallAsync(string methodName, string payload)
    {
        if (_methodHandlers.TryGetValue(methodName, out var handler))
        {
            var request = new MethodRequest(methodName, System.Text.Encoding.UTF8.GetBytes(payload));
            _logger.LogInformation("☁️ Simulating method call: {Method}", methodName);
            return await handler(request);
        }
        
        _logger.LogWarning("☁️ No handler for method: {Method}", methodName);
        return null;
    }

    /// <summary>
    /// Simulates receiving desired property update (for testing).
    /// </summary>
    public async Task SimulateDesiredPropertyUpdateAsync(TwinCollection properties)
    {
        if (_desiredPropertyHandler != null)
        {
            _logger.LogInformation("☁️ Simulating desired property update");
            await _desiredPropertyHandler(properties);
        }
    }

    public void Dispose()
    {
        _isConnected = false;
        _logger.LogInformation("☁️ Mock Azure IoT Hub service disposed");
    }
}

/// <summary>
/// Record of a mock telemetry message for debugging.
/// </summary>
public record MockTelemetryMessage
{
    public int MessageId { get; init; }
    public required string Topic { get; init; }
    public required string Payload { get; init; }
    public DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Mock LoRaWAN service for development and testing without hardware.
/// </summary>
public sealed class MockLoRaWanService : ILoRaWanService, IDisposable
{
    private readonly ILogger<MockLoRaWanService> _logger;
    private readonly MockModeConfiguration _mockConfig;
    private bool _isJoined;
    private int _signalStrength = -70; // Good signal
    private readonly Queue<LoRaWanMessage> _receiveQueue = new();
    private readonly List<MockLoRaMessage> _sentMessages = new();
    private int _messageCount;

    public MockLoRaWanService(
        IOptions<HexapodConfiguration> config,
        ILogger<MockLoRaWanService> logger)
    {
        _logger = logger;
        _mockConfig = config.Value.MockMode;
    }

    public bool IsJoined => _isJoined;
    public int SignalStrength => _signalStrength;

    /// <summary>
    /// Gets all sent messages during this session (for debugging).
    /// </summary>
    public IReadOnlyList<MockLoRaMessage> SentMessages => _sentMessages;

    public Task JoinNetworkAsync(CancellationToken cancellationToken = default)
    {
        _isJoined = true;
        _logger.LogInformation("📡 Mock LoRaWAN network joined (simulated)");
        _logger.LogInformation("   DevEUI: 00:00:00:00:00:00:00:01");
        _logger.LogInformation("   RSSI: {RSSI} dBm", _signalStrength);
        return Task.CompletedTask;
    }

    public Task<bool> SendAsync(byte[] data, int port = 1, bool confirmed = false, CancellationToken cancellationToken = default)
    {
        if (!_isJoined)
        {
            _logger.LogWarning("📡 Mock LoRaWAN: Cannot send - not joined");
            return Task.FromResult(false);
        }

        _messageCount++;
        var message = new MockLoRaMessage
        {
            MessageId = _messageCount,
            Data = data,
            Port = port,
            Confirmed = confirmed,
            Timestamp = DateTimeOffset.UtcNow
        };
        _sentMessages.Add(message);

        // Keep only last 500 messages
        if (_sentMessages.Count > 500)
            _sentMessages.RemoveAt(0);

        if (_mockConfig.VerboseLogging)
        {
            _logger.LogDebug("📡 LoRa TX [{Port}]: {Bytes} bytes, {Confirmed}",
                port, data.Length, confirmed ? "confirmed" : "unconfirmed");
        }

        return Task.FromResult(true);
    }

    public Task<bool> SendAsync(string message, int port = 1, bool confirmed = false, CancellationToken cancellationToken = default)
    {
        var data = System.Text.Encoding.UTF8.GetBytes(message);
        return SendAsync(data, port, confirmed, cancellationToken);
    }

    public Task<LoRaWanMessage?> ReceiveAsync(CancellationToken cancellationToken = default)
    {
        if (_receiveQueue.TryDequeue(out var message))
        {
            _logger.LogDebug("📡 LoRa RX: {Bytes} bytes on port {Port}",
                message.Data.Length, message.Port);
            return Task.FromResult<LoRaWanMessage?>(message);
        }

        return Task.FromResult<LoRaWanMessage?>(null);
    }

    /// <summary>
    /// Simulates receiving a downlink message (for testing).
    /// </summary>
    public void SimulateDownlink(byte[] data, int port = 1)
    {
        var message = new LoRaWanMessage
        {
            Data = data,
            Port = port,
            Rssi = _signalStrength,
            Snr = 10,
            Timestamp = DateTimeOffset.UtcNow
        };
        _receiveQueue.Enqueue(message);
        _logger.LogInformation("📡 Simulating LoRa downlink: {Bytes} bytes", data.Length);
    }

    /// <summary>
    /// Sets the simulated signal strength (for testing coverage scenarios).
    /// </summary>
    public void SetSignalStrength(int rssi)
    {
        _signalStrength = rssi;
        _logger.LogDebug("📡 Mock RSSI set to {RSSI} dBm", rssi);
    }

    public void Dispose()
    {
        _isJoined = false;
        _logger.LogInformation("📡 Mock LoRaWAN service disposed");
    }
}

/// <summary>
/// Record of a mock LoRa message for debugging.
/// </summary>
public record MockLoRaMessage
{
    public int MessageId { get; init; }
    public required byte[] Data { get; init; }
    public int Port { get; init; }
    public bool Confirmed { get; init; }
    public DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// Mock communication manager for development and testing.
/// </summary>
public sealed class MockCommunicationManager : ICommunicationManager
{
    private readonly ILogger<MockCommunicationManager> _logger;
    private readonly MockModeConfiguration _mockConfig;
    private readonly MockAzureIoTHubService _iotHub;
    private readonly MockLoRaWanService _loRaWan;
    private bool _isConnected;
    private CommunicationChannel _activeChannel = CommunicationChannel.None;

    public MockCommunicationManager(
        IOptions<HexapodConfiguration> config,
        MockAzureIoTHubService iotHub,
        MockLoRaWanService loRaWan,
        ILogger<MockCommunicationManager> logger)
    {
        _logger = logger;
        _mockConfig = config.Value.MockMode;
        _iotHub = iotHub;
        _loRaWan = loRaWan;
    }

    public bool IsConnected => _isConnected;
    public CommunicationChannel ActiveChannel => _activeChannel;
    public bool IsWifiAvailable => true; // Always available in mock
    public bool IsLoRaWanJoined => _loRaWan.IsJoined;

    public async Task InitializeAsync(CancellationToken cancellationToken = default)
    {
        await _iotHub.ConnectAsync(cancellationToken);
        await _loRaWan.JoinNetworkAsync(cancellationToken);
        
        _isConnected = true;
        _activeChannel = CommunicationChannel.Wifi;
        
        _logger.LogInformation("🌐 Mock Communication Manager initialized");
        _logger.LogInformation("   WiFi: Available (simulated)");
        _logger.LogInformation("   LoRaWAN: Joined (simulated)");
        _logger.LogInformation("   Active Channel: {Channel}", _activeChannel);
    }

    public async Task<bool> SendTelemetryAsync<T>(T telemetry, CancellationToken cancellationToken = default) where T : class
    {
        await _iotHub.SendTelemetryAsync("telemetry", telemetry, cancellationToken);
        return true;
    }

    public async Task<bool> SendPositionAsync(GeoPosition position, CancellationToken cancellationToken = default)
    {
        await _iotHub.SendTelemetryAsync("position", position, cancellationToken);
        return true;
    }

    public async Task<bool> SendStatusAsync(StatusReport status, CancellationToken cancellationToken = default)
    {
        await _iotHub.SendTelemetryAsync("status", status, cancellationToken);
        return true;
    }

    public async Task SendAlertAsync(string alertType, string message, CancellationToken cancellationToken = default)
    {
        var alert = new { Type = alertType, Message = message, Timestamp = DateTimeOffset.UtcNow };
        
        // Send via both channels in mock mode
        await _iotHub.SendTelemetryAsync("alert", alert, cancellationToken);
        await _loRaWan.SendAsync($"ALERT:{alertType}:{message}", 2, true, cancellationToken);
        
        _logger.LogWarning("⚠️ Mock Alert sent: [{Type}] {Message}", alertType, message);
    }

    public Task RefreshConnectivityAsync(CancellationToken cancellationToken = default)
    {
        _logger.LogDebug("🌐 Mock connectivity check - all channels available");
        return Task.CompletedTask;
    }

    public async Task DisconnectAsync(CancellationToken cancellationToken = default)
    {
        await _iotHub.DisconnectAsync(cancellationToken);
        _isConnected = false;
        _activeChannel = CommunicationChannel.None;
        _logger.LogInformation("🌐 Mock Communication Manager disconnected");
    }
}
