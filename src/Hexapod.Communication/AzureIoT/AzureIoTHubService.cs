using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Text.Json;
using Hexapod.Core.Configuration;
using Hexapod.Core.Events;
using Hexapod.Core.Models;
using Hexapod.Core.Services;
using Microsoft.Azure.Devices.Client;
using Microsoft.Azure.Devices.Provisioning.Client;
using Microsoft.Azure.Devices.Provisioning.Client.Transport;
using Microsoft.Azure.Devices.Shared;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Communication.AzureIoT;

/// <summary>
/// Interface for Azure IoT Hub communication service.
/// </summary>
public interface IAzureIoTHubService
{
    /// <summary>
    /// Gets whether the device is connected to IoT Hub.
    /// </summary>
    bool IsConnected { get; }

    /// <summary>
    /// Connects to Azure IoT Hub.
    /// </summary>
    Task ConnectAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Disconnects from Azure IoT Hub.
    /// </summary>
    Task DisconnectAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Sends telemetry data to IoT Hub.
    /// </summary>
    Task SendTelemetryAsync<T>(string topic, T data, CancellationToken cancellationToken = default);

    /// <summary>
    /// Sends a batch of telemetry messages.
    /// </summary>
    Task SendTelemetryBatchAsync<T>(string topic, IEnumerable<T> data, CancellationToken cancellationToken = default);

    /// <summary>
    /// Updates reported properties in device twin.
    /// </summary>
    Task UpdateReportedPropertiesAsync(TwinCollection properties, CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the current device twin.
    /// </summary>
    Task<Twin> GetTwinAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Registers a handler for direct method calls.
    /// </summary>
    void RegisterMethodHandler(string methodName, Func<MethodRequest, Task<MethodResponse>> handler);

    /// <summary>
    /// Registers a handler for desired property updates.
    /// </summary>
    void RegisterDesiredPropertyHandler(Func<TwinCollection, Task> handler);
}

/// <summary>
/// Azure IoT Hub communication service implementation.
/// </summary>
public sealed class AzureIoTHubService : IAzureIoTHubService, IDisposable
{
    private readonly ILogger<AzureIoTHubService> _logger;
    private readonly IEventBus _eventBus;
    private readonly AzureIoTConfig _config;
    private DeviceClient? _deviceClient;
    private bool _isConnected;
    private readonly Dictionary<string, Func<MethodRequest, Task<MethodResponse>>> _methodHandlers = new();
    private Func<TwinCollection, Task>? _desiredPropertyHandler;
    private readonly SemaphoreSlim _connectionLock = new(1, 1);

    public AzureIoTHubService(
        IEventBus eventBus,
        IOptions<HexapodConfiguration> config,
        ILogger<AzureIoTHubService> logger)
    {
        _eventBus = eventBus;
        _config = config.Value.Communication.AzureIoT;
        _logger = logger;
    }

    public bool IsConnected => _isConnected;

    public async Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        await _connectionLock.WaitAsync(cancellationToken);
        try
        {
            if (_isConnected)
            {
                _logger.LogDebug("Already connected to IoT Hub");
                return;
            }

            _logger.LogInformation("Connecting to Azure IoT Hub...");

            _deviceClient = await CreateDeviceClientAsync(cancellationToken);

            // Set connection status change handler
            _deviceClient.SetConnectionStatusChangesHandler(OnConnectionStatusChanged);

            // Open connection
            await _deviceClient.OpenAsync(cancellationToken);

            // Register method handlers
            foreach (var handler in _methodHandlers)
            {
                await _deviceClient.SetMethodHandlerAsync(handler.Key, 
                    (request, _) => handler.Value(request), null, cancellationToken);
            }

            // Register default method handlers
            await RegisterDefaultMethodHandlersAsync(cancellationToken);

            // Register desired property update callback
            if (_desiredPropertyHandler != null)
            {
                await _deviceClient.SetDesiredPropertyUpdateCallbackAsync(
                    (twin, _) => _desiredPropertyHandler(twin), null, cancellationToken);
            }

            _isConnected = true;
            _logger.LogInformation("Connected to Azure IoT Hub successfully");

            PublishConnectivityEvent(true);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Failed to connect to Azure IoT Hub");
            _isConnected = false;
            PublishConnectivityEvent(false);
            throw;
        }
        finally
        {
            _connectionLock.Release();
        }
    }

    public async Task DisconnectAsync(CancellationToken cancellationToken = default)
    {
        await _connectionLock.WaitAsync(cancellationToken);
        try
        {
            if (_deviceClient != null)
            {
                await _deviceClient.CloseAsync(cancellationToken);
                _deviceClient.Dispose();
                _deviceClient = null;
            }

            _isConnected = false;
            _logger.LogInformation("Disconnected from Azure IoT Hub");
            
            PublishConnectivityEvent(false);
        }
        finally
        {
            _connectionLock.Release();
        }
    }

    public async Task SendTelemetryAsync<T>(string topic, T data, CancellationToken cancellationToken = default)
    {
        if (!_isConnected || _deviceClient == null)
        {
            _logger.LogWarning("Cannot send telemetry - not connected");
            return;
        }

        try
        {
            var json = JsonSerializer.Serialize(data);
            var message = new Message(Encoding.UTF8.GetBytes(json))
            {
                ContentType = "application/json",
                ContentEncoding = "utf-8"
            };
            
            message.Properties.Add("topic", topic);
            message.Properties.Add("timestamp", DateTimeOffset.UtcNow.ToString("O"));

            await _deviceClient.SendEventAsync(message, cancellationToken);
            
            _logger.LogDebug("Sent telemetry to topic {Topic}: {Size} bytes", topic, json.Length);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Failed to send telemetry to topic {Topic}", topic);
            throw;
        }
    }

    public async Task SendTelemetryBatchAsync<T>(string topic, IEnumerable<T> data, CancellationToken cancellationToken = default)
    {
        if (!_isConnected || _deviceClient == null)
        {
            _logger.LogWarning("Cannot send telemetry batch - not connected");
            return;
        }

        var messages = data.Select(item =>
        {
            var json = JsonSerializer.Serialize(item);
            var message = new Message(Encoding.UTF8.GetBytes(json))
            {
                ContentType = "application/json",
                ContentEncoding = "utf-8"
            };
            message.Properties.Add("topic", topic);
            message.Properties.Add("timestamp", DateTimeOffset.UtcNow.ToString("O"));
            return message;
        }).ToList();

        try
        {
            await _deviceClient.SendEventBatchAsync(messages, cancellationToken);
            _logger.LogDebug("Sent batch of {Count} messages to topic {Topic}", messages.Count, topic);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Failed to send telemetry batch to topic {Topic}", topic);
            throw;
        }
        finally
        {
            foreach (var message in messages)
            {
                message.Dispose();
            }
        }
    }

    public async Task UpdateReportedPropertiesAsync(TwinCollection properties, CancellationToken cancellationToken = default)
    {
        if (!_isConnected || _deviceClient == null)
        {
            _logger.LogWarning("Cannot update reported properties - not connected");
            return;
        }

        try
        {
            await _deviceClient.UpdateReportedPropertiesAsync(properties, cancellationToken);
            _logger.LogDebug("Updated reported properties");
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Failed to update reported properties");
            throw;
        }
    }

    public async Task<Twin> GetTwinAsync(CancellationToken cancellationToken = default)
    {
        if (!_isConnected || _deviceClient == null)
        {
            throw new InvalidOperationException("Not connected to IoT Hub");
        }

        return await _deviceClient.GetTwinAsync(cancellationToken);
    }

    public void RegisterMethodHandler(string methodName, Func<MethodRequest, Task<MethodResponse>> handler)
    {
        _methodHandlers[methodName] = handler;
        _logger.LogDebug("Registered method handler for {MethodName}", methodName);
    }

    public void RegisterDesiredPropertyHandler(Func<TwinCollection, Task> handler)
    {
        _desiredPropertyHandler = handler;
        _logger.LogDebug("Registered desired property handler");
    }

    private async Task<DeviceClient> CreateDeviceClientAsync(CancellationToken cancellationToken)
    {
        var transportSettings = new ITransportSettings[]
        {
            new AmqpTransportSettings(TransportType.Amqp_Tcp_Only)
            {
                AmqpConnectionPoolSettings = new AmqpConnectionPoolSettings
                {
                    Pooling = true,
                    MaxPoolSize = 1
                }
            }
        };

        return _config.AuthMethod switch
        {
            "ConnectionString" => DeviceClient.CreateFromConnectionString(
                _config.ConnectionString, transportSettings),
            
            "X509" => await CreateX509ClientAsync(transportSettings, cancellationToken),
            
            "DPS" => await CreateDpsClientAsync(transportSettings, cancellationToken),
            
            _ => throw new InvalidOperationException($"Unknown auth method: {_config.AuthMethod}")
        };
    }

    private async Task<DeviceClient> CreateX509ClientAsync(
        ITransportSettings[] transportSettings,
        CancellationToken cancellationToken)
    {
        var certificate = new X509Certificate2(
            _config.CertificatePath, 
            _config.CertificatePassword);

        var auth = new DeviceAuthenticationWithX509Certificate(
            _config.DeviceId, certificate);

        return DeviceClient.Create(_config.IoTHubHostname, auth, transportSettings);
    }

    private async Task<DeviceClient> CreateDpsClientAsync(
        ITransportSettings[] transportSettings,
        CancellationToken cancellationToken)
    {
        var certificate = new X509Certificate2(
            _config.CertificatePath, 
            _config.CertificatePassword);

        using var security = new SecurityProviderX509Certificate(certificate);
        using var transport = new ProvisioningTransportHandlerAmqp();
        
        var provClient = ProvisioningDeviceClient.Create(
            _config.DPS.GlobalEndpoint,
            _config.DPS.IdScope,
            security,
            transport);

        var result = await provClient.RegisterAsync(cancellationToken);

        if (result.Status != ProvisioningRegistrationStatusType.Assigned)
        {
            throw new InvalidOperationException($"DPS registration failed: {result.Status}");
        }

        _logger.LogInformation("DPS registration successful. Assigned to hub: {Hub}", 
            result.AssignedHub);

        var auth = new DeviceAuthenticationWithX509Certificate(
            result.DeviceId, certificate);

        return DeviceClient.Create(result.AssignedHub, auth, transportSettings);
    }

    private async Task RegisterDefaultMethodHandlersAsync(CancellationToken cancellationToken)
    {
        // Register default direct methods
        await _deviceClient!.SetMethodHandlerAsync("SwitchMode", HandleSwitchMode, null, cancellationToken);
        await _deviceClient.SetMethodHandlerAsync("EmergencyStop", HandleEmergencyStop, null, cancellationToken);
        await _deviceClient.SetMethodHandlerAsync("RequestStatus", HandleRequestStatus, null, cancellationToken);
        await _deviceClient.SetMethodHandlerAsync("Ping", HandlePing, null, cancellationToken);
    }

    private Task<MethodResponse> HandleSwitchMode(MethodRequest request, object? userContext)
    {
        _logger.LogInformation("Received SwitchMode command: {Payload}", request.DataAsJson);
        
        _eventBus.Publish(new CommandReceivedEvent
        {
            EventId = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow,
            Source = nameof(AzureIoTHubService),
            CommandType = "SwitchMode",
            Payload = request.DataAsJson,
            CorrelationId = request.Name
        });

        return Task.FromResult(new MethodResponse(
            Encoding.UTF8.GetBytes("{\"status\":\"accepted\"}"), 200));
    }

    private Task<MethodResponse> HandleEmergencyStop(MethodRequest request, object? userContext)
    {
        _logger.LogCritical("Received EmergencyStop command!");
        
        _eventBus.Publish(new EmergencyEvent
        {
            EventId = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow,
            Source = nameof(AzureIoTHubService),
            Type = EmergencyType.OperatorAlert,
            Description = "Remote emergency stop triggered",
            RequiresImmediateAction = true
        });

        return Task.FromResult(new MethodResponse(
            Encoding.UTF8.GetBytes("{\"status\":\"stopped\"}"), 200));
    }

    private Task<MethodResponse> HandleRequestStatus(MethodRequest request, object? userContext)
    {
        _logger.LogDebug("Received RequestStatus command");
        
        var status = new
        {
            connected = _isConnected,
            timestamp = DateTimeOffset.UtcNow,
            uptime = TimeSpan.FromMilliseconds(Environment.TickCount64).ToString()
        };

        return Task.FromResult(new MethodResponse(
            Encoding.UTF8.GetBytes(JsonSerializer.Serialize(status)), 200));
    }

    private Task<MethodResponse> HandlePing(MethodRequest request, object? userContext)
    {
        return Task.FromResult(new MethodResponse(
            Encoding.UTF8.GetBytes("{\"pong\":true}"), 200));
    }

    private void OnConnectionStatusChanged(ConnectionStatus status, ConnectionStatusChangeReason reason)
    {
        _logger.LogInformation("IoT Hub connection status changed: {Status} ({Reason})", status, reason);
        
        var wasConnected = _isConnected;
        _isConnected = status == ConnectionStatus.Connected;

        if (wasConnected != _isConnected)
        {
            PublishConnectivityEvent(_isConnected);
        }
    }

    private void PublishConnectivityEvent(bool isConnected)
    {
        _eventBus.Publish(new ConnectivityChangedEvent
        {
            EventId = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow,
            Source = nameof(AzureIoTHubService),
            IsConnected = isConnected,
            ConnectionType = ConnectionType.WiFi // Would be determined dynamically
        });
    }

    public void Dispose()
    {
        _deviceClient?.Dispose();
        _connectionLock.Dispose();
    }
}

/// <summary>
/// Device Twin manager for maintaining device state synchronization.
/// </summary>
public sealed class DeviceTwinManager
{
    private readonly IAzureIoTHubService _iotHubService;
    private readonly ILogger<DeviceTwinManager> _logger;
    private TwinCollection? _lastDesiredProperties;
    private TwinCollection _reportedProperties = new();

    public DeviceTwinManager(
        IAzureIoTHubService iotHubService,
        ILogger<DeviceTwinManager> logger)
    {
        _iotHubService = iotHubService;
        _logger = logger;

        // Register for desired property updates
        _iotHubService.RegisterDesiredPropertyHandler(OnDesiredPropertiesUpdated);
    }

    /// <summary>
    /// Updates a reported property.
    /// </summary>
    public async Task UpdateReportedPropertyAsync(string key, object value, CancellationToken cancellationToken = default)
    {
        _reportedProperties[key] = value;
        await _iotHubService.UpdateReportedPropertiesAsync(_reportedProperties, cancellationToken);
    }

    /// <summary>
    /// Updates multiple reported properties.
    /// </summary>
    public async Task UpdateReportedPropertiesAsync(
        Dictionary<string, object> properties, 
        CancellationToken cancellationToken = default)
    {
        foreach (var kvp in properties)
        {
            _reportedProperties[kvp.Key] = kvp.Value;
        }
        await _iotHubService.UpdateReportedPropertiesAsync(_reportedProperties, cancellationToken);
    }

    /// <summary>
    /// Reports the current system state.
    /// </summary>
    public async Task ReportSystemStateAsync(SystemState state, CancellationToken cancellationToken = default)
    {
        var properties = new Dictionary<string, object>
        {
            ["operationMode"] = state.OperationMode.ToString(),
            ["movementState"] = state.MovementState.ToString(),
            ["currentGait"] = state.CurrentGait.ToString(),
            ["overallHealth"] = state.OverallHealth.ToString(),
            ["isConnected"] = state.IsConnected,
            ["uptime"] = state.Uptime.TotalSeconds,
            ["lastUpdate"] = DateTimeOffset.UtcNow.ToString("O")
        };

        if (state.SensorState.Position != null)
        {
            properties["position"] = new
            {
                latitude = state.SensorState.Position.Latitude,
                longitude = state.SensorState.Position.Longitude,
                altitude = state.SensorState.Position.Altitude
            };
        }

        if (state.SensorState.PowerStatus != null)
        {
            properties["batteryLevel"] = state.SensorState.PowerStatus.BatteryPercentage;
            properties["powerConsumption"] = state.SensorState.PowerStatus.PowerConsumption;
        }

        await UpdateReportedPropertiesAsync(properties, cancellationToken);
    }

    /// <summary>
    /// Gets a desired property value.
    /// </summary>
    public T? GetDesiredProperty<T>(string key, T? defaultValue = default)
    {
        if (_lastDesiredProperties?.Contains(key) == true)
        {
            try
            {
                return (T)_lastDesiredProperties[key];
            }
            catch
            {
                return defaultValue;
            }
        }
        return defaultValue;
    }

    private Task OnDesiredPropertiesUpdated(TwinCollection desiredProperties)
    {
        _logger.LogInformation("Received desired properties update: {Version}", 
            desiredProperties.Version);
        
        _lastDesiredProperties = desiredProperties;
        
        // Process specific desired properties
        if (desiredProperties.Contains("operationMode"))
        {
            object? modeValue = desiredProperties["operationMode"];
            var mode = modeValue?.ToString();
            _logger.LogInformation("Desired operation mode: {Mode}", mode);
        }

        return Task.CompletedTask;
    }
}
