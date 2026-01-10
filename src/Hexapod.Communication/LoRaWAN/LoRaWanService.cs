using System.IO.Ports;
using System.Text;
using Hexapod.Core.Configuration;
using Hexapod.Core.Events;
using Hexapod.Core.Services;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Communication.LoRaWAN;

/// <summary>
/// Interface for LoRaWAN communication service.
/// </summary>
public interface ILoRaWanService
{
    /// <summary>
    /// Gets whether the device has joined the LoRaWAN network.
    /// </summary>
    bool IsJoined { get; }

    /// <summary>
    /// Gets the current signal strength (RSSI).
    /// </summary>
    int SignalStrength { get; }

    /// <summary>
    /// Initializes and joins the LoRaWAN network.
    /// </summary>
    Task JoinNetworkAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Sends data over LoRaWAN.
    /// </summary>
    Task<bool> SendAsync(byte[] data, int port = 1, bool confirmed = false, CancellationToken cancellationToken = default);

    /// <summary>
    /// Sends a string message over LoRaWAN.
    /// </summary>
    Task<bool> SendAsync(string message, int port = 1, bool confirmed = false, CancellationToken cancellationToken = default);

    /// <summary>
    /// Receives data from LoRaWAN (downlink).
    /// </summary>
    Task<LoRaWanMessage?> ReceiveAsync(CancellationToken cancellationToken = default);
}

/// <summary>
/// Represents a LoRaWAN message.
/// </summary>
public record LoRaWanMessage
{
    public required byte[] Data { get; init; }
    public required int Port { get; init; }
    public required int Rssi { get; init; }
    public required int Snr { get; init; }
    public required DateTimeOffset Timestamp { get; init; }
}

/// <summary>
/// LoRaWAN communication service using AT command interface (e.g., SX1262 module).
/// Optimized for low-bandwidth, long-range communication.
/// </summary>
public sealed class LoRaWanService : ILoRaWanService, IDisposable
{
    private readonly ILogger<LoRaWanService> _logger;
    private readonly IEventBus _eventBus;
    private readonly LoRaWanConfig _config;
    private SerialPort? _serialPort;
    private bool _isJoined;
    private int _signalStrength;
    private readonly Queue<LoRaWanMessage> _receiveQueue = new();
    private readonly SemaphoreSlim _sendLock = new(1, 1);
    private CancellationTokenSource? _receiveCts;
    private Task? _receiveTask;

    public LoRaWanService(
        IEventBus eventBus,
        IOptions<HexapodConfiguration> config,
        ILogger<LoRaWanService> logger)
    {
        _eventBus = eventBus;
        _config = config.Value.Communication.LoRaWan;
        _logger = logger;
    }

    public bool IsJoined => _isJoined;
    public int SignalStrength => _signalStrength;

    public async Task JoinNetworkAsync(CancellationToken cancellationToken = default)
    {
        if (!_config.Enabled)
        {
            _logger.LogInformation("LoRaWAN is disabled in configuration");
            return;
        }

        try
        {
            // Initialize serial port for AT commands
            _serialPort = new SerialPort("/dev/ttyUSB0", 115200)
            {
                ReadTimeout = 5000,
                WriteTimeout = 1000,
                NewLine = "\r\n"
            };

            _serialPort.Open();
            _logger.LogInformation("LoRaWAN serial port opened");

            // Reset module
            await SendAtCommandAsync("AT+RESET", cancellationToken);
            await Task.Delay(2000, cancellationToken);

            // Configure for OTAA join
            await SendAtCommandAsync($"AT+DEVEUI={_config.DeviceEUI}", cancellationToken);
            await SendAtCommandAsync($"AT+APPEUI={_config.ApplicationEUI}", cancellationToken);
            await SendAtCommandAsync($"AT+APPKEY={_config.ApplicationKey}", cancellationToken);

            // Set region
            await SendAtCommandAsync($"AT+REGION={_config.Region}", cancellationToken);

            // Set data rate and power
            await SendAtCommandAsync($"AT+DR={_config.SpreadingFactor}", cancellationToken);
            await SendAtCommandAsync($"AT+TXP={_config.TxPower}", cancellationToken);

            // Enable ADR if configured
            await SendAtCommandAsync($"AT+ADR={((_config.AdaptiveDataRate) ? "ON" : "OFF")}", cancellationToken);

            // Attempt to join network
            _logger.LogInformation("Attempting to join LoRaWAN network (OTAA)...");
            var response = await SendAtCommandAsync("AT+JOIN", cancellationToken, timeout: 30000);

            if (response.Contains("OK") || response.Contains("JOINED"))
            {
                _isJoined = true;
                _logger.LogInformation("Successfully joined LoRaWAN network");
                
                // Start receive loop
                _receiveCts = new CancellationTokenSource();
                _receiveTask = Task.Run(() => ReceiveLoopAsync(_receiveCts.Token), cancellationToken);

                PublishConnectivityEvent(true);
            }
            else
            {
                throw new InvalidOperationException($"Failed to join network: {response}");
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Failed to join LoRaWAN network");
            _isJoined = false;
            PublishConnectivityEvent(false);
            throw;
        }
    }

    public async Task<bool> SendAsync(byte[] data, int port = 1, bool confirmed = false, CancellationToken cancellationToken = default)
    {
        if (!_isJoined || _serialPort == null)
        {
            _logger.LogWarning("Cannot send - not joined to LoRaWAN network");
            return false;
        }

        await _sendLock.WaitAsync(cancellationToken);
        try
        {
            // Convert data to hex string
            var hexData = BitConverter.ToString(data).Replace("-", "");
            
            // Send using AT command
            var command = confirmed 
                ? $"AT+SENDB={port}:{hexData}" 
                : $"AT+SEND={port}:{hexData}";

            var response = await SendAtCommandAsync(command, cancellationToken);
            
            if (response.Contains("OK") || response.Contains("DONE"))
            {
                _logger.LogDebug("LoRaWAN sent {Bytes} bytes on port {Port}", data.Length, port);
                return true;
            }

            _logger.LogWarning("LoRaWAN send failed: {Response}", response);
            return false;
        }
        finally
        {
            _sendLock.Release();
        }
    }

    public Task<bool> SendAsync(string message, int port = 1, bool confirmed = false, CancellationToken cancellationToken = default)
    {
        return SendAsync(Encoding.UTF8.GetBytes(message), port, confirmed, cancellationToken);
    }

    public Task<LoRaWanMessage?> ReceiveAsync(CancellationToken cancellationToken = default)
    {
        lock (_receiveQueue)
        {
            if (_receiveQueue.Count > 0)
            {
                return Task.FromResult<LoRaWanMessage?>(_receiveQueue.Dequeue());
            }
        }

        return Task.FromResult<LoRaWanMessage?>(null);
    }

    private async Task<string> SendAtCommandAsync(string command, CancellationToken cancellationToken, int timeout = 5000)
    {
        if (_serialPort == null || !_serialPort.IsOpen)
        {
            throw new InvalidOperationException("Serial port not open");
        }

        // Clear any pending data
        _serialPort.DiscardInBuffer();
        
        // Send command
        _serialPort.WriteLine(command);
        _logger.LogDebug("AT Command: {Command}", command);

        // Wait for response
        var response = new StringBuilder();
        var startTime = DateTime.UtcNow;

        while ((DateTime.UtcNow - startTime).TotalMilliseconds < timeout)
        {
            if (cancellationToken.IsCancellationRequested)
                break;

            try
            {
                if (_serialPort.BytesToRead > 0)
                {
                    var line = _serialPort.ReadLine();
                    response.AppendLine(line);

                    // Check for terminal responses
                    if (line.Contains("OK") || line.Contains("ERROR") || 
                        line.Contains("JOINED") || line.Contains("DONE"))
                    {
                        break;
                    }
                }
                else
                {
                    await Task.Delay(100, cancellationToken);
                }
            }
            catch (TimeoutException)
            {
                // Continue waiting
            }
        }

        var result = response.ToString().Trim();
        _logger.LogDebug("AT Response: {Response}", result);
        return result;
    }

    private async Task ReceiveLoopAsync(CancellationToken cancellationToken)
    {
        while (!cancellationToken.IsCancellationRequested && _serialPort?.IsOpen == true)
        {
            try
            {
                if (_serialPort.BytesToRead > 0)
                {
                    var line = _serialPort.ReadLine();
                    ProcessReceivedLine(line);
                }
                else
                {
                    await Task.Delay(100, cancellationToken);
                }
            }
            catch (TimeoutException)
            {
                // Normal timeout
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error in LoRaWAN receive loop");
                await Task.Delay(1000, cancellationToken);
            }
        }
    }

    private void ProcessReceivedLine(string line)
    {
        // Parse downlink messages
        // Format varies by module, typical: +RECV=<port>,<rssi>,<snr>:<hex_data>
        if (line.StartsWith("+RECV") || line.StartsWith("+RX"))
        {
            try
            {
                var parts = line.Split(':', 2);
                if (parts.Length == 2)
                {
                    var metadata = parts[0].Split(',');
                    var hexData = parts[1].Trim();

                    var port = int.Parse(metadata[0].Split('=')[1]);
                    var rssi = metadata.Length > 1 ? int.Parse(metadata[1]) : 0;
                    var snr = metadata.Length > 2 ? int.Parse(metadata[2]) : 0;

                    // Convert hex to bytes
                    var data = new byte[hexData.Length / 2];
                    for (int i = 0; i < data.Length; i++)
                    {
                        data[i] = Convert.ToByte(hexData.Substring(i * 2, 2), 16);
                    }

                    var message = new LoRaWanMessage
                    {
                        Data = data,
                        Port = port,
                        Rssi = rssi,
                        Snr = snr,
                        Timestamp = DateTimeOffset.UtcNow
                    };

                    lock (_receiveQueue)
                    {
                        _receiveQueue.Enqueue(message);
                    }

                    _signalStrength = rssi;
                    _logger.LogDebug("LoRaWAN received {Bytes} bytes on port {Port}, RSSI={Rssi}", 
                        data.Length, port, rssi);

                    // Publish command event if it's a control message
                    if (port == 10) // Control port
                    {
                        _eventBus.Publish(new CommandReceivedEvent
                        {
                            EventId = Guid.NewGuid().ToString(),
                            Timestamp = DateTimeOffset.UtcNow,
                            Source = nameof(LoRaWanService),
                            CommandType = "LoRaWAN",
                            Payload = Encoding.UTF8.GetString(data),
                            CorrelationId = Guid.NewGuid().ToString()
                        });
                    }
                }
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to parse LoRaWAN message: {Line}", line);
            }
        }
        else if (line.Contains("RSSI"))
        {
            // Parse RSSI updates
            var match = System.Text.RegularExpressions.Regex.Match(line, @"RSSI[=:]?\s*(-?\d+)");
            if (match.Success)
            {
                _signalStrength = int.Parse(match.Groups[1].Value);
            }
        }
    }

    private void PublishConnectivityEvent(bool isConnected)
    {
        _eventBus.Publish(new ConnectivityChangedEvent
        {
            EventId = Guid.NewGuid().ToString(),
            Timestamp = DateTimeOffset.UtcNow,
            Source = nameof(LoRaWanService),
            IsConnected = isConnected,
            ConnectionType = ConnectionType.LoRaWAN,
            SignalStrength = _signalStrength
        });
    }

    public void Dispose()
    {
        _receiveCts?.Cancel();
        _serialPort?.Close();
        _serialPort?.Dispose();
        _sendLock.Dispose();
        _receiveCts?.Dispose();
    }
}

/// <summary>
/// Message encoder for efficient LoRaWAN payload formatting.
/// Optimizes data for limited bandwidth.
/// </summary>
public static class LoRaWanMessageEncoder
{
    /// <summary>
    /// Encodes a position message (10 bytes).
    /// </summary>
    public static byte[] EncodePosition(double latitude, double longitude, double altitude)
    {
        var buffer = new byte[10];
        
        // Latitude: 4 bytes (scaled integer, 0.0000001 degree resolution)
        var latInt = (int)(latitude * 10000000);
        Buffer.BlockCopy(BitConverter.GetBytes(latInt), 0, buffer, 0, 4);
        
        // Longitude: 4 bytes
        var lonInt = (int)(longitude * 10000000);
        Buffer.BlockCopy(BitConverter.GetBytes(lonInt), 0, buffer, 4, 4);
        
        // Altitude: 2 bytes (signed, 0.1m resolution)
        var altInt = (short)(altitude * 10);
        Buffer.BlockCopy(BitConverter.GetBytes(altInt), 0, buffer, 8, 2);

        return buffer;
    }

    /// <summary>
    /// Encodes a status message (8 bytes).
    /// </summary>
    public static byte[] EncodeStatus(
        byte operationMode,
        byte movementState,
        byte healthStatus,
        byte batteryPercent,
        ushort uptime)
    {
        return new byte[]
        {
            operationMode,
            movementState,
            healthStatus,
            batteryPercent,
            (byte)(uptime & 0xFF),
            (byte)((uptime >> 8) & 0xFF),
            0, // Reserved
            0  // Reserved
        };
    }

    /// <summary>
    /// Encodes an alert message (4 bytes).
    /// </summary>
    public static byte[] EncodeAlert(byte alertType, byte severity, ushort data)
    {
        return new byte[]
        {
            alertType,
            severity,
            (byte)(data & 0xFF),
            (byte)((data >> 8) & 0xFF)
        };
    }

    /// <summary>
    /// Decodes a command message.
    /// </summary>
    public static (byte CommandType, byte[] Parameters) DecodeCommand(byte[] data)
    {
        if (data.Length < 1)
            return (0, Array.Empty<byte>());

        var commandType = data[0];
        var parameters = new byte[data.Length - 1];
        if (parameters.Length > 0)
        {
            Array.Copy(data, 1, parameters, 0, parameters.Length);
        }

        return (commandType, parameters);
    }
}
