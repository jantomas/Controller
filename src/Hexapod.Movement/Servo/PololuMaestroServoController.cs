using System.IO.Ports;
using Hexapod.Core.Configuration;
using Hexapod.Movement.Services;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Movement.Servo;

/// <summary>
/// Pololu Maestro 18-Channel USB Servo Controller implementation.
/// Communicates via serial interface using the Pololu protocol.
/// </summary>
/// <remarks>
/// The Maestro supports multiple serial modes:
/// - USB Dual Port: Virtual COM port for commands
/// - USB Chained: Multiple Maestros on one bus
/// - UART: Direct serial connection
/// 
/// This implementation uses the Compact Protocol for efficiency.
/// Commands use device number 12 (0x0C) by default.
/// </remarks>
public sealed class PololuMaestroServoController : IServoController, IDisposable
{
    private readonly ILogger<PololuMaestroServoController> _logger;
    private readonly MaestroServoConfiguration _config;
    private readonly SerialPort? _serialPort;
    private readonly object _lock = new();
    private bool _enabled = true;
    private bool _disposed;

    // Pololu Protocol Commands (Compact Protocol)
    private const byte CommandSetTarget = 0x84;           // Set servo target position
    private const byte CommandSetSpeed = 0x87;            // Set servo speed limit
    private const byte CommandSetAcceleration = 0x89;     // Set servo acceleration
    private const byte CommandGetPosition = 0x90;         // Get current position
    private const byte CommandGetMovingState = 0x93;      // Check if servos are moving
    private const byte CommandGetErrors = 0xA1;           // Get error flags
    private const byte CommandGoHome = 0xA2;              // Return all servos to home

    // Mini SSC Protocol (alternative)
    private const byte MiniSscCommand = 0xFF;

    /// <summary>
    /// Servo channel mapping: LegId * 3 + JointIndex -> Maestro Channel
    /// Leg 0: Channels 0, 1, 2 (Coxa, Femur, Tibia)
    /// Leg 1: Channels 3, 4, 5
    /// Leg 2: Channels 6, 7, 8
    /// Leg 3: Channels 9, 10, 11
    /// Leg 4: Channels 12, 13, 14
    /// Leg 5: Channels 15, 16, 17
    /// </summary>
    private readonly int[] _channelMap;

    /// <summary>
    /// Per-servo calibration: min/max PWM values and center offset
    /// </summary>
    private readonly ServoCalibration[] _calibrations;

    public PololuMaestroServoController(
        IOptions<HexapodConfiguration> config,
        ILogger<PololuMaestroServoController> logger)
    {
        _logger = logger;
        _config = config.Value.Hardware.MaestroServo;

        // Initialize channel mapping (default: direct 1:1)
        _channelMap = _config.ChannelMapping ?? Enumerable.Range(0, 18).ToArray();

        // Initialize calibrations with defaults or from config
        _calibrations = InitializeCalibrations();

        if (!_config.Enabled)
        {
            _logger.LogWarning("Pololu Maestro servo controller is disabled in configuration");
            return;
        }

        try
        {
            _serialPort = new SerialPort
            {
                PortName = _config.SerialPort,
                BaudRate = _config.BaudRate,
                DataBits = 8,
                Parity = Parity.None,
                StopBits = StopBits.One,
                ReadTimeout = _config.ReadTimeoutMs,
                WriteTimeout = _config.WriteTimeoutMs,
                Handshake = Handshake.None
            };

            _serialPort.Open();
            _logger.LogInformation(
                "Pololu Maestro initialized on {Port} at {Baud} baud, Device #{DeviceNumber}",
                _config.SerialPort,
                _config.BaudRate,
                _config.DeviceNumber);

            // Clear any startup errors
            ClearErrors();

            // Set initial speed and acceleration limits
            InitializeServoLimits();
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Failed to initialize Pololu Maestro on {Port}", _config.SerialPort);
            _serialPort?.Dispose();
            _serialPort = null;
        }
    }

    /// <inheritdoc/>
    public async Task SetPositionsAsync(IReadOnlyList<LegJointState> jointStates, CancellationToken cancellationToken = default)
    {
        if (!_enabled || _serialPort == null || !_serialPort.IsOpen)
            return;

        try
        {
            // Build command buffer for all servos (more efficient than individual commands)
            var commands = new List<byte>();

            foreach (var state in jointStates)
            {
                // Each leg has 3 joints: Coxa, Femur, Tibia
                AddServoCommand(commands, state.LegId, 0, state.CoxaAngle);
                AddServoCommand(commands, state.LegId, 1, state.FemurAngle);
                AddServoCommand(commands, state.LegId, 2, state.TibiaAngle);
            }

            if (commands.Count > 0)
            {
                await WriteCommandsAsync(commands.ToArray(), cancellationToken);
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error setting servo positions");
        }
    }

    /// <inheritdoc/>
    public void DisableAll()
    {
        lock (_lock)
        {
            _enabled = false;
        }

        if (_serialPort?.IsOpen == true)
        {
            try
            {
                // Set all channels to 0 (disabled)
                for (int channel = 0; channel < 18; channel++)
                {
                    SetTargetRaw(channel, 0);
                }
                _logger.LogWarning("All Maestro servos disabled");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error disabling servos");
            }
        }
    }

    /// <inheritdoc/>
    public void EnableAll()
    {
        lock (_lock)
        {
            _enabled = true;
        }
        _logger.LogInformation("All Maestro servos enabled");
    }

    /// <summary>
    /// Sends all servos to their home (center) positions.
    /// </summary>
    public void GoHome()
    {
        if (_serialPort?.IsOpen != true)
            return;

        try
        {
            var command = new byte[] { CommandGoHome };
            WriteCommands(command);
            _logger.LogInformation("Servos returning to home positions");
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error sending servos home");
        }
    }

    /// <summary>
    /// Gets the current position of a servo channel.
    /// </summary>
    /// <param name="channel">Servo channel (0-17)</param>
    /// <returns>Position in quarter-microseconds, or -1 on error</returns>
    public int GetPosition(int channel)
    {
        if (_serialPort?.IsOpen != true || channel < 0 || channel > 17)
            return -1;

        lock (_lock)
        {
            try
            {
                var command = new byte[] { CommandGetPosition, (byte)channel };
                _serialPort.Write(command, 0, command.Length);

                // Response is 2 bytes (low byte first)
                var lowByte = _serialPort.ReadByte();
                var highByte = _serialPort.ReadByte();

                return lowByte + (highByte << 8);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error getting position for channel {Channel}", channel);
                return -1;
            }
        }
    }

    /// <summary>
    /// Checks if any servos are still moving to their targets.
    /// </summary>
    /// <returns>True if any servos are moving</returns>
    public bool IsMoving()
    {
        if (_serialPort?.IsOpen != true)
            return false;

        lock (_lock)
        {
            try
            {
                var command = new byte[] { CommandGetMovingState };
                _serialPort.Write(command, 0, command.Length);

                var response = _serialPort.ReadByte();
                return response != 0;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error checking moving state");
                return false;
            }
        }
    }

    /// <summary>
    /// Sets the speed limit for a servo channel.
    /// </summary>
    /// <param name="channel">Servo channel (0-17)</param>
    /// <param name="speed">Speed limit (0 = unlimited, 1-255)</param>
    public void SetSpeed(int channel, int speed)
    {
        if (_serialPort?.IsOpen != true || channel < 0 || channel > 17)
            return;

        var speedValue = Math.Clamp(speed, 0, 255);
        var command = new byte[]
        {
            CommandSetSpeed,
            (byte)channel,
            (byte)(speedValue & 0x7F),
            (byte)((speedValue >> 7) & 0x7F)
        };

        WriteCommands(command);
    }

    /// <summary>
    /// Sets the acceleration limit for a servo channel.
    /// </summary>
    /// <param name="channel">Servo channel (0-17)</param>
    /// <param name="acceleration">Acceleration limit (0 = unlimited, 1-255)</param>
    public void SetAcceleration(int channel, int acceleration)
    {
        if (_serialPort?.IsOpen != true || channel < 0 || channel > 17)
            return;

        var accelValue = Math.Clamp(acceleration, 0, 255);
        var command = new byte[]
        {
            CommandSetAcceleration,
            (byte)channel,
            (byte)(accelValue & 0x7F),
            (byte)((accelValue >> 7) & 0x7F)
        };

        WriteCommands(command);
    }

    /// <summary>
    /// Sets the position of a servo channel directly in PWM microseconds.
    /// Used for testing and calibration.
    /// </summary>
    /// <param name="channel">Servo channel (0-17)</param>
    /// <param name="pwmMicroseconds">Target position in microseconds (typically 500-2500)</param>
    public void SetChannelPosition(int channel, double pwmMicroseconds)
    {
        if (_serialPort?.IsOpen != true || channel < 0 || channel > 17)
            return;

        // Convert to quarter-microseconds
        var targetQus = (int)(pwmMicroseconds * 4);
        targetQus = Math.Clamp(targetQus, 2000, 10000); // 500-2500µs range

        var command = new byte[]
        {
            CommandSetTarget,
            (byte)channel,
            (byte)(targetQus & 0x7F),
            (byte)((targetQus >> 7) & 0x7F)
        };

        WriteCommands(command);
    }

    /// <summary>
    /// Gets and clears error flags from the Maestro.
    /// </summary>
    /// <returns>Error flags bitmask</returns>
    public MaestroErrors GetAndClearErrors()
    {
        if (_serialPort?.IsOpen != true)
            return MaestroErrors.None;

        lock (_lock)
        {
            try
            {
                var command = new byte[] { CommandGetErrors };
                _serialPort.Write(command, 0, command.Length);

                var lowByte = _serialPort.ReadByte();
                var highByte = _serialPort.ReadByte();

                var errors = (MaestroErrors)(lowByte + (highByte << 8));

                if (errors != MaestroErrors.None)
                {
                    _logger.LogWarning("Maestro errors: {Errors}", errors);
                }

                return errors;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error getting Maestro error flags");
                return MaestroErrors.None;
            }
        }
    }

    private void AddServoCommand(List<byte> commands, int legId, int jointIndex, double angleRadians)
    {
        var channel = _channelMap[legId * 3 + jointIndex];
        var calibration = _calibrations[channel];

        // Convert angle to PWM microseconds
        var pwmUs = AngleToPwmMicroseconds(angleRadians, calibration);

        // Maestro uses quarter-microseconds (multiply by 4)
        var target = (int)(pwmUs * 4);

        // Clamp to calibration limits
        target = Math.Clamp(target, calibration.MinPwmQus, calibration.MaxPwmQus);

        // Compact Protocol: 0x84, channel, target_low, target_high
        commands.Add(CommandSetTarget);
        commands.Add((byte)channel);
        commands.Add((byte)(target & 0x7F));         // Low 7 bits
        commands.Add((byte)((target >> 7) & 0x7F));  // High 7 bits
    }

    private void SetTargetRaw(int channel, int targetQus)
    {
        var command = new byte[]
        {
            CommandSetTarget,
            (byte)channel,
            (byte)(targetQus & 0x7F),
            (byte)((targetQus >> 7) & 0x7F)
        };

        WriteCommands(command);
    }

    private double AngleToPwmMicroseconds(double angleRadians, ServoCalibration calibration)
    {
        // Convert radians to degrees
        var angleDegrees = angleRadians * 180.0 / Math.PI;

        // Apply center offset
        angleDegrees += calibration.CenterOffsetDegrees;

        // Normalize to servo range (-90 to +90 degrees typically)
        // Map to PWM range (typically 500-2500 microseconds)
        var normalized = (angleDegrees + 90.0) / 180.0;
        normalized = Math.Clamp(normalized, 0.0, 1.0);

        return calibration.MinPwmUs + normalized * (calibration.MaxPwmUs - calibration.MinPwmUs);
    }

    private async Task WriteCommandsAsync(byte[] commands, CancellationToken cancellationToken)
    {
        if (_serialPort == null)
            return;

        await Task.Run(() =>
        {
            lock (_lock)
            {
                try
                {
                    _serialPort.Write(commands, 0, commands.Length);
                }
                catch (Exception ex)
                {
                    _logger.LogError(ex, "Error writing to Maestro serial port");
                }
            }
        }, cancellationToken);
    }

    private void WriteCommands(byte[] commands)
    {
        if (_serialPort == null)
            return;

        lock (_lock)
        {
            try
            {
                _serialPort.Write(commands, 0, commands.Length);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error writing to Maestro serial port");
            }
        }
    }

    private void ClearErrors()
    {
        GetAndClearErrors();
    }

    private void InitializeServoLimits()
    {
        for (int channel = 0; channel < 18; channel++)
        {
            var calibration = _calibrations[channel];
            SetSpeed(channel, calibration.SpeedLimit);
            SetAcceleration(channel, calibration.AccelerationLimit);
        }

        _logger.LogDebug("Servo speed and acceleration limits initialized");
    }

    private ServoCalibration[] InitializeCalibrations()
    {
        var calibrations = new ServoCalibration[18];

        for (int i = 0; i < 18; i++)
        {
            // Check if config has calibration for this channel
            if (_config.ServoCalibrations != null &&
                _config.ServoCalibrations.TryGetValue(i, out var configCal))
            {
                calibrations[i] = new ServoCalibration
                {
                    MinPwmUs = configCal.MinPwmUs,
                    MaxPwmUs = configCal.MaxPwmUs,
                    CenterOffsetDegrees = configCal.CenterOffsetDegrees,
                    SpeedLimit = configCal.SpeedLimit,
                    AccelerationLimit = configCal.AccelerationLimit
                };
            }
            else
            {
                // Use defaults
                calibrations[i] = new ServoCalibration
                {
                    MinPwmUs = _config.DefaultMinPwmUs,
                    MaxPwmUs = _config.DefaultMaxPwmUs,
                    CenterOffsetDegrees = 0,
                    SpeedLimit = _config.DefaultSpeedLimit,
                    AccelerationLimit = _config.DefaultAccelerationLimit
                };
            }

            // Pre-calculate quarter-microsecond limits
            calibrations[i].MinPwmQus = (int)(calibrations[i].MinPwmUs * 4);
            calibrations[i].MaxPwmQus = (int)(calibrations[i].MaxPwmUs * 4);
        }

        return calibrations;
    }

    public void Dispose()
    {
        if (_disposed)
            return;

        _disposed = true;

        if (_serialPort?.IsOpen == true)
        {
            try
            {
                // Optionally disable servos on shutdown
                if (_config.DisableOnShutdown)
                {
                    DisableAll();
                }

                _serialPort.Close();
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error closing Maestro serial port");
            }
        }

        _serialPort?.Dispose();
        _logger.LogInformation("Pololu Maestro servo controller disposed");
    }
}

/// <summary>
/// Internal calibration settings for an individual servo.
/// </summary>
internal class ServoCalibration
{
    public double MinPwmUs { get; set; } = 500;
    public double MaxPwmUs { get; set; } = 2500;
    public double CenterOffsetDegrees { get; set; } = 0;
    public int SpeedLimit { get; set; } = 0;
    public int AccelerationLimit { get; set; } = 0;
    public int MinPwmQus { get; set; }
    public int MaxPwmQus { get; set; }
}

/// <summary>
/// Maestro error flags.
/// </summary>
[Flags]
public enum MaestroErrors : ushort
{
    None = 0,
    SerialSignal = 1 << 0,
    SerialOverrun = 1 << 1,
    SerialBufferFull = 1 << 2,
    SerialCrc = 1 << 3,
    SerialProtocol = 1 << 4,
    SerialTimeout = 1 << 5,
    ScriptStack = 1 << 6,
    ScriptCallStack = 1 << 7,
    ScriptProgramCounter = 1 << 8
}
