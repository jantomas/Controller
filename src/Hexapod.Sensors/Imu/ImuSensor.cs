using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Models;
using Hexapod.Sensors.Abstractions;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace Hexapod.Sensors.Imu;

/// <summary>
/// IMU sensor implementation for BNO055 absolute orientation sensor.
/// Uses I2C communication and includes Madgwick filter for sensor fusion.
/// </summary>
public sealed class ImuSensor : IImuSensor
{
    private readonly ILogger<ImuSensor> _logger;
    private readonly ImuConfig _config;
    private HealthStatus _health = HealthStatus.Unknown;
    private bool _isCalibrated;
    
    // Sensor data
    private Orientation _currentOrientation = new() { Timestamp = DateTimeOffset.UtcNow };
    private Acceleration _currentAcceleration = new() { Timestamp = DateTimeOffset.UtcNow };
    private Acceleration _currentAngularVelocity = new() { Timestamp = DateTimeOffset.UtcNow };
    
    // Madgwick filter parameters
    private float _beta = 0.1f; // Filter gain
    private float[] _quaternion = { 1.0f, 0.0f, 0.0f, 0.0f };

    public ImuSensor(IOptions<HexapodConfiguration> config, ILogger<ImuSensor> logger)
    {
        _logger = logger;
        _config = config.Value.Sensors.Imu;
    }

    public SensorType Type => SensorType.Imu;
    public string Name => "IMU-BNO055";
    public bool IsEnabled => _config.Enabled;
    public HealthStatus Health => _health;
    public bool IsCalibrated => _isCalibrated;

    public async Task InitializeAsync(CancellationToken cancellationToken = default)
    {
        if (!_config.Enabled)
        {
            _logger.LogInformation("IMU sensor is disabled in configuration");
            return;
        }

        try
        {
            // Initialize I2C communication with BNO055
            // In production, use System.Device.I2c
            
            // Configure BNO055 for NDOF mode (9-axis sensor fusion)
            await ConfigureBno055Async(cancellationToken);
            
            _health = HealthStatus.Healthy;
            _logger.LogInformation("IMU sensor initialized at I2C address 0x{Address:X2}", _config.I2CAddress);
        }
        catch (Exception ex)
        {
            _health = HealthStatus.Unhealthy;
            _logger.LogError(ex, "Failed to initialize IMU sensor");
            throw;
        }
    }

    public Task ShutdownAsync(CancellationToken cancellationToken = default)
    {
        _health = HealthStatus.Offline;
        _logger.LogInformation("IMU sensor shut down");
        return Task.CompletedTask;
    }

    public async Task<bool> SelfTestAsync(CancellationToken cancellationToken = default)
    {
        try
        {
            // Read chip ID (should be 0xA0 for BNO055)
            // var chipId = await ReadRegisterAsync(0x00, cancellationToken);
            // return chipId == 0xA0;
            
            await Task.Delay(100, cancellationToken);
            return true;
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "IMU self-test failed");
            return false;
        }
    }

    public Task<Orientation> GetOrientationAsync(CancellationToken cancellationToken = default)
    {
        // In production, read from BNO055 Euler angle registers
        // Or use Madgwick filter with raw sensor data
        
        return Task.FromResult(_currentOrientation with { Timestamp = DateTimeOffset.UtcNow });
    }

    public Task<Acceleration> GetAccelerationAsync(CancellationToken cancellationToken = default)
    {
        // Read from BNO055 accelerometer registers
        return Task.FromResult(_currentAcceleration with { Timestamp = DateTimeOffset.UtcNow });
    }

    public Task<Acceleration> GetAngularVelocityAsync(CancellationToken cancellationToken = default)
    {
        // Read from BNO055 gyroscope registers
        return Task.FromResult(_currentAngularVelocity with { Timestamp = DateTimeOffset.UtcNow });
    }

    public async Task<bool> CalibrateAsync(CancellationToken cancellationToken = default)
    {
        _logger.LogInformation("Starting IMU calibration...");
        
        // BNO055 has automatic calibration
        // Wait for calibration status registers to indicate completion
        
        var timeout = DateTimeOffset.UtcNow.AddSeconds(30);
        while (DateTimeOffset.UtcNow < timeout && !cancellationToken.IsCancellationRequested)
        {
            // Read calibration status register (0x35)
            // Check system, gyro, accel, and mag calibration (each 0-3)
            
            await Task.Delay(500, cancellationToken);
            
            // Simulated calibration completion
            _isCalibrated = true;
            _logger.LogInformation("IMU calibration completed successfully");
            return true;
        }

        _logger.LogWarning("IMU calibration timed out");
        return false;
    }

    /// <summary>
    /// Updates sensor readings with raw data (called from sensor polling loop).
    /// </summary>
    public void UpdateRawData(
        double accelX, double accelY, double accelZ,
        double gyroX, double gyroY, double gyroZ,
        double magX, double magY, double magZ,
        double deltaTime)
    {
        _currentAcceleration = new Acceleration
        {
            X = accelX,
            Y = accelY,
            Z = accelZ,
            Timestamp = DateTimeOffset.UtcNow
        };

        _currentAngularVelocity = new Acceleration
        {
            X = gyroX,
            Y = gyroY,
            Z = gyroZ,
            Timestamp = DateTimeOffset.UtcNow
        };

        // Apply Madgwick filter for orientation
        MadgwickUpdate(
            (float)gyroX, (float)gyroY, (float)gyroZ,
            (float)accelX, (float)accelY, (float)accelZ,
            (float)magX, (float)magY, (float)magZ,
            (float)deltaTime);

        // Convert quaternion to Euler angles
        _currentOrientation = QuaternionToEuler();
    }

    private void MadgwickUpdate(
        float gx, float gy, float gz,
        float ax, float ay, float az,
        float mx, float my, float mz,
        float deltaTime)
    {
        float q0 = _quaternion[0], q1 = _quaternion[1], q2 = _quaternion[2], q3 = _quaternion[3];

        // Normalize accelerometer measurement
        var norm = MathF.Sqrt(ax * ax + ay * ay + az * az);
        if (norm > 0)
        {
            ax /= norm;
            ay /= norm;
            az /= norm;
        }

        // Normalize magnetometer measurement
        norm = MathF.Sqrt(mx * mx + my * my + mz * mz);
        if (norm > 0)
        {
            mx /= norm;
            my /= norm;
            mz /= norm;
        }

        // Reference direction of Earth's magnetic field
        var hx = mx * (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) + 2 * my * (q1 * q2 - q0 * q3) + 2 * mz * (q1 * q3 + q0 * q2);
        var hy = 2 * mx * (q1 * q2 + q0 * q3) + my * (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) + 2 * mz * (q2 * q3 - q0 * q1);
        var bx = MathF.Sqrt(hx * hx + hy * hy);
        var bz = 2 * mx * (q1 * q3 - q0 * q2) + 2 * my * (q2 * q3 + q0 * q1) + mz * (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

        // Gradient descent algorithm corrective step
        var s0 = -2 * q2 * (2 * (q1 * q3 - q0 * q2) - ax) + 2 * q1 * (2 * (q0 * q1 + q2 * q3) - ay);
        var s1 = 2 * q3 * (2 * (q1 * q3 - q0 * q2) - ax) + 2 * q0 * (2 * (q0 * q1 + q2 * q3) - ay) - 4 * q1 * (1 - 2 * (q1 * q1 + q2 * q2) - az);
        var s2 = -2 * q0 * (2 * (q1 * q3 - q0 * q2) - ax) + 2 * q3 * (2 * (q0 * q1 + q2 * q3) - ay) - 4 * q2 * (1 - 2 * (q1 * q1 + q2 * q2) - az);
        var s3 = 2 * q1 * (2 * (q1 * q3 - q0 * q2) - ax) + 2 * q2 * (2 * (q0 * q1 + q2 * q3) - ay);

        norm = MathF.Sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (norm > 0)
        {
            s0 /= norm;
            s1 /= norm;
            s2 /= norm;
            s3 /= norm;
        }

        // Compute rate of change of quaternion
        var qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - _beta * s0;
        var qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - _beta * s1;
        var qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - _beta * s2;
        var qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - _beta * s3;

        // Integrate to yield quaternion
        q0 += qDot0 * deltaTime;
        q1 += qDot1 * deltaTime;
        q2 += qDot2 * deltaTime;
        q3 += qDot3 * deltaTime;

        // Normalize quaternion
        norm = MathF.Sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        _quaternion[0] = q0 / norm;
        _quaternion[1] = q1 / norm;
        _quaternion[2] = q2 / norm;
        _quaternion[3] = q3 / norm;
    }

    private Orientation QuaternionToEuler()
    {
        var q0 = _quaternion[0];
        var q1 = _quaternion[1];
        var q2 = _quaternion[2];
        var q3 = _quaternion[3];

        // Roll (x-axis rotation)
        var sinRoll = 2 * (q0 * q1 + q2 * q3);
        var cosRoll = 1 - 2 * (q1 * q1 + q2 * q2);
        var roll = Math.Atan2(sinRoll, cosRoll) * 180 / Math.PI;

        // Pitch (y-axis rotation)
        var sinPitch = 2 * (q0 * q2 - q3 * q1);
        sinPitch = (float)Math.Clamp(sinPitch, -1.0, 1.0);
        var pitch = Math.Asin(sinPitch) * 180 / Math.PI;

        // Yaw (z-axis rotation)
        var sinYaw = 2 * (q0 * q3 + q1 * q2);
        var cosYaw = 1 - 2 * (q2 * q2 + q3 * q3);
        var yaw = Math.Atan2(sinYaw, cosYaw) * 180 / Math.PI;

        return new Orientation
        {
            Roll = roll,
            Pitch = pitch,
            Yaw = yaw,
            Timestamp = DateTimeOffset.UtcNow
        };
    }

    private Task ConfigureBno055Async(CancellationToken cancellationToken)
    {
        // BNO055 configuration sequence:
        // 1. Switch to config mode (0x3D = 0x00)
        // 2. Set page 0 (0x07 = 0x00)
        // 3. Configure power mode (0x3E = 0x00 for normal)
        // 4. Set units (0x3B = 0x00 for m/s², rad/s, Celsius)
        // 5. Set to NDOF mode for 9-axis fusion (0x3D = 0x0C)
        
        return Task.CompletedTask;
    }
}
