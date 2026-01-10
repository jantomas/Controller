namespace Hexapod.Core.Configuration;

/// <summary>
/// Main configuration for the hexapod controller.
/// </summary>
public class HexapodConfiguration
{
    public const string SectionName = "Hexapod";

    /// <summary>
    /// Unique device identifier.
    /// </summary>
    public string DeviceId { get; set; } = string.Empty;

    /// <summary>
    /// Device name for display purposes.
    /// </summary>
    public string DeviceName { get; set; } = "Hexapod-01";

    /// <summary>
    /// Hardware configuration settings.
    /// </summary>
    public HardwareConfiguration Hardware { get; set; } = new();

    /// <summary>
    /// Movement system configuration.
    /// </summary>
    public MovementConfiguration Movement { get; set; } = new();

    /// <summary>
    /// Sensor configuration settings.
    /// </summary>
    public SensorConfiguration Sensors { get; set; } = new();

    /// <summary>
    /// Vision and AI configuration.
    /// </summary>
    public VisionConfiguration Vision { get; set; } = new();

    /// <summary>
    /// Autonomy system configuration.
    /// </summary>
    public AutonomyConfiguration Autonomy { get; set; } = new();

    /// <summary>
    /// Communication configuration.
    /// </summary>
    public CommunicationConfiguration Communication { get; set; } = new();

    /// <summary>
    /// Telemetry and logging configuration.
    /// </summary>
    public TelemetryConfiguration Telemetry { get; set; } = new();

    /// <summary>
    /// Power management configuration.
    /// </summary>
    public PowerConfiguration Power { get; set; } = new();

    /// <summary>
    /// Mock mode configuration for development and testing.
    /// </summary>
    public MockModeConfiguration MockMode { get; set; } = new();
}

public class HardwareConfiguration
{
    /// <summary>
    /// Number of legs (typically 6 for hexapod).
    /// </summary>
    public int LegCount { get; set; } = 6;

    /// <summary>
    /// Number of joints per leg (typically 3: coxa, femur, tibia).
    /// </summary>
    public int JointsPerLeg { get; set; } = 3;

    /// <summary>
    /// Servo controller I2C address (for PCA9685).
    /// </summary>
    public int ServoControllerAddress { get; set; } = 0x40;

    /// <summary>
    /// Maximum servo update rate in Hz.
    /// </summary>
    public int ServoUpdateRate { get; set; } = 50;

    /// <summary>
    /// Servo controller type selection.
    /// </summary>
    public ServoControllerType ServoControllerType { get; set; } = ServoControllerType.PololuMaestro;

    /// <summary>
    /// Pololu Maestro servo controller configuration.
    /// </summary>
    public MaestroServoConfiguration MaestroServo { get; set; } = new();
}

/// <summary>
/// Available servo controller types.
/// </summary>
public enum ServoControllerType
{
    /// <summary>
    /// PCA9685 I2C PWM controller.
    /// </summary>
    Pca9685,

    /// <summary>
    /// Pololu Maestro USB servo controller.
    /// </summary>
    PololuMaestro
}

/// <summary>
/// Configuration for Pololu Maestro 18-Channel USB Servo Controller.
/// </summary>
public class MaestroServoConfiguration
{
    /// <summary>
    /// Enable/disable the Maestro controller.
    /// </summary>
    public bool Enabled { get; set; } = true;

    /// <summary>
    /// Serial port for communication (e.g., "/dev/ttyACM0" on Linux, "COM3" on Windows).
    /// </summary>
    public string SerialPort { get; set; } = "/dev/ttyACM0";

    /// <summary>
    /// Baud rate for serial communication.
    /// Maestro supports various rates; 115200 is recommended for best performance.
    /// </summary>
    public int BaudRate { get; set; } = 115200;

    /// <summary>
    /// Device number for chained Maestros (0-127, default 12).
    /// </summary>
    public int DeviceNumber { get; set; } = 12;

    /// <summary>
    /// Serial read timeout in milliseconds.
    /// </summary>
    public int ReadTimeoutMs { get; set; } = 100;

    /// <summary>
    /// Serial write timeout in milliseconds.
    /// </summary>
    public int WriteTimeoutMs { get; set; } = 100;

    /// <summary>
    /// Default minimum PWM pulse width in microseconds.
    /// </summary>
    public double DefaultMinPwmUs { get; set; } = 500;

    /// <summary>
    /// Default maximum PWM pulse width in microseconds.
    /// </summary>
    public double DefaultMaxPwmUs { get; set; } = 2500;

    /// <summary>
    /// Default speed limit for servos (0 = unlimited).
    /// </summary>
    public int DefaultSpeedLimit { get; set; } = 0;

    /// <summary>
    /// Default acceleration limit for servos (0 = unlimited).
    /// </summary>
    public int DefaultAccelerationLimit { get; set; } = 0;

    /// <summary>
    /// Whether to disable servos when the controller is disposed.
    /// </summary>
    public bool DisableOnShutdown { get; set; } = true;

    /// <summary>
    /// Custom channel mapping (LegId * 3 + JointIndex -> Maestro Channel).
    /// If null, uses default 1:1 mapping.
    /// </summary>
    public int[]? ChannelMapping { get; set; }

    /// <summary>
    /// Per-servo calibration settings keyed by channel number (0-17).
    /// </summary>
    public Dictionary<int, ServoCalibrationConfig>? ServoCalibrations { get; set; }
}

/// <summary>
/// Calibration configuration for an individual servo (for config file).
/// </summary>
public class ServoCalibrationConfig
{
    /// <summary>
    /// Minimum PWM pulse width in microseconds.
    /// </summary>
    public double MinPwmUs { get; set; } = 500;

    /// <summary>
    /// Maximum PWM pulse width in microseconds.
    /// </summary>
    public double MaxPwmUs { get; set; } = 2500;

    /// <summary>
    /// Center position offset in degrees.
    /// </summary>
    public double CenterOffsetDegrees { get; set; } = 0;

    /// <summary>
    /// Speed limit (0 = unlimited).
    /// </summary>
    public int SpeedLimit { get; set; } = 0;

    /// <summary>
    /// Acceleration limit (0 = unlimited).
    /// </summary>
    public int AccelerationLimit { get; set; } = 0;
}

public class MovementConfiguration
{
    /// <summary>
    /// Default walking speed in m/s.
    /// </summary>
    public double DefaultSpeed { get; set; } = 0.15;

    /// <summary>
    /// Maximum walking speed in m/s.
    /// </summary>
    public double MaxSpeed { get; set; } = 0.3;

    /// <summary>
    /// Default gait type.
    /// </summary>
    public string DefaultGait { get; set; } = "Ripple";

    /// <summary>
    /// Step height in meters.
    /// </summary>
    public double StepHeight { get; set; } = 0.03;

    /// <summary>
    /// Step length in meters.
    /// </summary>
    public double StepLength { get; set; } = 0.06;

    /// <summary>
    /// Body height from ground in meters.
    /// </summary>
    public double BodyHeight { get; set; } = 0.08;

    /// <summary>
    /// Leg dimensions for IK calculations.
    /// </summary>
    public LegDimensions LegDimensions { get; set; } = new();
}

public class LegDimensions
{
    /// <summary>
    /// Coxa (hip) segment length in meters.
    /// </summary>
    public double CoxaLength { get; set; } = 0.03;

    /// <summary>
    /// Femur (thigh) segment length in meters.
    /// </summary>
    public double FemurLength { get; set; } = 0.05;

    /// <summary>
    /// Tibia (shin) segment length in meters.
    /// </summary>
    public double TibiaLength { get; set; } = 0.07;
}

public class SensorConfiguration
{
    /// <summary>
    /// GPS module configuration.
    /// </summary>
    public GpsConfig Gps { get; set; } = new();

    /// <summary>
    /// IMU sensor configuration.
    /// </summary>
    public ImuConfig Imu { get; set; } = new();

    /// <summary>
    /// Distance sensor configuration.
    /// </summary>
    public DistanceSensorConfig DistanceSensors { get; set; } = new();

    /// <summary>
    /// Touch/force sensor configuration.
    /// </summary>
    public TouchSensorConfig TouchSensors { get; set; } = new();

    /// <summary>
    /// Power monitoring configuration.
    /// </summary>
    public PowerMonitorConfig PowerMonitor { get; set; } = new();
}

public class GpsConfig
{
    public bool Enabled { get; set; } = true;
    public string SerialPort { get; set; } = "/dev/ttyAMA0";
    public int BaudRate { get; set; } = 9600;
    public int UpdateRateHz { get; set; } = 1;
}

public class ImuConfig
{
    public bool Enabled { get; set; } = true;
    public int I2CAddress { get; set; } = 0x28;
    public int UpdateRateHz { get; set; } = 100;
    public string FilterType { get; set; } = "Madgwick";
}

public class DistanceSensorConfig
{
    public bool Enabled { get; set; } = true;
    public int SensorCount { get; set; } = 6;
    public int[] I2CAddresses { get; set; } = { 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E };
    public double MaxRange { get; set; } = 4.0;
    public int UpdateRateHz { get; set; } = 20;
}

public class TouchSensorConfig
{
    public bool Enabled { get; set; } = true;
    public int SensorCount { get; set; } = 6;
    public double ForceThreshold { get; set; } = 0.5;
    public int UpdateRateHz { get; set; } = 50;
}

public class PowerMonitorConfig
{
    public bool Enabled { get; set; } = true;
    public int I2CAddress { get; set; } = 0x40;
    public double LowBatteryThreshold { get; set; } = 20.0;
    public double CriticalBatteryThreshold { get; set; } = 10.0;
    public int UpdateRateHz { get; set; } = 1;
}

public class VisionConfiguration
{
    /// <summary>
    /// Camera configuration.
    /// </summary>
    public CameraConfig Camera { get; set; } = new();

    /// <summary>
    /// Hailo accelerator configuration.
    /// </summary>
    public HailoConfig Hailo { get; set; } = new();

    /// <summary>
    /// AI model configuration.
    /// </summary>
    public ModelConfig Models { get; set; } = new();
}

public class CameraConfig
{
    public bool Enabled { get; set; } = true;
    public int Width { get; set; } = 640;
    public int Height { get; set; } = 480;
    public int FrameRate { get; set; } = 30;
    public string Format { get; set; } = "RGB888";
}

public class HailoConfig
{
    public bool Enabled { get; set; } = true;
    public string DevicePath { get; set; } = "/dev/hailo0";
    public int MaxBatchSize { get; set; } = 1;
    public double PowerMode { get; set; } = 1.0; // 0.0 to 1.0
}

public class ModelConfig
{
    /// <summary>
    /// Object detection model configuration.
    /// </summary>
    public ObjectDetectionModelConfig ObjectDetection { get; set; } = new();

    /// <summary>
    /// Terrain classification model configuration.
    /// </summary>
    public ClassificationModelConfig TerrainClassification { get; set; } = new();
}

public class ObjectDetectionModelConfig
{
    public bool Enabled { get; set; } = true;
    public string ModelPath { get; set; } = "models/yolov8n.hef";
    public string ModelName { get; set; } = "YOLOv8n";
    public double ConfidenceThreshold { get; set; } = 0.5;
    public double NmsThreshold { get; set; } = 0.45;
    public int InputWidth { get; set; } = 640;
    public int InputHeight { get; set; } = 640;
}

public class ClassificationModelConfig
{
    public bool Enabled { get; set; } = true;
    public string ModelPath { get; set; } = "models/mobilenetv3_terrain.hef";
    public string ModelName { get; set; } = "MobileNetV3-Small";
    public double ConfidenceThreshold { get; set; } = 0.6;
    public int InputWidth { get; set; } = 224;
    public int InputHeight { get; set; } = 224;
}

public class AutonomyConfiguration
{
    /// <summary>
    /// Default operation mode on startup.
    /// </summary>
    public string DefaultMode { get; set; } = "SemiAutonomous";

    /// <summary>
    /// Risk level requiring operator confirmation.
    /// </summary>
    public string ConfirmationRiskThreshold { get; set; } = "High";

    /// <summary>
    /// Timeout for operator confirmation in seconds.
    /// </summary>
    public int ConfirmationTimeoutSeconds { get; set; } = 60;

    /// <summary>
    /// Action to take on confirmation timeout.
    /// </summary>
    public string TimeoutAction { get; set; } = "SafeAction";

    /// <summary>
    /// Maximum allowed distance from mission boundary in meters.
    /// </summary>
    public double MaxBoundaryViolation { get; set; } = 5.0;

    /// <summary>
    /// Minimum obstacle clearance in meters.
    /// </summary>
    public double MinObstacleClearance { get; set; } = 0.2;
}

public class CommunicationConfiguration
{
    /// <summary>
    /// LoRaWAN configuration.
    /// </summary>
    public LoRaWanConfig LoRaWan { get; set; } = new();

    /// <summary>
    /// Azure IoT Hub configuration.
    /// </summary>
    public AzureIoTConfig AzureIoT { get; set; } = new();
}

public class LoRaWanConfig
{
    public bool Enabled { get; set; } = true;
    public string DeviceEUI { get; set; } = string.Empty;
    public string ApplicationEUI { get; set; } = string.Empty;
    public string ApplicationKey { get; set; } = string.Empty;
    public string Region { get; set; } = "EU868";
    public int SpreadingFactor { get; set; } = 7;
    public int TxPower { get; set; } = 14;
    public bool AdaptiveDataRate { get; set; } = true;
}

public class AzureIoTConfig
{
    public string IoTHubHostname { get; set; } = string.Empty;
    public string DeviceId { get; set; } = string.Empty;
    
    /// <summary>
    /// Authentication method: "ConnectionString", "X509", or "DPS"
    /// </summary>
    public string AuthMethod { get; set; } = "X509";
    
    public string ConnectionString { get; set; } = string.Empty;
    public string CertificatePath { get; set; } = "certs/device.pfx";
    public string CertificatePassword { get; set; } = string.Empty;
    
    /// <summary>
    /// DPS provisioning settings.
    /// </summary>
    public DpsConfig DPS { get; set; } = new();
}

public class DpsConfig
{
    public string GlobalEndpoint { get; set; } = "global.azure-devices-provisioning.net";
    public string IdScope { get; set; } = string.Empty;
    public string RegistrationId { get; set; } = string.Empty;
}

public class TelemetryConfiguration
{
    /// <summary>
    /// Telemetry send interval in seconds.
    /// </summary>
    public int SendIntervalSeconds { get; set; } = 30;

    /// <summary>
    /// Batch creation interval in seconds.
    /// </summary>
    public int BatchIntervalSeconds { get; set; } = 10;

    /// <summary>
    /// Upload interval in seconds.
    /// </summary>
    public int UploadIntervalSeconds { get; set; } = 60;

    /// <summary>
    /// Maximum data points per batch.
    /// </summary>
    public int MaxBatchSize { get; set; } = 100;

    /// <summary>
    /// Maximum buffered batches.
    /// </summary>
    public int MaxBufferedBatches { get; set; } = 50;

    /// <summary>
    /// Maximum local buffer size in MB.
    /// </summary>
    public int MaxBufferSizeMB { get; set; } = 100;

    /// <summary>
    /// Local storage path for telemetry buffer.
    /// </summary>
    public string BufferPath { get; set; } = "/var/hexapod/telemetry";

    /// <summary>
    /// Local storage path (alias for BufferPath).
    /// </summary>
    public string LocalStoragePath { get; set; } = "/var/hexapod/telemetry";

    /// <summary>
    /// Minimum connection quality for bulk upload.
    /// </summary>
    public int MinConnectionQualityForUpload { get; set; } = 3;

    /// <summary>
    /// Enable detailed activity logging.
    /// </summary>
    public bool EnableDetailedLogging { get; set; } = true;
}

public class PowerConfiguration
{
    /// <summary>
    /// Enable power-saving mode.
    /// </summary>
    public bool EnablePowerSaving { get; set; } = true;

    /// <summary>
    /// Battery capacity in mAh.
    /// </summary>
    public int BatteryCapacityMah { get; set; } = 5000;

    /// <summary>
    /// Nominal battery voltage.
    /// </summary>
    public double NominalVoltage { get; set; } = 11.1;

    /// <summary>
    /// Low power mode trigger threshold (%).
    /// </summary>
    public double LowPowerThreshold { get; set; } = 25.0;

    /// <summary>
    /// Critical power mode trigger threshold (%).
    /// </summary>
    public double CriticalPowerThreshold { get; set; } = 10.0;
}

/// <summary>
/// Configuration for mock mode to enable development and debugging without hardware.
/// </summary>
public class MockModeConfiguration
{
    /// <summary>
    /// Enable mock mode - when true, all hardware is simulated.
    /// </summary>
    public bool Enabled { get; set; } = false;

    /// <summary>
    /// Enable mock servo controller.
    /// </summary>
    public bool MockServos { get; set; } = true;

    /// <summary>
    /// Enable mock sensors (IMU, GPS, distance, touch, power).
    /// </summary>
    public bool MockSensors { get; set; } = true;

    /// <summary>
    /// Enable mock communication services (Azure IoT Hub, LoRaWAN).
    /// </summary>
    public bool MockCommunication { get; set; } = true;

    /// <summary>
    /// Enable mock vision/camera services.
    /// </summary>
    public bool MockVision { get; set; } = true;

    /// <summary>
    /// Simulated servo response delay in milliseconds.
    /// </summary>
    public int SimulatedServoDelayMs { get; set; } = 5;

    /// <summary>
    /// Simulated sensor polling interval in milliseconds.
    /// </summary>
    public int SimulatedSensorPollingMs { get; set; } = 20;

    /// <summary>
    /// Enable verbose logging of mock operations for debugging.
    /// </summary>
    public bool VerboseLogging { get; set; } = true;

    /// <summary>
    /// Simulate random sensor noise for realistic testing.
    /// </summary>
    public bool SimulateNoise { get; set; } = true;

    /// <summary>
    /// Noise amplitude as percentage (0.0-1.0).
    /// </summary>
    public double NoiseAmplitude { get; set; } = 0.02;

    /// <summary>
    /// Simulated GPS position (latitude).
    /// </summary>
    public double SimulatedLatitude { get; set; } = 48.8566;

    /// <summary>
    /// Simulated GPS position (longitude).
    /// </summary>
    public double SimulatedLongitude { get; set; } = 2.3522;

    /// <summary>
    /// Simulated battery percentage (0-100).
    /// </summary>
    public double SimulatedBatteryPercent { get; set; } = 85.0;

    /// <summary>
    /// Simulated battery voltage.
    /// </summary>
    public double SimulatedBatteryVoltage { get; set; } = 11.8;
}
