using Hexapod.Core.Configuration;
using Hexapod.Core.Services;
using Hexapod.Core.StateMachine;
using Hexapod.Host;
using Hexapod.Host.Services;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Serilog;

// Configure Serilog
Log.Logger = new LoggerConfiguration()
    .MinimumLevel.Debug()
    .Enrich.FromLogContext()
    .WriteTo.Console(outputTemplate: "[{Timestamp:HH:mm:ss} {Level:u3}] {SourceContext}: {Message:lj}{NewLine}{Exception}")
    .WriteTo.File(
        path: "/var/log/hexapod/hexapod-.log",
        rollingInterval: RollingInterval.Day,
        retainedFileCountLimit: 7,
        outputTemplate: "{Timestamp:yyyy-MM-dd HH:mm:ss.fff zzz} [{Level:u3}] {SourceContext}: {Message:lj}{NewLine}{Exception}")
    .CreateLogger();

try
{
    Log.Information("=== Hexapod Controller Starting ===");
    Log.Information("Platform: {OS}, Architecture: {Arch}", 
        Environment.OSVersion, 
        System.Runtime.InteropServices.RuntimeInformation.ProcessArchitecture);

    var builder = Host.CreateApplicationBuilder(args);

    // Configure logging
    builder.Services.AddSerilog();

    // Clear default configuration sources and rebuild with correct base path
    // This ensures appsettings.Development.json properly overrides appsettings.json
    // Note: Command line args are parsed by CreateApplicationBuilder for environment
    var environmentName = builder.Environment.EnvironmentName;
    builder.Configuration.Sources.Clear();
    builder.Configuration
        .SetBasePath(AppContext.BaseDirectory)
        .AddJsonFile("appsettings.json", optional: false, reloadOnChange: true)
        .AddJsonFile($"appsettings.{environmentName}.json", optional: true, reloadOnChange: true)
        .AddEnvironmentVariables("HEXAPOD_")
        .AddCommandLine(args);

    // Bind configuration
    builder.Services.Configure<HexapodConfiguration>(builder.Configuration.GetSection("Hexapod"));

    // Get mock mode configuration early to decide which services to register
    var tempConfig = builder.Configuration.GetSection("Hexapod").Get<HexapodConfiguration>() ?? new HexapodConfiguration();
    var mockMode = tempConfig.MockMode;

    Log.Debug("Environment: {Environment}, MockMode.Enabled: {Enabled}", 
        environmentName, mockMode.Enabled);

    if (mockMode.Enabled)
    {
        Log.Warning("╔════════════════════════════════════════════════════════════╗");
        Log.Warning("║           🎮 MOCK MODE ENABLED - NO HARDWARE 🎮            ║");
        Log.Warning("║     All hardware interactions are simulated for dev/test   ║");
        Log.Warning("╚════════════════════════════════════════════════════════════╝");
    }

    // Register core services
    builder.Services.AddSingleton<IEventBus, EventBus>();
    builder.Services.AddSingleton<IOperationStateMachine, OperationStateMachine>();

    // Register services based on mock mode
    RegisterHardwareServices(builder.Services, mockMode);

    // Register movement services
    builder.Services.AddSingleton<Hexapod.Movement.Services.IMovementController, Hexapod.Movement.Services.MovementController>();

    // Register autonomy services
    builder.Services.AddSingleton<Hexapod.Autonomy.Decision.IDecisionEngine, Hexapod.Autonomy.Decision.DecisionEngine>();
    builder.Services.AddSingleton<Hexapod.Autonomy.Confirmation.IOperatorConfirmationService, Hexapod.Autonomy.Confirmation.OperatorConfirmationService>();
    builder.Services.AddSingleton<Hexapod.Autonomy.Mission.IMissionPlanner, Hexapod.Autonomy.Mission.MissionPlanner>();
    builder.Services.AddSingleton<Hexapod.Autonomy.Confirmation.AutonomyManager>();

    // Register telemetry services
    builder.Services.AddSingleton<Hexapod.Telemetry.Logging.IActivityLogger, Hexapod.Telemetry.Logging.ActivityLogger>();
    builder.Services.AddSingleton<Hexapod.Telemetry.Collection.ITelemetryCollector, Hexapod.Telemetry.Collection.TelemetryCollector>();
    builder.Services.AddSingleton<Hexapod.Telemetry.Upload.IUploadDestination, Hexapod.Telemetry.Upload.AzureIoTUploadDestination>();
    builder.Services.AddSingleton<Hexapod.Telemetry.Upload.ITelemetryUploader, Hexapod.Telemetry.Upload.TelemetryUploader>();

    // Register hosted services
    builder.Services.AddHostedService<SystemHealthMonitor>();
    builder.Services.AddHostedService<HexapodControllerService>();

    var host = builder.Build();

    Log.Information("Services registered successfully");
    
    await host.RunAsync();
}
catch (Exception ex)
{
    Log.Fatal(ex, "Hexapod Controller terminated unexpectedly");
    Environment.ExitCode = 1;
}
finally
{
    Log.Information("=== Hexapod Controller Stopped ===");
    await Log.CloseAndFlushAsync();
}

/// <summary>
/// Registers hardware-related services based on mock mode configuration.
/// </summary>
static void RegisterHardwareServices(IServiceCollection services, MockModeConfiguration mockMode)
{
    // ═══════════════════════════════════════════════════════════════════════════
    // SERVO CONTROLLER
    // ═══════════════════════════════════════════════════════════════════════════
    if (mockMode.Enabled && mockMode.MockServos)
    {
        Log.Information("🎮 Registering Mock Servo Controller");
        services.AddSingleton<Hexapod.Movement.Services.IServoController, Hexapod.Movement.Mock.MockServoController>();
    }
    else
    {
        services.AddSingleton<Hexapod.Movement.Services.IServoController>(sp =>
        {
            var config = sp.GetRequiredService<IOptions<HexapodConfiguration>>().Value;
            var loggerFactory = sp.GetRequiredService<ILoggerFactory>();

            return config.Hardware.ServoControllerType switch
            {
                Hexapod.Core.Configuration.ServoControllerType.PololuMaestro =>
                    new Hexapod.Movement.Servo.PololuMaestroServoController(
                        sp.GetRequiredService<IOptions<HexapodConfiguration>>(),
                        loggerFactory.CreateLogger<Hexapod.Movement.Servo.PololuMaestroServoController>()),
                Hexapod.Core.Configuration.ServoControllerType.Pca9685 =>
                    new Hexapod.Movement.Services.Pca9685ServoController(
                        sp.GetRequiredService<IOptions<HexapodConfiguration>>(),
                        loggerFactory.CreateLogger<Hexapod.Movement.Services.Pca9685ServoController>()),
                _ => throw new InvalidOperationException($"Unknown servo controller type: {config.Hardware.ServoControllerType}")
            };
        });
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SENSORS
    // ═══════════════════════════════════════════════════════════════════════════
    if (mockMode.Enabled && mockMode.MockSensors)
    {
        Log.Information("🎮 Registering Mock Sensors (GPS, IMU, Power, Distance, Touch)");
        
        // Register mock sensor implementations
        services.AddSingleton<Hexapod.Sensors.Abstractions.IGpsSensor, Hexapod.Sensors.Mock.MockGpsSensor>();
        services.AddSingleton<Hexapod.Sensors.Abstractions.IImuSensor, Hexapod.Sensors.Mock.MockImuSensor>();
        services.AddSingleton<Hexapod.Sensors.Abstractions.IPowerSensor, Hexapod.Sensors.Mock.MockPowerSensor>();
        services.AddSingleton<Hexapod.Sensors.Abstractions.IDistanceSensorArray, Hexapod.Sensors.Mock.MockDistanceSensorArray>();
        services.AddSingleton<Hexapod.Sensors.Abstractions.ITouchSensorArray, Hexapod.Sensors.Mock.MockTouchSensorArray>();
    }
    else
    {
        // Register real sensor implementations
        services.AddSingleton<Hexapod.Sensors.Abstractions.IGpsSensor, Hexapod.Sensors.Gps.GpsSensor>();
        services.AddSingleton<Hexapod.Sensors.Abstractions.IImuSensor, Hexapod.Sensors.Imu.ImuSensor>();
        // Note: Real power, distance, and touch sensors would be registered here
    }
    
    // Sensor manager uses injected sensors
    services.AddSingleton<Hexapod.Sensors.Services.ISensorManager, Hexapod.Sensors.Services.SensorManager>();

    // ═══════════════════════════════════════════════════════════════════════════
    // COMMUNICATION
    // ═══════════════════════════════════════════════════════════════════════════
    if (mockMode.Enabled && mockMode.MockCommunication)
    {
        Log.Information("🎮 Registering Mock Communication Services (IoT Hub, LoRaWAN)");
        
        services.AddSingleton<Hexapod.Communication.Mock.MockAzureIoTHubService>();
        services.AddSingleton<Hexapod.Communication.Mock.MockLoRaWanService>();
        
        services.AddSingleton<Hexapod.Communication.AzureIoT.IAzureIoTHubService>(sp => 
            sp.GetRequiredService<Hexapod.Communication.Mock.MockAzureIoTHubService>());
        services.AddSingleton<Hexapod.Communication.LoRaWAN.ILoRaWanService>(sp => 
            sp.GetRequiredService<Hexapod.Communication.Mock.MockLoRaWanService>());
        
        // Use mock communication manager that coordinates mock services
        services.AddSingleton<Hexapod.Communication.ICommunicationManager>(sp =>
            new Hexapod.Communication.Mock.MockCommunicationManager(
                sp.GetRequiredService<IOptions<HexapodConfiguration>>(),
                sp.GetRequiredService<Hexapod.Communication.Mock.MockAzureIoTHubService>(),
                sp.GetRequiredService<Hexapod.Communication.Mock.MockLoRaWanService>(),
                sp.GetRequiredService<ILoggerFactory>().CreateLogger<Hexapod.Communication.Mock.MockCommunicationManager>()));
    }
    else
    {
        services.AddSingleton<Hexapod.Communication.AzureIoT.IAzureIoTHubService, Hexapod.Communication.AzureIoT.AzureIoTHubService>();
        services.AddSingleton<Hexapod.Communication.LoRaWAN.ILoRaWanService, Hexapod.Communication.LoRaWAN.LoRaWanService>();
        services.AddSingleton<Hexapod.Communication.ICommunicationManager, Hexapod.Communication.CommunicationManager>();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // VISION
    // ═══════════════════════════════════════════════════════════════════════════
    if (mockMode.Enabled && mockMode.MockVision)
    {
        Log.Information("🎮 Registering Mock Vision Services (Hailo inference disabled)");
        // For now, still register real services but they will handle missing hardware gracefully
        // TODO: Add MockInferenceEngine, MockObjectDetector, MockTerrainClassifier
    }
    
    services.AddSingleton<Hexapod.Vision.Abstractions.IInferenceEngine, Hexapod.Vision.Hailo.HailoInferenceEngine>();
    services.AddSingleton<Hexapod.Vision.Abstractions.IObjectDetector, Hexapod.Vision.Detection.YoloObjectDetector>();
    services.AddSingleton<Hexapod.Vision.Abstractions.ITerrainClassifier, Hexapod.Vision.Detection.TerrainClassifier>();
}
