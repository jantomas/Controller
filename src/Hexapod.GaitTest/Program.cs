using System.Numerics;
using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.GaitTest;
using Hexapod.Movement.Gait;
using Hexapod.Movement.Kinematics;
using Hexapod.Movement.Servo;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Spectre.Console;

// Build configuration from shared hexapod.json + project-specific appsettings
var environment = Environment.GetEnvironmentVariable("DOTNET_ENVIRONMENT") ?? "Development";
var configuration = new ConfigurationBuilder()
    .SetBasePath(AppContext.BaseDirectory)
    .AddJsonFile("hexapod.json", optional: false, reloadOnChange: false)
    .AddJsonFile($"hexapod.{environment}.json", optional: true, reloadOnChange: false)
    .AddJsonFile("appsettings.json", optional: true, reloadOnChange: false)
    .AddJsonFile($"appsettings.{environment}.json", optional: true, reloadOnChange: false)
    .AddEnvironmentVariables()
    .Build();

// Setup DI for hardware configuration
var services = new ServiceCollection();
services.Configure<HexapodConfiguration>(configuration.GetSection("Hexapod"));
services.AddLogging(builder => builder.AddConsole().SetMinimumLevel(LogLevel.Warning));

var serviceProvider = services.BuildServiceProvider();
var config = serviceProvider.GetRequiredService<IOptions<HexapodConfiguration>>();
var loggerFactory = serviceProvider.GetRequiredService<ILoggerFactory>();

// Read kinematics configuration (strongly-typed, per-leg config)
var kinConfig = new KinematicsConfiguration();
configuration.GetSection(KinematicsConfiguration.SectionName).Bind(kinConfig);

// Create hexapod body model from per-leg config
var body = new HexapodBody(kinConfig);

// Try to create servo controller
PololuMaestroServoController? controller = null;
try
{
    controller = new PololuMaestroServoController(config, loggerFactory.CreateLogger<PololuMaestroServoController>());
    AnsiConsole.MarkupLine($"[green]✓[/] Connected to Maestro on [cyan]{config.Value.Hardware.MaestroServo.SerialPort}[/]");
}
catch (Exception ex)
{
    AnsiConsole.MarkupLine($"[yellow]⚠[/] No hardware controller: {ex.Message}");
    AnsiConsole.MarkupLine("[grey]Running in simulation mode[/]");
}

var tester = new GaitTester(body, controller, config.Value.Hardware.MaestroServo.ChannelMapping);

// Welcome banner
AnsiConsole.Write(
    new FigletText("Gait Tester")
        .LeftJustified()
        .Color(Color.Green));

AnsiConsole.MarkupLine("[grey]Hexapod Inverse Kinematics & Gait Pattern Test Utility[/]");
AnsiConsole.MarkupLine($"[grey]Leg dimensions (leg 0): Coxa={kinConfig.Legs[0].CoxaLengthMm:F1}mm, Femur={kinConfig.Legs[0].FemurLengthMm:F1}mm, Tibia={kinConfig.Legs[0].TibiaLengthMm:F1}mm, Mount radius={kinConfig.Legs[0].MountRadiusMm:F1}mm[/]");
AnsiConsole.MarkupLine($"[grey]Configuration loaded from hexapod.json[/]");
AnsiConsole.MarkupLine(tester.HasHardware 
    ? "[green]Hardware controller connected - physical execution available[/]\n" 
    : "[yellow]Simulation mode - no hardware connected[/]\n");

// Main menu loop
while (true)
{
    var hardwareStatus = tester.HardwareEnabled ? "[green]HW ON[/]" : "[grey]HW OFF[/]";
    
    var choices = new List<string>
    {
        "🦵 Test leg lift/put down",
        "👣 Test single step",
        "🚶 Run tripod gait sample",
        "🌊 Run wave gait sample",
        "〰️ Run ripple gait sample",
        "🔄 Run metachronal gait sample",
        "📊 Compare all gait patterns",
        "🎯 Test inverse kinematics",
        "📐 Test joint angle reachability",
        "📐 Show leg workspace",
        "📋 Show leg configuration",
        "🔧 Interactive foot position",
        "🧍 Standing pose",
        "🎯 Move leg to target",
    };
    
    // Add hardware-specific options
    if (tester.HasHardware)
    {
        choices.Add(tester.HardwareEnabled 
            ? "🔌 Toggle hardware mode [green](ON)[/]" 
            : "🔌 Toggle hardware mode [grey](OFF)[/]");
        choices.Add("🏠 Center all servos");
        choices.Add("⚠️ Disable all servos");
        choices.Add("✅ Enable all servos");
    }
    
    choices.Add("❌ Exit");
    
    var choice = AnsiConsole.Prompt(
        new SelectionPrompt<string>()
            .Title($"[bold]Select operation:[/] {hardwareStatus}")
            .PageSize(18)
            .HighlightStyle(Style.Parse("cyan"))
            .AddChoices(choices));

    AnsiConsole.WriteLine();

    switch (choice)
    {
        case "🦵 Test leg lift/put down":
            tester.TestLegLiftPutDown();
            break;
        case "👣 Test single step":
            tester.TestSingleStep();
            break;
        case "🚶 Run tripod gait sample":
            tester.RunGaitSample(GaitType.Tripod);
            break;
        case "🌊 Run wave gait sample":
            tester.RunGaitSample(GaitType.Wave);
            break;
        case "〰️ Run ripple gait sample":
            tester.RunGaitSample(GaitType.Ripple);
            break;
        case "🔄 Run metachronal gait sample":
            tester.RunGaitSample(GaitType.Metachronal);
            break;
        case "📊 Compare all gait patterns":
            tester.CompareGaitPatterns();
            break;
        case "🎯 Test inverse kinematics":
            tester.TestInverseKinematics();
            break;
        case "📐 Test joint angle reachability":
            tester.TestJointAngleReachability();
            break;
        case "📐 Show leg workspace":
            tester.ShowLegWorkspace();
            break;
        case "📋 Show leg configuration":
            tester.ShowLegConfiguration();
            break;
        case "🔧 Interactive foot position":
            tester.InteractiveFootPosition();
            break;
        case "🧍 Standing pose":
            tester.GoToStandingPose();
            break;
        case "🎯 Move leg to target":
            tester.MoveToTarget();
            break;
        case var s when s.StartsWith("🔌 Toggle hardware mode"):
            tester.ToggleHardwareMode();
            break;
        case "🏠 Center all servos":
            tester.CenterAllServos();
            break;
        case "⚠️ Disable all servos":
            tester.DisableAllServos();
            break;
        case "✅ Enable all servos":
            tester.EnableAllServos();
            break;
        case "❌ Exit":
            if (tester.HasHardware)
            {
                tester.DisableAllServos();
            }
            AnsiConsole.MarkupLine("[green]Goodbye![/]");
            return;
    }

    AnsiConsole.WriteLine();
    AnsiConsole.MarkupLine("[grey]Press any key to continue...[/]");
    Console.ReadKey(true);
    AnsiConsole.Clear();
}
