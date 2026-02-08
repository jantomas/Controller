using Hexapod.Core.Configuration;
using Hexapod.Movement.Servo;
using Hexapod.ServoTest;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Spectre.Console;

// Banner
AnsiConsole.Write(new FigletText("Hexapod").Color(Color.Green));
AnsiConsole.MarkupLine("[bold blue]Servo Test Utility[/]");
AnsiConsole.MarkupLine("[grey]Interactive tool for verifying servo wiring and channel configuration[/]");
AnsiConsole.WriteLine();

// Load configuration
var environment = Environment.GetEnvironmentVariable("DOTNET_ENVIRONMENT") 
                  ?? Environment.GetEnvironmentVariable("ASPNETCORE_ENVIRONMENT") 
                  ?? "Production";
AnsiConsole.MarkupLine($"[grey]Environment: {environment}[/]");                 
var configuration = new ConfigurationBuilder()
    .SetBasePath(AppContext.BaseDirectory)
    .AddJsonFile("hexapod.json", optional: false)
    .AddJsonFile($"hexapod.{environment}.json", optional: true)
    .AddJsonFile("appsettings.json", optional: true)
    .AddJsonFile($"appsettings.{environment}.json", optional: true)
    .Build();

// Setup DI
var services = new ServiceCollection();
services.Configure<HexapodConfiguration>(configuration.GetSection("Hexapod"));
services.AddLogging(builder => builder.AddConsole().SetMinimumLevel(LogLevel.Warning));

var serviceProvider = services.BuildServiceProvider();
var config = serviceProvider.GetRequiredService<IOptions<HexapodConfiguration>>();
var loggerFactory = serviceProvider.GetRequiredService<ILoggerFactory>();

AnsiConsole.MarkupLine($"[grey]Port: {config.Value.Hardware.MaestroServo.SerialPort}[/]");                 

// Create servo controller
PololuMaestroServoController? controller = null;
try
{
    controller = new PololuMaestroServoController(config, loggerFactory.CreateLogger<PololuMaestroServoController>());
    AnsiConsole.MarkupLine($"[green]✓[/] Connected to Maestro on [cyan]{config.Value.Hardware.MaestroServo.SerialPort}[/]");
}
catch (Exception ex)
{
    AnsiConsole.MarkupLine($"[red]✗[/] Failed to connect to Maestro: {ex.Message}");
    AnsiConsole.MarkupLine("[yellow]Running in simulation mode (no hardware)[/]");
}

var tester = new ServoTester(controller, config.Value.Hardware.MaestroServo);

// Main loop
var running = true;
while (running)
{
    AnsiConsole.WriteLine();
    var choice = AnsiConsole.Prompt(
        new SelectionPrompt<string>()
            .Title("[bold]Select an action:[/]")
            .AddChoices(new[]
            {
                "Test single channel",
                "Test single leg",
                "Test all servos",
                "Sweep channel",
                "Show channel mapping",
                "Show servo positions",
                "Center all servos",
                "Disable all servos",
                "Enable all servos",
                "Configure serial port",
                "Exit"
            }));

    try
    {
        switch (choice)
        {
            case "Test single channel":
                tester.TestSingleChannel();
                break;
            case "Test single leg":
                tester.TestSingleLeg();
                break;
            case "Test all servos":
                tester.TestAllServos();
                break;
            case "Sweep channel":
                tester.SweepChannel();
                break;
            case "Show channel mapping":
                tester.ShowChannelMapping();
                break;
            case "Show servo positions":
                tester.ShowServoPositions();
                break;
            case "Center all servos":
                tester.CenterAllServos();
                break;
            case "Disable all servos":
                tester.DisableAllServos();
                break;
            case "Enable all servos":
                tester.EnableAllServos();
                break;
            case "Configure serial port":
                ConfigureSerialPort();
                break;
            case "Exit":
                running = false;
                break;
        }
    }
    catch (Exception ex)
    {
        AnsiConsole.MarkupLine($"[red]Error:[/] {ex.Message}");
    }
}

// Cleanup
controller?.Dispose();
AnsiConsole.MarkupLine("[grey]Goodbye![/]");

void ConfigureSerialPort()
{
    var ports = System.IO.Ports.SerialPort.GetPortNames();
    if (ports.Length == 0)
    {
        AnsiConsole.MarkupLine("[yellow]No serial ports found[/]");
        return;
    }

    AnsiConsole.MarkupLine("[bold]Available serial ports:[/]");
    foreach (var port in ports)
    {
        AnsiConsole.MarkupLine($"  • {port}");
    }
    AnsiConsole.MarkupLine("\n[grey]Edit appsettings.json to change the serial port configuration[/]");
}
