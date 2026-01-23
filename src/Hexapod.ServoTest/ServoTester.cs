using System;
using System.Linq;
using Hexapod.Core.Configuration;
using Hexapod.Movement.Servo;
using Spectre.Console;

namespace Hexapod.ServoTest;

/// <summary>
/// Interactive servo tester for verifying wiring and channel configuration.
/// </summary>
public class ServoTester
{
    private readonly PololuMaestroServoController? _controller;
    private readonly MaestroServoConfiguration _config;
    private readonly int[] _channelMap;

    private static readonly string[] LegNames = 
    { 
        "Front Right", "Middle Right", "Rear Right",
        "Rear Left", "Middle Left", "Front Left"
    };

    private static readonly string[] JointNames = { "Coxa", "Femur", "Tibia" };

    public ServoTester(PololuMaestroServoController? controller, MaestroServoConfiguration config)
    {
        _controller = controller;
        _config = config;
        _channelMap = config.ChannelMapping ?? Enumerable.Range(0, 18).ToArray();
    }

    /// <summary>
    /// Test a single servo channel with movement.
    /// </summary>
    public void TestSingleChannel()
    {
        var channel = AnsiConsole.Prompt(
            new TextPrompt<int>("Enter channel number [grey](0-17)[/]:")
                .Validate(c => c >= 0 && c <= 17 ? ValidationResult.Success() : ValidationResult.Error("Channel must be 0-17")));

        var (legId, jointIndex) = GetLegAndJointFromChannel(channel);
        
        AnsiConsole.MarkupLine($"Channel [cyan]{channel}[/] → [yellow]{LegNames[legId]}[/] [green]{JointNames[jointIndex]}[/]");
        
        PerformChannelTest(channel);
    }

    /// <summary>
    /// Test all 3 servos of a single leg.
    /// </summary>
    public void TestSingleLeg()
    {
        var legChoice = AnsiConsole.Prompt(
            new SelectionPrompt<string>()
                .Title("Select leg to test:")
                .AddChoices(LegNames.Select((name, i) => $"{i}: {name}")));

        var legId = int.Parse(legChoice.Split(':')[0]);
        
        AnsiConsole.MarkupLine($"\n[bold]Testing {LegNames[legId]}[/]");
        
        for (int jointIndex = 0; jointIndex < 3; jointIndex++)
        {
            var channel = _channelMap[legId * 3 + jointIndex];
            AnsiConsole.MarkupLine($"\n[yellow]{JointNames[jointIndex]}[/] (Channel {channel}):");
            PerformChannelTest(channel);
            
            if (jointIndex < 2)
            {
                if (!AnsiConsole.Confirm("Continue to next joint?"))
                    break;
            }
        }
    }

    /// <summary>
    /// Test all 18 servos sequentially.
    /// </summary>
    public void TestAllServos()
    {
        AnsiConsole.MarkupLine("[bold]Testing all servos sequentially...[/]");
        
        var testType = AnsiConsole.Prompt(
            new SelectionPrompt<string>()
                .Title("Test type:")
                .AddChoices("Quick pulse (1 second each)", "Full sweep (3 seconds each)", "Interactive (confirm each)"));

        for (int legId = 0; legId < 6; legId++)
        {
            AnsiConsole.MarkupLine($"\n[bold blue]═══ {LegNames[legId]} ═══[/]");
            
            for (int jointIndex = 0; jointIndex < 3; jointIndex++)
            {
                var channel = _channelMap[legId * 3 + jointIndex];
                AnsiConsole.MarkupLine($"  [yellow]{JointNames[jointIndex]}[/] (Ch {channel})");

                switch (testType)
                {
                    case "Quick pulse (1 second each)":
                        QuickPulseChannel(channel);
                        break;
                    case "Full sweep (3 seconds each)":
                        SweepChannelInternal(channel, silent: true);
                        break;
                    case "Interactive (confirm each)":
                        PerformChannelTest(channel);
                        break;
                }
            }
        }
        
        AnsiConsole.MarkupLine("\n[green]✓[/] All servos tested");
    }

    /// <summary>
    /// Sweep a channel through its full range.
    /// </summary>
    public void SweepChannel()
    {
        var channel = AnsiConsole.Prompt(
            new TextPrompt<int>("Enter channel number [grey](0-17)[/]:")
                .Validate(c => c >= 0 && c <= 17 ? ValidationResult.Success() : ValidationResult.Error("Channel must be 0-17")));

        SweepChannelInternal(channel, silent: false);
    }

    /// <summary>
    /// Display the current channel mapping configuration.
    /// </summary>
    public void ShowChannelMapping()
    {
        var table = new Table()
            .Border(TableBorder.Rounded)
            .AddColumn("Leg")
            .AddColumn("Joint")
            .AddColumn("Logical Index")
            .AddColumn("Physical Channel");

        for (int legId = 0; legId < 6; legId++)
        {
            for (int jointIndex = 0; jointIndex < 3; jointIndex++)
            {
                var logicalIndex = legId * 3 + jointIndex;
                var physicalChannel = _channelMap[logicalIndex];
                
                table.AddRow(
                    legId == 0 || jointIndex == 0 ? $"[cyan]{LegNames[legId]}[/]" : "",
                    $"[yellow]{JointNames[jointIndex]}[/]",
                    logicalIndex.ToString(),
                    physicalChannel.ToString());
            }
            
            if (legId < 5)
                table.AddEmptyRow();
        }

        AnsiConsole.Write(table);
        
        // Show mapping array for config
        AnsiConsole.MarkupLine("\n[grey]Channel mapping array for appsettings.json:[/]");
        AnsiConsole.MarkupLine($"[grey]\"ChannelMapping\": [{string.Join(", ", _channelMap)}][/]");
    }

    /// <summary>
    /// Show current positions of all servos.
    /// </summary>
    public void ShowServoPositions()
    {
        if (_controller == null)
        {
            AnsiConsole.MarkupLine("[yellow]No controller connected (simulation mode)[/]");
            return;
        }

        var table = new Table()
            .Border(TableBorder.Rounded)
            .AddColumn("Channel")
            .AddColumn("Leg/Joint")
            .AddColumn("Position (µs)");

        for (int channel = 0; channel < 18; channel++)
        {
            var position = _controller.GetPosition(channel);
            var positionUs = position > 0 ? (position / 4.0).ToString("F1") : "OFF";
            
            var (legId, jointIndex) = GetLegAndJointFromChannel(channel);
            var legJoint = $"{LegNames[legId]} {JointNames[jointIndex]}";

            table.AddRow(
                channel.ToString(),
                legJoint,
                position > 0 ? $"[green]{positionUs}[/]" : "[grey]OFF[/]");
        }

        AnsiConsole.Write(table);
    }

    /// <summary>
    /// Center all servos to their neutral position.
    /// </summary>
    public void CenterAllServos()
    {
        if (_controller == null)
        {
            AnsiConsole.MarkupLine("[yellow]No controller connected (simulation mode)[/]");
            return;
        }

        AnsiConsole.Status()
            .Spinner(Spinner.Known.Dots)
            .Start("Centering all servos...", ctx =>
            {
                _controller.GoHome();
                Thread.Sleep(1000);
            });

        AnsiConsole.MarkupLine("[green]✓[/] All servos centered");
    }

    /// <summary>
    /// Disable all servos (turn off PWM).
    /// </summary>
    public void DisableAllServos()
    {
        if (_controller == null)
        {
            AnsiConsole.MarkupLine("[yellow]No controller connected (simulation mode)[/]");
            return;
        }

        _controller.DisableAll();
        AnsiConsole.MarkupLine("[yellow]⚠[/] All servos disabled");
    }

    /// <summary>
    /// Enable all servos.
    /// </summary>
    public void EnableAllServos()
    {
        if (_controller == null)
        {
            AnsiConsole.MarkupLine("[yellow]No controller connected (simulation mode)[/]");
            return;
        }

        _controller.EnableAll();
        AnsiConsole.MarkupLine("[green]✓[/] All servos enabled");
    }

    private void PerformChannelTest(int channel)
    {
        if (_controller == null)
        {
            AnsiConsole.MarkupLine("[yellow]Simulation:[/] Would move channel {0} to center, min, max", channel);
            return;
        }

        var centerPwm = (_config.DefaultMinPwmUs + _config.DefaultMaxPwmUs) / 2;
        var minPwm = _config.DefaultMinPwmUs;
        var maxPwm = _config.DefaultMaxPwmUs;

        AnsiConsole.Status()
            .Spinner(Spinner.Known.Dots)
            .Start("Testing...", ctx =>
            {
                // Move to center
                ctx.Status("Moving to center...");
                SetChannelPwm(channel, centerPwm);
                Thread.Sleep(500);

                // Move to minimum
                ctx.Status("Moving to minimum...");
                SetChannelPwm(channel, minPwm);
                Thread.Sleep(500);

                // Move to maximum
                ctx.Status("Moving to maximum...");
                SetChannelPwm(channel, maxPwm);
                Thread.Sleep(500);

                // Return to center
                ctx.Status("Returning to center...");
                SetChannelPwm(channel, centerPwm);
                Thread.Sleep(300);
            });

        AnsiConsole.MarkupLine("[green]✓[/] Test complete");
    }

    private void QuickPulseChannel(int channel)
    {
        if (_controller == null)
            return;

        var centerPwm = (_config.DefaultMinPwmUs + _config.DefaultMaxPwmUs) / 2;
        var offsetPwm = centerPwm + 200; // Small movement

        SetChannelPwm(channel, offsetPwm);
        Thread.Sleep(300);
        SetChannelPwm(channel, centerPwm);
        Thread.Sleep(200);
    }

    private void SweepChannelInternal(int channel, bool silent)
    {
        if (_controller == null)
        {
            if (!silent)
                AnsiConsole.MarkupLine("[yellow]No controller connected (simulation mode)[/]");
            return;
        }

        var minPwm = _config.DefaultMinPwmUs;
        var maxPwm = _config.DefaultMaxPwmUs;
        var steps = 50;
        var stepDelay = 30;

        Action<string> status = silent 
            ? _ => { } 
            : msg => AnsiConsole.MarkupLine($"  [grey]{msg}[/]");

        // Sweep from min to max
        status("Sweeping min → max");
        for (int i = 0; i <= steps; i++)
        {
            var pwm = minPwm + (maxPwm - minPwm) * i / steps;
            SetChannelPwm(channel, pwm);
            Thread.Sleep(stepDelay);
        }

        // Sweep from max to min
        status("Sweeping max → min");
        for (int i = steps; i >= 0; i--)
        {
            var pwm = minPwm + (maxPwm - minPwm) * i / steps;
            SetChannelPwm(channel, pwm);
            Thread.Sleep(stepDelay);
        }

        // Return to center
        var centerPwm = (minPwm + maxPwm) / 2;
        SetChannelPwm(channel, centerPwm);
    }

    private void SetChannelPwm(int channel, double pwmMicroseconds)
    {
        if (_controller == null)
            return;

        _controller.SetChannelPosition(channel, pwmMicroseconds);
    }

    private (int LegId, int JointIndex) GetLegAndJointFromChannel(int channel)
    {
        // Find the logical index that maps to this physical channel
        for (int i = 0; i < _channelMap.Length; i++)
        {
            if (_channelMap[i] == channel)
            {
                return (i / 3, i % 3);
            }
        }
        
        // Default: assume 1:1 mapping
        return (channel / 3, channel % 3);
    }
}
