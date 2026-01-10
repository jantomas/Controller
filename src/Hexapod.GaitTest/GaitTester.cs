using System.Numerics;
using System.Text;
using Hexapod.Core.Enums;
using Hexapod.Movement.Gait;
using Hexapod.Movement.Kinematics;
using Spectre.Console;

namespace Hexapod.GaitTest;

/// <summary>
/// Interactive tester for inverse kinematics and gait patterns.
/// </summary>
public class GaitTester
{
    private readonly HexapodBody _body;

    private static readonly string[] LegNames = 
    { 
        "Front Right", "Middle Right", "Rear Right",
        "Rear Left", "Middle Left", "Front Left"
    };

    // Default parameters
    private const double DefaultStepLength = 0.06;  // 60mm
    private const double DefaultStepHeight = 0.03;  // 30mm
    private const double DefaultBodyHeight = 0.08;  // 80mm

    public GaitTester(HexapodBody body)
    {
        _body = body;
    }

    /// <summary>
    /// Test lifting and putting down a single leg.
    /// </summary>
    public void TestLegLiftPutDown()
    {
        var legChoice = AnsiConsole.Prompt(
            new SelectionPrompt<string>()
                .Title("Select leg to test:")
                .AddChoices(LegNames.Select((name, i) => $"{i}: {name}")));

        var legId = int.Parse(legChoice.Split(':')[0]);
        var leg = _body.Legs[legId];

        AnsiConsole.MarkupLine($"\n[bold cyan]Testing {LegNames[legId]} lift/put down[/]");
        
        var liftHeight = AnsiConsole.Prompt(
            new TextPrompt<double>("Lift height (mm) [grey](default 30)[/]:")
                .DefaultValue(30.0)) / 1000.0;

        var steps = 20;
        
        // Get starting position
        var startPos = leg.GetFootPosition();
        
        AnsiConsole.MarkupLine($"\n[yellow]Start position:[/] X={startPos.X * 1000:F1}mm, Y={startPos.Y * 1000:F1}mm, Z={startPos.Z * 1000:F1}mm");

        // Animate lift
        AnsiConsole.Status()
            .Spinner(Spinner.Known.Dots)
            .Start("Lifting leg...", ctx =>
            {
                for (int i = 0; i <= steps; i++)
                {
                    var progress = (double)i / steps;
                    var height = liftHeight * Math.Sin(progress * Math.PI / 2);
                    var targetPos = new Vector3(startPos.X, startPos.Y, startPos.Z + (float)height);
                    
                    var angles = leg.InverseKinematics(targetPos);
                    if (angles.HasValue)
                    {
                        ctx.Status($"Lifting: {progress * 100:F0}% | Height: {height * 1000:F1}mm | " +
                                 $"Coxa: {ToDegrees(angles.Value.Coxa):F1}° Femur: {ToDegrees(angles.Value.Femur):F1}° Tibia: {ToDegrees(angles.Value.Tibia):F1}°");
                    }
                    Thread.Sleep(50);
                }
            });

        var topPos = new Vector3(startPos.X, startPos.Y, startPos.Z + (float)liftHeight);
        var topAngles = leg.InverseKinematics(topPos);
        
        AnsiConsole.MarkupLine($"[green]✓[/] [yellow]Lifted position:[/] Z={topPos.Z * 1000:F1}mm");
        if (topAngles.HasValue)
        {
            ShowJointAngles("At top", topAngles.Value);
        }

        // Animate put down
        AnsiConsole.Status()
            .Spinner(Spinner.Known.Dots)
            .Start("Putting down leg...", ctx =>
            {
                for (int i = steps; i >= 0; i--)
                {
                    var progress = (double)i / steps;
                    var height = liftHeight * Math.Sin(progress * Math.PI / 2);
                    var targetPos = new Vector3(startPos.X, startPos.Y, startPos.Z + (float)height);
                    
                    var angles = leg.InverseKinematics(targetPos);
                    if (angles.HasValue)
                    {
                        ctx.Status($"Lowering: {(1 - progress) * 100:F0}% | Height: {height * 1000:F1}mm");
                    }
                    Thread.Sleep(50);
                }
            });

        AnsiConsole.MarkupLine($"[green]✓[/] Leg returned to ground position");
    }

    /// <summary>
    /// Test a complete single step with one leg.
    /// </summary>
    public void TestSingleStep()
    {
        var legChoice = AnsiConsole.Prompt(
            new SelectionPrompt<string>()
                .Title("Select leg to test:")
                .AddChoices(LegNames.Select((name, i) => $"{i}: {name}")));

        var legId = int.Parse(legChoice.Split(':')[0]);
        var leg = _body.Legs[legId];

        var stepLength = AnsiConsole.Prompt(
            new TextPrompt<double>("Step length (mm) [grey](default 60)[/]:")
                .DefaultValue(60.0)) / 1000.0;

        var stepHeight = AnsiConsole.Prompt(
            new TextPrompt<double>("Step height (mm) [grey](default 30)[/]:")
                .DefaultValue(30.0)) / 1000.0;

        AnsiConsole.MarkupLine($"\n[bold cyan]Testing single step for {LegNames[legId]}[/]");

        var startPos = leg.GetFootPosition();
        var legAngle = leg.MountAngle;
        
        // Calculate step direction (forward in leg's local frame)
        var stepDirX = (float)(Math.Cos(legAngle) * stepLength / 2);
        var stepDirY = (float)(Math.Sin(legAngle) * stepLength / 2);

        var table = new Table()
            .Border(TableBorder.Rounded)
            .AddColumn("Phase")
            .AddColumn("Position (mm)")
            .AddColumn("Joint Angles (°)")
            .AddColumn("Status");

        // Phase 1: Lift
        var liftPos = new Vector3(startPos.X - stepDirX, startPos.Y - stepDirY, startPos.Z + (float)stepHeight);
        var liftAngles = leg.InverseKinematics(liftPos);
        AddPhaseRow(table, "1. Lift", liftPos, liftAngles);

        // Phase 2: Move forward (in air)
        var forwardPos = new Vector3(startPos.X + stepDirX, startPos.Y + stepDirY, startPos.Z + (float)stepHeight);
        var forwardAngles = leg.InverseKinematics(forwardPos);
        AddPhaseRow(table, "2. Forward", forwardPos, forwardAngles);

        // Phase 3: Put down
        var downPos = new Vector3(startPos.X + stepDirX, startPos.Y + stepDirY, startPos.Z);
        var downAngles = leg.InverseKinematics(downPos);
        AddPhaseRow(table, "3. Put down", downPos, downAngles);

        // Phase 4: Push back (stance)
        var pushPos = new Vector3(startPos.X - stepDirX, startPos.Y - stepDirY, startPos.Z);
        var pushAngles = leg.InverseKinematics(pushPos);
        AddPhaseRow(table, "4. Push back", pushPos, pushAngles);

        AnsiConsole.Write(table);

        // Animate the step
        if (AnsiConsole.Confirm("\nAnimate the step?", true))
        {
            AnimateSingleStep(leg, startPos, stepDirX, stepDirY, (float)stepHeight);
        }
    }

    /// <summary>
    /// Run a sample gait pattern animation.
    /// </summary>
    public void RunGaitSample(GaitType gaitType)
    {
        var gait = GaitFactory.Create(gaitType);
        
        AnsiConsole.MarkupLine($"[bold cyan]Running {gaitType} gait sample[/]");
        AnsiConsole.MarkupLine($"[grey]Duty Factor: {gait.DutyFactor:P0} | Stability: {gait.Stability:P0} | Efficiency: {gait.EnergyEfficiency:P0}[/]\n");

        var cycles = AnsiConsole.Prompt(
            new TextPrompt<int>("Number of cycles [grey](default 2)[/]:")
                .DefaultValue(2));

        var stepsPerCycle = 24;
        var totalSteps = cycles * stepsPerCycle;

        AnsiConsole.Write(new Rule("[yellow]Gait Animation[/]"));

        // Legend
        AnsiConsole.MarkupLine("[grey]Legend: [green]█[/]=Ground [red]▀[/]=Air[/]\n");

        for (int step = 0; step < totalSteps; step++)
        {
            var phase = (double)(step % stepsPerCycle) / stepsPerCycle;
            var legPhases = gait.GetLegPhases(phase);

            var visual = new StringBuilder();
            visual.Append($"[grey]Phase {phase:P0}[/] ");
            
            for (int i = 0; i < 6; i++)
            {
                var lp = legPhases[i];
                var symbol = lp.IsSwingPhase ? "[red]▀[/]" : "[green]█[/]";
                visual.Append($"{LegNames[i][..2]}:{symbol} ");
            }

            AnsiConsole.MarkupLine(visual.ToString());
            Thread.Sleep(100);
        }

        AnsiConsole.MarkupLine($"\n[green]✓[/] Completed {cycles} cycle(s) of {gaitType} gait");

        // Show detailed phase information
        if (AnsiConsole.Confirm("\nShow detailed phase table?", false))
        {
            ShowGaitPhaseTable(gait);
        }
    }

    /// <summary>
    /// Compare all gait patterns side by side.
    /// </summary>
    public void CompareGaitPatterns()
    {
        var table = new Table()
            .Border(TableBorder.Rounded)
            .Title("[bold]Gait Pattern Comparison[/]")
            .AddColumn("Property")
            .AddColumn("Tripod")
            .AddColumn("Wave")
            .AddColumn("Ripple")
            .AddColumn("Metachronal");

        var gaits = new[]
        {
            GaitFactory.Create(GaitType.Tripod),
            GaitFactory.Create(GaitType.Wave),
            GaitFactory.Create(GaitType.Ripple),
            GaitFactory.Create(GaitType.Metachronal)
        };

        table.AddRow("Duty Factor", 
            $"{gaits[0].DutyFactor:P0}", 
            $"{gaits[1].DutyFactor:P0}", 
            $"{gaits[2].DutyFactor:P0}", 
            $"{gaits[3].DutyFactor:P0}");

        table.AddRow("Stability", 
            FormatRating(gaits[0].Stability), 
            FormatRating(gaits[1].Stability), 
            FormatRating(gaits[2].Stability), 
            FormatRating(gaits[3].Stability));

        table.AddRow("Efficiency", 
            FormatRating(gaits[0].EnergyEfficiency), 
            FormatRating(gaits[1].EnergyEfficiency), 
            FormatRating(gaits[2].EnergyEfficiency), 
            FormatRating(gaits[3].EnergyEfficiency));

        table.AddRow("Best For",
            "[yellow]Speed[/]",
            "[yellow]Rough terrain[/]",
            "[yellow]Balanced[/]",
            "[yellow]Endurance[/]");

        // Add leg phase patterns
        table.AddEmptyRow();
        table.AddRow("[bold]Leg Phase Offsets[/]", "", "", "", "");
        
        for (int leg = 0; leg < 6; leg++)
        {
            var phases = gaits.Select(g => g.GetLegPhases(0)[leg].Phase).ToArray();
            table.AddRow(
                $"  {LegNames[leg]}",
                $"{phases[0]:F2}",
                $"{phases[1]:F2}",
                $"{phases[2]:F2}",
                $"{phases[3]:F2}");
        }

        AnsiConsole.Write(table);

        // Show timing diagram
        AnsiConsole.WriteLine();
        AnsiConsole.Write(new Rule("[yellow]Timing Diagrams[/]"));
        
        foreach (var gait in gaits)
        {
            ShowGaitTimingDiagram(gait);
            AnsiConsole.WriteLine();
        }
    }

    /// <summary>
    /// Test inverse kinematics with custom positions.
    /// </summary>
    public void TestInverseKinematics()
    {
        var legChoice = AnsiConsole.Prompt(
            new SelectionPrompt<string>()
                .Title("Select leg to test:")
                .AddChoices(LegNames.Select((name, i) => $"{i}: {name}")));

        var legId = int.Parse(legChoice.Split(':')[0]);
        var leg = _body.Legs[legId];

        // Get current position as default
        var currentPos = leg.GetFootPosition();

        AnsiConsole.MarkupLine($"[grey]Current position: X={currentPos.X * 1000:F1}mm, Y={currentPos.Y * 1000:F1}mm, Z={currentPos.Z * 1000:F1}mm[/]\n");

        var x = AnsiConsole.Prompt(
            new TextPrompt<double>($"Target X (mm) [grey](default {currentPos.X * 1000:F1})[/]:")
                .DefaultValue(currentPos.X * 1000)) / 1000.0;

        var y = AnsiConsole.Prompt(
            new TextPrompt<double>($"Target Y (mm) [grey](default {currentPos.Y * 1000:F1})[/]:")
                .DefaultValue(currentPos.Y * 1000)) / 1000.0;

        var z = AnsiConsole.Prompt(
            new TextPrompt<double>($"Target Z (mm) [grey](default {currentPos.Z * 1000:F1})[/]:")
                .DefaultValue(currentPos.Z * 1000)) / 1000.0;

        var target = new Vector3((float)x, (float)y, (float)z);
        var result = leg.InverseKinematics(target);

        if (result.HasValue)
        {
            AnsiConsole.MarkupLine("\n[green]✓ Position is reachable![/]");
            
            var table = new Table()
                .Border(TableBorder.Rounded)
                .AddColumn("Joint")
                .AddColumn("Angle (°)")
                .AddColumn("Angle (rad)")
                .AddColumn("Limits (°)");

            table.AddRow("Coxa", 
                $"{ToDegrees(result.Value.Coxa):F2}",
                $"{result.Value.Coxa:F4}",
                $"{ToDegrees(leg.CoxaLimits.Min):F0} to {ToDegrees(leg.CoxaLimits.Max):F0}");

            table.AddRow("Femur", 
                $"{ToDegrees(result.Value.Femur):F2}",
                $"{result.Value.Femur:F4}",
                $"{ToDegrees(leg.FemurLimits.Min):F0} to {ToDegrees(leg.FemurLimits.Max):F0}");

            table.AddRow("Tibia", 
                $"{ToDegrees(result.Value.Tibia):F2}",
                $"{result.Value.Tibia:F4}",
                $"{ToDegrees(leg.TibiaLimits.Min):F0} to {ToDegrees(leg.TibiaLimits.Max):F0}");

            AnsiConsole.Write(table);

            // Verify with forward kinematics
            var fkResult = leg.ForwardKinematics(result.Value.Coxa, result.Value.Femur, result.Value.Tibia);
            var error = Vector3.Distance(target, fkResult);
            
            AnsiConsole.MarkupLine($"\n[grey]Verification (FK):[/] X={fkResult.X * 1000:F2}mm, Y={fkResult.Y * 1000:F2}mm, Z={fkResult.Z * 1000:F2}mm");
            AnsiConsole.MarkupLine($"[grey]Position error:[/] {error * 1000:F4}mm");
        }
        else
        {
            AnsiConsole.MarkupLine("\n[red]✗ Position is NOT reachable![/]");
            
            // Calculate why
            var mountOffset = new Vector3(
                (float)(leg.MountRadius * Math.Cos(leg.MountAngle)),
                (float)(leg.MountRadius * Math.Sin(leg.MountAngle)),
                0);
            var distance = Vector3.Distance(target, mountOffset);
            var maxReach = leg.CoxaLength + leg.FemurLength + leg.TibiaLength;
            var minReach = Math.Abs(leg.FemurLength - leg.TibiaLength);
            
            AnsiConsole.MarkupLine($"[grey]Distance from mount: {distance * 1000:F1}mm[/]");
            AnsiConsole.MarkupLine($"[grey]Reachable range: {minReach * 1000:F1}mm to {maxReach * 1000:F1}mm[/]");
        }
    }

    /// <summary>
    /// Show the workspace (reachable area) of a leg.
    /// </summary>
    public void ShowLegWorkspace()
    {
        var legChoice = AnsiConsole.Prompt(
            new SelectionPrompt<string>()
                .Title("Select leg:")
                .AddChoices(LegNames.Select((name, i) => $"{i}: {name}")));

        var legId = int.Parse(legChoice.Split(':')[0]);
        var leg = _body.Legs[legId];

        AnsiConsole.MarkupLine($"\n[bold cyan]Workspace for {LegNames[legId]}[/]\n");

        var maxReach = leg.CoxaLength + leg.FemurLength + leg.TibiaLength;
        var minReach = leg.CoxaLength + Math.Abs(leg.FemurLength - leg.TibiaLength);

        var table = new Table()
            .Border(TableBorder.Rounded)
            .AddColumn("Property")
            .AddColumn("Value");

        table.AddRow("Mount Angle", $"{ToDegrees(leg.MountAngle):F1}°");
        table.AddRow("Mount Radius", $"{leg.MountRadius * 1000:F1}mm");
        table.AddRow("Coxa Length", $"{leg.CoxaLength * 1000:F1}mm");
        table.AddRow("Femur Length", $"{leg.FemurLength * 1000:F1}mm");
        table.AddRow("Tibia Length", $"{leg.TibiaLength * 1000:F1}mm");
        table.AddEmptyRow();
        table.AddRow("[yellow]Min Reach[/]", $"{minReach * 1000:F1}mm");
        table.AddRow("[yellow]Max Reach[/]", $"{maxReach * 1000:F1}mm");
        table.AddRow("[yellow]Coxa Sweep[/]", $"{ToDegrees(leg.CoxaLimits.Max - leg.CoxaLimits.Min):F0}° ({ToDegrees(leg.CoxaLimits.Min):F0}° to {ToDegrees(leg.CoxaLimits.Max):F0}°)");

        AnsiConsole.Write(table);

        // Show ASCII workspace visualization
        AnsiConsole.WriteLine();
        ShowWorkspaceVisualization(leg);
    }

    /// <summary>
    /// Show configuration of all legs.
    /// </summary>
    public void ShowLegConfiguration()
    {
        var table = new Table()
            .Border(TableBorder.Rounded)
            .Title("[bold]Leg Configuration[/]")
            .AddColumn("Leg")
            .AddColumn("Mount Angle")
            .AddColumn("Position (mm)")
            .AddColumn("Current Angles (°)");

        foreach (var leg in _body.Legs)
        {
            var footPos = leg.GetFootPosition();
            table.AddRow(
                $"[cyan]{leg.Name}[/]",
                $"{ToDegrees(leg.MountAngle):F0}°",
                $"X:{footPos.X * 1000:F0} Y:{footPos.Y * 1000:F0} Z:{footPos.Z * 1000:F0}",
                $"C:{ToDegrees(leg.CoxaAngle):F0} F:{ToDegrees(leg.FemurAngle):F0} T:{ToDegrees(leg.TibiaAngle):F0}");
        }

        AnsiConsole.Write(table);

        // Show body diagram
        AnsiConsole.WriteLine();
        ShowBodyDiagram();
    }

    /// <summary>
    /// Interactive foot position control.
    /// </summary>
    public void InteractiveFootPosition()
    {
        var legChoice = AnsiConsole.Prompt(
            new SelectionPrompt<string>()
                .Title("Select leg:")
                .AddChoices(LegNames.Select((name, i) => $"{i}: {name}")));

        var legId = int.Parse(legChoice.Split(':')[0]);
        var leg = _body.Legs[legId];

        AnsiConsole.MarkupLine($"\n[bold cyan]Interactive control for {LegNames[legId]}[/]");
        AnsiConsole.MarkupLine("[grey]Use +/- keys with X, Y, Z to adjust position. Q to quit.[/]\n");

        var currentPos = leg.GetFootPosition();
        double x = currentPos.X * 1000;
        double y = currentPos.Y * 1000;
        double z = currentPos.Z * 1000;
        double step = 5.0; // 5mm step

        while (true)
        {
            var target = new Vector3((float)(x / 1000), (float)(y / 1000), (float)(z / 1000));
            var result = leg.InverseKinematics(target);
            
            Console.SetCursorPosition(0, Console.CursorTop);
            AnsiConsole.MarkupLine($"Position: [yellow]X={x:F1}mm[/] [green]Y={y:F1}mm[/] [blue]Z={z:F1}mm[/]  Step: {step}mm");
            
            if (result.HasValue)
            {
                AnsiConsole.MarkupLine($"Angles:   [yellow]C={ToDegrees(result.Value.Coxa):F1}°[/] [green]F={ToDegrees(result.Value.Femur):F1}°[/] [blue]T={ToDegrees(result.Value.Tibia):F1}°[/]");
                AnsiConsole.MarkupLine("[green]Status: Reachable[/]      ");
            }
            else
            {
                AnsiConsole.MarkupLine("Angles:   [grey]N/A[/]                                  ");
                AnsiConsole.MarkupLine("[red]Status: Out of reach[/]  ");
            }

            AnsiConsole.MarkupLine("\n[grey]Keys: X/x=X±  Y/y=Y±  Z/z=Z±  +/-=step  Q=quit[/]");

            var key = Console.ReadKey(true);
            switch (key.Key)
            {
                case ConsoleKey.X:
                    x += key.Modifiers.HasFlag(ConsoleModifiers.Shift) ? step : -step;
                    break;
                case ConsoleKey.Y:
                    y += key.Modifiers.HasFlag(ConsoleModifiers.Shift) ? step : -step;
                    break;
                case ConsoleKey.Z:
                    z += key.Modifiers.HasFlag(ConsoleModifiers.Shift) ? step : -step;
                    break;
                case ConsoleKey.OemPlus:
                case ConsoleKey.Add:
                    step = Math.Min(step * 2, 50);
                    break;
                case ConsoleKey.OemMinus:
                case ConsoleKey.Subtract:
                    step = Math.Max(step / 2, 1);
                    break;
                case ConsoleKey.Q:
                    return;
            }

            // Move cursor up to overwrite
            Console.SetCursorPosition(0, Console.CursorTop - 5);
        }
    }

    // Helper methods

    private void AnimateSingleStep(HexapodLeg leg, Vector3 startPos, float stepDirX, float stepDirY, float stepHeight)
    {
        var steps = 40;
        
        AnsiConsole.Status()
            .Spinner(Spinner.Known.Dots)
            .Start("Animating step...", ctx =>
            {
                for (int i = 0; i <= steps; i++)
                {
                    var t = (double)i / steps;
                    Vector3 pos;
                    string phase;

                    if (t < 0.25) // Lift
                    {
                        var p = t / 0.25;
                        var height = stepHeight * (float)Math.Sin(p * Math.PI / 2);
                        var xOff = -stepDirX * (float)p;
                        var yOff = -stepDirY * (float)p;
                        pos = new Vector3(startPos.X + xOff, startPos.Y + yOff, startPos.Z + height);
                        phase = "Lifting";
                    }
                    else if (t < 0.5) // Forward
                    {
                        var p = (t - 0.25) / 0.25;
                        var xOff = -stepDirX + 2 * stepDirX * (float)p;
                        var yOff = -stepDirY + 2 * stepDirY * (float)p;
                        pos = new Vector3(startPos.X + xOff, startPos.Y + yOff, startPos.Z + stepHeight);
                        phase = "Swinging forward";
                    }
                    else if (t < 0.75) // Down
                    {
                        var p = (t - 0.5) / 0.25;
                        var height = stepHeight * (float)Math.Cos(p * Math.PI / 2);
                        pos = new Vector3(startPos.X + stepDirX, startPos.Y + stepDirY, startPos.Z + height);
                        phase = "Putting down";
                    }
                    else // Push back
                    {
                        var p = (t - 0.75) / 0.25;
                        var xOff = stepDirX - 2 * stepDirX * (float)p;
                        var yOff = stepDirY - 2 * stepDirY * (float)p;
                        pos = new Vector3(startPos.X + xOff, startPos.Y + yOff, startPos.Z);
                        phase = "Pushing back (stance)";
                    }

                    var angles = leg.InverseKinematics(pos);
                    if (angles.HasValue)
                    {
                        ctx.Status($"{phase}: {t * 100:F0}% | " +
                                 $"Pos: ({pos.X * 1000:F0}, {pos.Y * 1000:F0}, {pos.Z * 1000:F0})mm | " +
                                 $"C:{ToDegrees(angles.Value.Coxa):F0}° F:{ToDegrees(angles.Value.Femur):F0}° T:{ToDegrees(angles.Value.Tibia):F0}°");
                    }
                    Thread.Sleep(50);
                }
            });

        AnsiConsole.MarkupLine("[green]✓[/] Step animation complete");
    }

    private void AddPhaseRow(Table table, string phase, Vector3 pos, (double Coxa, double Femur, double Tibia)? angles)
    {
        var posStr = $"X:{pos.X * 1000:F1} Y:{pos.Y * 1000:F1} Z:{pos.Z * 1000:F1}";
        
        if (angles.HasValue)
        {
            table.AddRow(
                phase,
                posStr,
                $"C:{ToDegrees(angles.Value.Coxa):F1} F:{ToDegrees(angles.Value.Femur):F1} T:{ToDegrees(angles.Value.Tibia):F1}",
                "[green]✓ Reachable[/]");
        }
        else
        {
            table.AddRow(phase, posStr, "[grey]N/A[/]", "[red]✗ Unreachable[/]");
        }
    }

    private void ShowGaitPhaseTable(IGaitGenerator gait)
    {
        var table = new Table()
            .Border(TableBorder.Rounded)
            .AddColumn("Cycle %");

        foreach (var name in LegNames)
        {
            table.AddColumn(name[..2]);
        }

        for (int p = 0; p <= 100; p += 10)
        {
            var phase = p / 100.0;
            var legPhases = gait.GetLegPhases(phase);
            
            var row = new List<string> { $"{p}%" };
            foreach (var lp in legPhases)
            {
                row.Add(lp.IsSwingPhase ? "[red]Air[/]" : "[green]Gnd[/]");
            }
            table.AddRow(row.ToArray());
        }

        AnsiConsole.Write(table);
    }

    private void ShowGaitTimingDiagram(IGaitGenerator gait)
    {
        AnsiConsole.MarkupLine($"[bold]{gait.GaitType}[/] (Duty: {gait.DutyFactor:P0})");
        
        const int width = 40;
        for (int leg = 0; leg < 6; leg++)
        {
            var sb = new StringBuilder();
            sb.Append($"{LegNames[leg],-14} ");
            
            for (int i = 0; i < width; i++)
            {
                var phase = (double)i / width;
                var legPhases = gait.GetLegPhases(phase);
                sb.Append(legPhases[leg].IsSwingPhase ? "[red]▀[/]" : "[green]█[/]");
            }
            
            AnsiConsole.MarkupLine(sb.ToString());
        }
    }

    private void ShowWorkspaceVisualization(HexapodLeg leg)
    {
        const int size = 21;
        var center = size / 2;
        var scale = 0.005; // 5mm per character

        var grid = new char[size, size];
        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
                grid[i, j] = '·';

        // Test points in a grid pattern
        for (int yi = 0; yi < size; yi++)
        {
            for (int xi = 0; xi < size; xi++)
            {
                var x = (float)(leg.MountRadius * Math.Cos(leg.MountAngle) + (xi - center) * scale);
                var y = (float)(leg.MountRadius * Math.Sin(leg.MountAngle) + (yi - center) * scale);
                var z = -0.08f; // Default body height

                var target = new Vector3((float)x, (float)y, (float)z);
                var result = leg.InverseKinematics(target);
                
                if (result.HasValue)
                {
                    grid[yi, xi] = '█';
                }
            }
        }

        // Mark mount point
        grid[center, center] = 'O';

        AnsiConsole.MarkupLine("[grey]Top-down view (X→, Y↑)[/]");
        for (int yi = size - 1; yi >= 0; yi--)
        {
            var line = new StringBuilder();
            for (int xi = 0; xi < size; xi++)
            {
                var c = grid[yi, xi];
                line.Append(c == '█' ? "[green]█[/]" : c == 'O' ? "[yellow]O[/]" : "[grey]·[/]");
            }
            AnsiConsole.MarkupLine(line.ToString());
        }
    }

    private void ShowBodyDiagram()
    {
        AnsiConsole.MarkupLine("[grey]Body layout (top view):[/]");
        AnsiConsole.MarkupLine(@"
              [cyan]FL[/]           [cyan]FR[/]
               \           /
                \    ▲    /
           [cyan]ML[/] ──[yellow]■■■[/]── [cyan]MR[/]
                /    │    \
               /     │     \
              [cyan]RL[/]           [cyan]RR[/]
        
        [grey]▲ = Forward direction[/]
        ");
    }

    private void ShowJointAngles(string label, (double Coxa, double Femur, double Tibia) angles)
    {
        AnsiConsole.MarkupLine($"[grey]{label}:[/] Coxa={ToDegrees(angles.Coxa):F1}° Femur={ToDegrees(angles.Femur):F1}° Tibia={ToDegrees(angles.Tibia):F1}°");
    }

    private static double ToDegrees(double radians) => radians * 180.0 / Math.PI;

    private static string FormatRating(double value)
    {
        var bars = (int)(value * 5);
        var color = value >= 0.8 ? "green" : value >= 0.6 ? "yellow" : "red";
        return $"[{color}]{new string('█', bars)}{new string('░', 5 - bars)}[/] {value:P0}";
    }
}
