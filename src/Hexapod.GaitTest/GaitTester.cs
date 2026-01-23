using System.Numerics;
using System.Text;
using Hexapod.Core.Enums;
using Hexapod.Movement.Gait;
using Hexapod.Movement.Kinematics;
using Hexapod.Movement.Servo;
using Spectre.Console;

namespace Hexapod.GaitTest;

/// <summary>
/// Interactive tester for inverse kinematics and gait patterns.
/// Supports both simulation mode and physical execution via servo controller.
/// </summary>
public class GaitTester
{
    private readonly HexapodBody _body;
    private readonly PololuMaestroServoController? _controller;
    private bool _hardwareEnabled;

    private static readonly string[] LegNames = 
    { 
        "Front Right", "Middle Right", "Rear Right",
        "Rear Left", "Middle Left", "Front Left"
    };

    // Default parameters
    private const double DefaultStepLength = 0.06;  // 60mm
    private const double DefaultStepHeight = 0.03;  // 30mm
    private const double DefaultBodyHeight = 0.08;  // 80mm

    public GaitTester(HexapodBody body, PololuMaestroServoController? controller = null)
    {
        _body = body;
        _controller = controller;
        _hardwareEnabled = false;
    }

    /// <summary>
    /// Gets or sets whether hardware execution is enabled.
    /// </summary>
    public bool HardwareEnabled
    {
        get => _hardwareEnabled && _controller != null;
        set => _hardwareEnabled = value && _controller != null;
    }

    /// <summary>
    /// Check if hardware controller is connected.
    /// </summary>
    public bool HasHardware => _controller != null;

    /// <summary>
    /// Toggle hardware execution mode.
    /// </summary>
    public void ToggleHardwareMode()
    {
        if (_controller == null)
        {
            AnsiConsole.MarkupLine("[red]No hardware controller connected[/]");
            return;
        }

        _hardwareEnabled = !_hardwareEnabled;
        AnsiConsole.MarkupLine(_hardwareEnabled 
            ? "[green]✓ Hardware execution ENABLED - movements will be sent to servos[/]" 
            : "[yellow]⚠ Hardware execution DISABLED - simulation only[/]");
    }

    /// <summary>
    /// Center all servos to home position.
    /// </summary>
    public void CenterAllServos()
    {
        if (_controller == null)
        {
            AnsiConsole.MarkupLine("[yellow]No hardware controller connected (simulation mode)[/]");
            return;
        }

        _controller.GoHome();
        AnsiConsole.MarkupLine("[green]✓[/] All servos centered to home position");
    }

    /// <summary>
    /// Disable all servos (turn off PWM).
    /// </summary>
    public void DisableAllServos()
    {
        if (_controller == null)
        {
            AnsiConsole.MarkupLine("[yellow]No hardware controller connected (simulation mode)[/]");
            return;
        }

        _controller.DisableAll();
        _hardwareEnabled = false;
        AnsiConsole.MarkupLine("[yellow]⚠[/] All servos disabled");
    }

    /// <summary>
    /// Enable all servos.
    /// </summary>
    public void EnableAllServos()
    {
        if (_controller == null)
        {
            AnsiConsole.MarkupLine("[yellow]No hardware controller connected (simulation mode)[/]");
            return;
        }

        _controller.EnableAll();
        AnsiConsole.MarkupLine("[green]✓[/] All servos enabled");
    }

    /// <summary>
    /// Execute joint angles on physical hardware.
    /// </summary>
    private void ExecuteJointAngles(int legId, double coxa, double femur, double tibia)
    {
        if (!HardwareEnabled || _controller == null)
            return;

        // Calculate PWM values and send to controller
        // Channel mapping: legId * 3 + jointIndex
        var coxaChannel = legId * 3 + 0;
        var femurChannel = legId * 3 + 1;
        var tibiaChannel = legId * 3 + 2;

        // Convert radians to PWM microseconds
        // Assuming center = 1500us, range = 500-2500us for -90 to +90 degrees
        var coxaPwm = AngleToPwm(coxa);
        var femurPwm = AngleToPwm(femur);
        var tibiaPwm = AngleToPwm(tibia);

        _controller.SetChannelPosition(coxaChannel, coxaPwm);
        _controller.SetChannelPosition(femurChannel, femurPwm);
        _controller.SetChannelPosition(tibiaChannel, tibiaPwm);
    }

    /// <summary>
    /// Execute joint angles for all legs on physical hardware.
    /// </summary>
    private void ExecuteAllLegAngles((double Coxa, double Femur, double Tibia)[] legAngles)
    {
        if (!HardwareEnabled || _controller == null)
            return;

        for (int legId = 0; legId < 6; legId++)
        {
            if (legAngles[legId] != default)
            {
                ExecuteJointAngles(legId, legAngles[legId].Coxa, legAngles[legId].Femur, legAngles[legId].Tibia);
            }
        }
    }

    /// <summary>
    /// Convert angle in radians to PWM microseconds.
    /// </summary>
    private static double AngleToPwm(double angleRadians)
    {
        // Convert to degrees
        var angleDegrees = angleRadians * 180.0 / Math.PI;
        
        // Map -90 to +90 degrees to 500-2500 microseconds
        // Center (0°) = 1500us
        var normalized = (angleDegrees + 90.0) / 180.0;
        normalized = Math.Clamp(normalized, 0.0, 1.0);
        
        return 500.0 + normalized * 2000.0;
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
                        
                        // Execute on hardware if enabled
                        ExecuteJointAngles(legId, angles.Value.Coxa, angles.Value.Femur, angles.Value.Tibia);
                    }
                    Thread.Sleep(HardwareEnabled ? 100 : 50);
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
                        
                        // Execute on hardware if enabled
                        ExecuteJointAngles(legId, angles.Value.Coxa, angles.Value.Femur, angles.Value.Tibia);
                    }
                    Thread.Sleep(HardwareEnabled ? 100 : 50);
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

        // Calculate leg workspace bounds
        var mountX = leg.MountRadius * Math.Cos(leg.MountAngle);
        var mountY = leg.MountRadius * Math.Sin(leg.MountAngle);
        var maxReach = leg.FemurLength + leg.TibiaLength;
        var minReach = Math.Abs(leg.FemurLength - leg.TibiaLength);
        
        // Show initial valid ranges based on leg geometry
        var totalMaxReach = leg.CoxaLength + maxReach;
        var totalMinReach = leg.CoxaLength + minReach;
        
        AnsiConsole.MarkupLine($"[grey]Current position: X={currentPos.X * 1000:F1}mm, Y={currentPos.Y * 1000:F1}mm, Z={currentPos.Z * 1000:F1}mm[/]");
        AnsiConsole.MarkupLine($"[grey]Mount point: X={mountX * 1000:F1}mm, Y={mountY * 1000:F1}mm[/]");
        AnsiConsole.MarkupLine($"[grey]Geometric reach from mount: {totalMinReach * 1000:F1}mm to {totalMaxReach * 1000:F1}mm[/]");
        AnsiConsole.MarkupLine($"[grey]Joint limits: Coxa {ToDegrees(leg.CoxaLimits.Min):F0}° to {ToDegrees(leg.CoxaLimits.Max):F0}°, Femur {ToDegrees(leg.FemurLimits.Min):F0}° to {ToDegrees(leg.FemurLimits.Max):F0}°, Tibia {ToDegrees(leg.TibiaLimits.Min):F0}° to {ToDegrees(leg.TibiaLimits.Max):F0}°[/]\n");

        // Get X coordinate - calculate valid range considering joint limits
        var (xRangeMin, xRangeMax) = CalculateValidXRange(leg);
        
        if (double.IsNaN(xRangeMin))
        {
            AnsiConsole.MarkupLine("[red]Warning: Could not calculate valid X range[/]");
            xRangeMin = mountX + totalMinReach * Math.Cos(leg.MountAngle);
            xRangeMax = mountX + totalMaxReach * Math.Cos(leg.MountAngle);
        }
        
        AnsiConsole.MarkupLine($"[cyan]Valid X range (with joint limits): {xRangeMin * 1000:F1}mm to {xRangeMax * 1000:F1}mm[/]");
        
        var x = AnsiConsole.Prompt(
            new TextPrompt<double>($"Target X (mm) [grey](default {currentPos.X * 1000:F1})[/]:")
                .DefaultValue(currentPos.X * 1000)) / 1000.0;

        // After X is entered, calculate valid Y range considering joint limits
        var (yMin, yMax) = CalculateValidYRange(leg, x, totalMinReach, totalMaxReach);
        if (double.IsNaN(yMin))
        {
            AnsiConsole.MarkupLine($"\n[yellow]Warning: X={x * 1000:F1}mm may be out of reachable range[/]");
            yMin = mountY - totalMaxReach;
            yMax = mountY + totalMaxReach;
        }
        AnsiConsole.MarkupLine($"\n[cyan]Given X={x * 1000:F1}mm, valid Y range (with joint limits): {yMin * 1000:F1}mm to {yMax * 1000:F1}mm[/]");
        
        var y = AnsiConsole.Prompt(
            new TextPrompt<double>($"Target Y (mm) [grey](default {currentPos.Y * 1000:F1})[/]:")
                .DefaultValue(currentPos.Y * 1000)) / 1000.0;

        // After Y is entered, calculate valid Z range considering joint limits
        var (zMin, zMax) = CalculateValidZRange(leg, x, y, maxReach);
        if (double.IsNaN(zMin))
        {
            AnsiConsole.MarkupLine($"\n[yellow]Warning: Position X={x * 1000:F1}mm, Y={y * 1000:F1}mm may be out of reachable range[/]");
            zMin = -maxReach;
            zMax = maxReach;
        }
        AnsiConsole.MarkupLine($"\n[cyan]Given X={x * 1000:F1}mm, Y={y * 1000:F1}mm, valid Z range (with joint limits): {zMin * 1000:F1}mm to {zMax * 1000:F1}mm[/]");
        
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
            
            // Calculate detailed reachability analysis
            var mountOffset = new Vector3(
                (float)(leg.MountRadius * Math.Cos(leg.MountAngle)),
                (float)(leg.MountRadius * Math.Sin(leg.MountAngle)),
                0);
            
            // Distance from mount point in XY plane
            var dx = target.X - mountOffset.X;
            var dy = target.Y - mountOffset.Y;
            var dz = target.Z;
            var horizontalDistFromMount = Math.Sqrt(dx * dx + dy * dy);
            
            // Calculate horizontal distance from femur joint (after coxa)
            var horizontalDistFromFemur = horizontalDistFromMount - leg.CoxaLength;
            
            // Distance from femur joint to target (what femur+tibia need to reach)
            var L = Math.Sqrt(horizontalDistFromFemur * horizontalDistFromFemur + dz * dz);
            
            var legMaxReach = leg.FemurLength + leg.TibiaLength;
            var legMinReach = Math.Abs(leg.FemurLength - leg.TibiaLength);
            
            // Calculate coxa angle to check limits
            var coxaAngle = Math.Atan2(dy, dx) - leg.MountAngle;
            
            // Calculate femur and tibia angles (same as IK algorithm)
            double? femurAngle = null;
            double? tibiaAngle = null;
            bool geometricallyReachable = L <= legMaxReach && L >= legMinReach && horizontalDistFromFemur >= 0;
            
            if (geometricallyReachable)
            {
                // Law of cosines for tibia angle
                var cosGamma = (leg.FemurLength * leg.FemurLength + leg.TibiaLength * leg.TibiaLength - L * L) / 
                               (2 * leg.FemurLength * leg.TibiaLength);
                cosGamma = Math.Clamp(cosGamma, -1.0, 1.0);
                tibiaAngle = Math.PI - Math.Acos(cosGamma);

                // Calculate femur angle
                var alpha = Math.Atan2(dz, horizontalDistFromFemur);
                var cosBeta = (leg.FemurLength * leg.FemurLength + L * L - leg.TibiaLength * leg.TibiaLength) / 
                              (2 * leg.FemurLength * L);
                cosBeta = Math.Clamp(cosBeta, -1.0, 1.0);
                var beta = Math.Acos(cosBeta);
                femurAngle = alpha + beta;
            }
            
            var table = new Table()
                .Border(TableBorder.Rounded)
                .Title("[red]Reachability Analysis[/]")
                .AddColumn("Check")
                .AddColumn("Value")
                .AddColumn("Valid Range (given other coords)")
                .AddColumn("Status");

            // Calculate valid X range at current Y and Z (using same method as input prompts)
            var (validXMin, validXMax) = CalculateValidXRangeAtYZ(leg, target.Y, target.Z);
            var xInRange = !double.IsNaN(validXMin) && target.X >= validXMin && target.X <= validXMax;
            
            table.AddRow("X position", 
                $"{target.X * 1000:F1}mm",
                double.IsNaN(validXMin) ? "[grey]No valid X at this Y,Z[/]" : $"{validXMin * 1000:F1}mm to {validXMax * 1000:F1}mm",
                xInRange ? "[green]OK[/]" : "[red]OUT OF RANGE[/]");
            
            // Calculate valid Y range at current X and Z
            var (validYMin, validYMax) = CalculateValidYRangeAtXZ(leg, target.X, target.Z);
            var yInRange = !double.IsNaN(validYMin) && target.Y >= validYMin && target.Y <= validYMax;
            
            table.AddRow("Y position", 
                $"{target.Y * 1000:F1}mm",
                double.IsNaN(validYMin) ? "[grey]No valid Y at this X,Z[/]" : $"{validYMin * 1000:F1}mm to {validYMax * 1000:F1}mm",
                yInRange ? "[green]OK[/]" : "[red]OUT OF RANGE[/]");
            
            // Calculate valid Z range at current X and Y (using existing method)
            var (validZMin, validZMax) = CalculateValidZRange(leg, target.X, target.Y, legMaxReach);
            var zInRange = !double.IsNaN(validZMin) && target.Z >= validZMin && target.Z <= validZMax;
            
            table.AddRow("Z position", 
                $"{target.Z * 1000:F1}mm",
                double.IsNaN(validZMin) ? "[grey]No valid Z at this X,Y[/]" : $"{validZMin * 1000:F1}mm to {validZMax * 1000:F1}mm",
                zInRange ? "[green]OK[/]" : "[red]OUT OF RANGE[/]");
            
            // Add joint angle checks
            table.AddEmptyRow();
            table.AddRow("[bold]Joint Angles[/]", "", "", "");
            
            table.AddRow("Coxa angle",
                $"{ToDegrees(coxaAngle):F1}°",
                $"{ToDegrees(leg.CoxaLimits.Min):F0}° to {ToDegrees(leg.CoxaLimits.Max):F0}°",
                IsWithinLimits(coxaAngle, leg.CoxaLimits) ? "[green]OK[/]" : "[red]LIMIT EXCEEDED[/]");
            
            if (femurAngle.HasValue)
            {
                table.AddRow("Femur angle",
                    $"{ToDegrees(femurAngle.Value):F1}°",
                    $"{ToDegrees(leg.FemurLimits.Min):F0}° to {ToDegrees(leg.FemurLimits.Max):F0}°",
                    IsWithinLimits(femurAngle.Value, leg.FemurLimits) ? "[green]OK[/]" : "[red]LIMIT EXCEEDED[/]");
            }
            else
            {
                table.AddRow("Femur angle", "[grey]N/A[/]", 
                    $"{ToDegrees(leg.FemurLimits.Min):F0}° to {ToDegrees(leg.FemurLimits.Max):F0}°", 
                    "[grey]N/A[/]");
            }
            
            if (tibiaAngle.HasValue)
            {
                table.AddRow("Tibia angle",
                    $"{ToDegrees(tibiaAngle.Value):F1}°",
                    $"{ToDegrees(leg.TibiaLimits.Min):F0}° to {ToDegrees(leg.TibiaLimits.Max):F0}°",
                    IsWithinLimits(tibiaAngle.Value, leg.TibiaLimits) ? "[green]OK[/]" : "[red]LIMIT EXCEEDED[/]");
            }
            else
            {
                table.AddRow("Tibia angle", "[grey]N/A[/]", 
                    $"{ToDegrees(leg.TibiaLimits.Min):F0}° to {ToDegrees(leg.TibiaLimits.Max):F0}°", 
                    "[grey]N/A[/]");
            }
            
            AnsiConsole.Write(table);
            
            // Additional diagnostic information
            AnsiConsole.MarkupLine($"\n[yellow]Diagnostic Details:[/]");
            AnsiConsole.MarkupLine($"  Distance from mount point (XY): [cyan]{horizontalDistFromMount * 1000:F1}mm[/]");
            AnsiConsole.MarkupLine($"  Distance from femur joint (XY): [cyan]{horizontalDistFromFemur * 1000:F1}mm[/]");
            AnsiConsole.MarkupLine($"  Distance to target (3D from femur): [cyan]{L * 1000:F1}mm[/]");
            AnsiConsole.MarkupLine($"  Femur+Tibia reach range: [cyan]{legMinReach * 1000:F1}mm to {legMaxReach * 1000:F1}mm[/]");
            
            // Identify specific failure reason
            AnsiConsole.MarkupLine($"\n[yellow]Failure Reason:[/]");
            bool hasFailure = false;
            
            if (horizontalDistFromFemur < 0)
            {
                AnsiConsole.MarkupLine($"  [red]• Target is too close to body center (inside coxa reach)[/]");
                hasFailure = true;
            }
            if (L > legMaxReach)
            {
                AnsiConsole.MarkupLine($"  [red]• Target is too far: {L * 1000:F1}mm > max reach {legMaxReach * 1000:F1}mm[/]");
                hasFailure = true;
            }
            if (L < legMinReach && horizontalDistFromFemur >= 0)
            {
                AnsiConsole.MarkupLine($"  [red]• Target is too close: {L * 1000:F1}mm < min reach {legMinReach * 1000:F1}mm[/]");
                hasFailure = true;
            }
            if (!IsWithinLimits(coxaAngle, leg.CoxaLimits))
            {
                AnsiConsole.MarkupLine($"  [red]• Coxa angle {ToDegrees(coxaAngle):F1}° exceeds limits ({ToDegrees(leg.CoxaLimits.Min):F0}° to {ToDegrees(leg.CoxaLimits.Max):F0}°)[/]");
                hasFailure = true;
            }
            if (femurAngle.HasValue && !IsWithinLimits(femurAngle.Value, leg.FemurLimits))
            {
                AnsiConsole.MarkupLine($"  [red]• Femur angle {ToDegrees(femurAngle.Value):F1}° exceeds limits ({ToDegrees(leg.FemurLimits.Min):F0}° to {ToDegrees(leg.FemurLimits.Max):F0}°)[/]");
                hasFailure = true;
            }
            if (tibiaAngle.HasValue && !IsWithinLimits(tibiaAngle.Value, leg.TibiaLimits))
            {
                AnsiConsole.MarkupLine($"  [red]• Tibia angle {ToDegrees(tibiaAngle.Value):F1}° exceeds limits ({ToDegrees(leg.TibiaLimits.Min):F0}° to {ToDegrees(leg.TibiaLimits.Max):F0}°)[/]");
                hasFailure = true;
            }
            
            if (!hasFailure)
            {
                AnsiConsole.MarkupLine($"  [yellow]• Unknown failure - check IK algorithm[/]");
            }
        }
    }

    private static bool IsWithinLimits(double angle, (double Min, double Max) limits)
    {
        return angle >= limits.Min && angle <= limits.Max;
    }

    /// <summary>
    /// Check if a position is reachable considering all joint limits.
    /// Returns true if the position can be reached with valid joint angles.
    /// </summary>
    private static bool IsPositionReachable(HexapodLeg leg, double x, double y, double z)
    {
        var mountX = leg.MountRadius * Math.Cos(leg.MountAngle);
        var mountY = leg.MountRadius * Math.Sin(leg.MountAngle);
        
        var dx = x - mountX;
        var dy = y - mountY;
        
        // Check coxa angle
        var coxaAngle = Math.Atan2(dy, dx) - leg.MountAngle;
        if (!IsWithinLimits(coxaAngle, leg.CoxaLimits))
            return false;
        
        // Calculate horizontal distance from femur joint
        var horizontalDist = Math.Sqrt(dx * dx + dy * dy);
        var horizontalDistFromFemur = horizontalDist - leg.CoxaLength;
        
        if (horizontalDistFromFemur < 0)
            return false;
        
        // Distance from femur joint to target
        var L = Math.Sqrt(horizontalDistFromFemur * horizontalDistFromFemur + z * z);
        
        var maxReach = leg.FemurLength + leg.TibiaLength;
        var minReach = Math.Abs(leg.FemurLength - leg.TibiaLength);
        
        if (L > maxReach || L < minReach)
            return false;
        
        // Calculate tibia angle
        var cosGamma = (leg.FemurLength * leg.FemurLength + leg.TibiaLength * leg.TibiaLength - L * L) / 
                       (2 * leg.FemurLength * leg.TibiaLength);
        cosGamma = Math.Clamp(cosGamma, -1.0, 1.0);
        var tibiaAngle = Math.PI - Math.Acos(cosGamma);
        
        if (!IsWithinLimits(tibiaAngle, leg.TibiaLimits))
            return false;
        
        // Calculate femur angle
        var alpha = Math.Atan2(z, horizontalDistFromFemur);
        var cosBeta = (leg.FemurLength * leg.FemurLength + L * L - leg.TibiaLength * leg.TibiaLength) / 
                      (2 * leg.FemurLength * L);
        cosBeta = Math.Clamp(cosBeta, -1.0, 1.0);
        var beta = Math.Acos(cosBeta);
        var femurAngle = alpha + beta;
        
        if (!IsWithinLimits(femurAngle, leg.FemurLimits))
            return false;
        
        return true;
    }

    /// <summary>
    /// Calculate valid X range considering all joint limits.
    /// </summary>
    private static (double Min, double Max) CalculateValidXRange(HexapodLeg leg)
    {
        var mountX = leg.MountRadius * Math.Cos(leg.MountAngle);
        var mountY = leg.MountRadius * Math.Sin(leg.MountAngle);
        var maxReach = leg.FemurLength + leg.TibiaLength;
        var minReach = Math.Abs(leg.FemurLength - leg.TibiaLength);
        var totalMaxReach = leg.CoxaLength + maxReach;
        var totalMinReach = leg.CoxaLength + minReach;
        
        var xValues = new List<double>();
        
        // Sample the workspace considering all constraints
        var minAngle = leg.MountAngle + leg.CoxaLimits.Min;
        var maxAngle = leg.MountAngle + leg.CoxaLimits.Max;
        
        for (double reach = totalMinReach; reach <= totalMaxReach; reach += (totalMaxReach - totalMinReach) / 20)
        {
            for (double angle = minAngle; angle <= maxAngle; angle += (maxAngle - minAngle) / 40)
            {
                var testX = mountX + reach * Math.Cos(angle);
                var testY = mountY + reach * Math.Sin(angle);
                
                // Test at Z=0 and check if reachable with joint limits
                if (IsPositionReachable(leg, testX, testY, 0))
                {
                    xValues.Add(testX);
                }
            }
        }
        
        if (xValues.Count == 0)
            return (double.NaN, double.NaN);
        
        return (xValues.Min(), xValues.Max());
    }

    /// <summary>
    /// Calculate valid Y range given a fixed X coordinate, considering all joint limits.
    /// </summary>
    private static (double Min, double Max) CalculateValidYRange(HexapodLeg leg, double x, double minReach, double maxReach)
    {
        var mountX = leg.MountRadius * Math.Cos(leg.MountAngle);
        var mountY = leg.MountRadius * Math.Sin(leg.MountAngle);
        
        var minAngle = leg.MountAngle + leg.CoxaLimits.Min;
        var maxAngle = leg.MountAngle + leg.CoxaLimits.Max;
        
        var yValues = new List<double>();
        
        // Sample points along the reachable arc at different reaches
        for (double reach = minReach; reach <= maxReach; reach += (maxReach - minReach) / 20)
        {
            for (double angle = minAngle; angle <= maxAngle; angle += (maxAngle - minAngle) / 40)
            {
                var testX = mountX + reach * Math.Cos(angle);
                if (Math.Abs(testX - x) < 0.003) // Within 3mm tolerance
                {
                    var testY = mountY + reach * Math.Sin(angle);
                    
                    // Check if this position is reachable with joint limits (at Z=0)
                    if (IsPositionReachable(leg, x, testY, 0))
                    {
                        yValues.Add(testY);
                    }
                }
            }
        }
        
        if (yValues.Count == 0)
        {
            return (double.NaN, double.NaN);
        }
        
        return (yValues.Min(), yValues.Max());
    }

    /// <summary>
    /// Calculate valid Z range given fixed X and Y coordinates, considering all joint limits.
    /// </summary>
    private static (double Min, double Max) CalculateValidZRange(HexapodLeg leg, double x, double y, double maxReach)
    {
        var mountX = leg.MountRadius * Math.Cos(leg.MountAngle);
        var mountY = leg.MountRadius * Math.Sin(leg.MountAngle);
        
        // Horizontal distance from mount point
        var dx = x - mountX;
        var dy = y - mountY;
        var horizontalDist = Math.Sqrt(dx * dx + dy * dy);
        
        // Distance from femur joint (after coxa)
        var horizontalDistFromFemur = horizontalDist - leg.CoxaLength;
        
        if (horizontalDistFromFemur < 0)
        {
            return (double.NaN, double.NaN);
        }
        
        var minReach = Math.Abs(leg.FemurLength - leg.TibiaLength);
        
        if (horizontalDistFromFemur > maxReach)
        {
            return (double.NaN, double.NaN);
        }
        
        // Calculate geometric Z bounds
        var geometricMaxZ = Math.Sqrt(maxReach * maxReach - horizontalDistFromFemur * horizontalDistFromFemur);
        var geometricMinZ = -geometricMaxZ;
        
        // Now find actual bounds considering joint limits by sampling
        var validZValues = new List<double>();
        var step = (geometricMaxZ - geometricMinZ) / 50;
        
        for (double z = geometricMinZ; z <= geometricMaxZ; z += step)
        {
            if (IsPositionReachable(leg, x, y, z))
            {
                validZValues.Add(z);
            }
        }
        
        if (validZValues.Count == 0)
        {
            return (double.NaN, double.NaN);
        }
        
        return (validZValues.Min(), validZValues.Max());
    }

    /// <summary>
    /// Calculate valid X range given fixed Y and Z coordinates, considering all joint limits.
    /// </summary>
    private static (double Min, double Max) CalculateValidXRangeAtYZ(HexapodLeg leg, double y, double z)
    {
        var mountX = leg.MountRadius * Math.Cos(leg.MountAngle);
        var mountY = leg.MountRadius * Math.Sin(leg.MountAngle);
        var maxReach = leg.FemurLength + leg.TibiaLength;
        var minReach = Math.Abs(leg.FemurLength - leg.TibiaLength);
        var totalMaxReach = leg.CoxaLength + maxReach;
        var totalMinReach = leg.CoxaLength + minReach;
        
        var xValues = new List<double>();
        
        // Calculate approximate X range to search
        var xSearchMin = mountX - totalMaxReach;
        var xSearchMax = mountX + totalMaxReach;
        var step = (xSearchMax - xSearchMin) / 100;
        
        for (double x = xSearchMin; x <= xSearchMax; x += step)
        {
            if (IsPositionReachable(leg, x, y, z))
            {
                xValues.Add(x);
            }
        }
        
        if (xValues.Count == 0)
        {
            return (double.NaN, double.NaN);
        }
        
        return (xValues.Min(), xValues.Max());
    }

    /// <summary>
    /// Calculate valid Y range given fixed X and Z coordinates, considering all joint limits.
    /// </summary>
    private static (double Min, double Max) CalculateValidYRangeAtXZ(HexapodLeg leg, double x, double z)
    {
        var mountX = leg.MountRadius * Math.Cos(leg.MountAngle);
        var mountY = leg.MountRadius * Math.Sin(leg.MountAngle);
        var maxReach = leg.FemurLength + leg.TibiaLength;
        var minReach = Math.Abs(leg.FemurLength - leg.TibiaLength);
        var totalMaxReach = leg.CoxaLength + maxReach;
        
        var yValues = new List<double>();
        
        // Calculate approximate Y range to search
        var ySearchMin = mountY - totalMaxReach;
        var ySearchMax = mountY + totalMaxReach;
        var step = (ySearchMax - ySearchMin) / 100;
        
        for (double y = ySearchMin; y <= ySearchMax; y += step)
        {
            if (IsPositionReachable(leg, x, y, z))
            {
                yValues.Add(y);
            }
        }
        
        if (yValues.Count == 0)
        {
            return (double.NaN, double.NaN);
        }
        
        return (yValues.Min(), yValues.Max());
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

    /// <summary>
    /// Test joint angle reachability by moving servos to specified angles.
    /// </summary>
    public void TestJointAngleReachability()
    {
        var leg = _body.Legs[0]; // Use Leg 0 for limits reference

        AnsiConsole.MarkupLine("[bold cyan]Joint Angle Reachability Test[/]\n");
        AnsiConsole.MarkupLine("[grey]Joint limits from HexapodLeg model:[/]");
        AnsiConsole.MarkupLine($"  Coxa:  [yellow]{ToDegrees(leg.CoxaLimits.Min):F0}°[/] to [yellow]{ToDegrees(leg.CoxaLimits.Max):F0}°[/]");
        AnsiConsole.MarkupLine($"  Femur: [yellow]{ToDegrees(leg.FemurLimits.Min):F0}°[/] to [yellow]{ToDegrees(leg.FemurLimits.Max):F0}°[/]");
        AnsiConsole.MarkupLine($"  Tibia: [yellow]{ToDegrees(leg.TibiaLimits.Min):F0}°[/] to [yellow]{ToDegrees(leg.TibiaLimits.Max):F0}°[/]\n");

        var testMode = AnsiConsole.Prompt(
            new SelectionPrompt<string>()
                .Title("Select test mode:")
                .AddChoices(
                    "Test individual joint",
                    "Test all three joints together",
                    "Quick preset tests"));

        switch (testMode)
        {
            case "Test individual joint":
                TestIndividualJoint(leg);
                break;
            case "Test all three joints together":
                TestAllJointsTogether(leg);
                break;
            case "Quick preset tests":
                RunPresetTests(leg);
                break;
        }
    }

    private void TestIndividualJoint(HexapodLeg leg)
    {
        var jointChoice = AnsiConsole.Prompt(
            new SelectionPrompt<string>()
                .Title("Select joint to test:")
                .AddChoices("Coxa", "Femur", "Tibia"));

        var (limits, currentAngle) = jointChoice switch
        {
            "Coxa" => (leg.CoxaLimits, leg.CoxaAngle),
            "Femur" => (leg.FemurLimits, leg.FemurAngle),
            "Tibia" => (leg.TibiaLimits, leg.TibiaAngle),
            _ => ((Min: 0.0, Max: 0.0), 0.0)
        };

        var minDeg = ToDegrees(limits.Min);
        var maxDeg = ToDegrees(limits.Max);

        AnsiConsole.MarkupLine($"\n[grey]{jointChoice} valid range: {minDeg:F0}° to {maxDeg:F0}°[/]");
        AnsiConsole.MarkupLine($"[grey]Current angle: {ToDegrees(currentAngle):F1}°[/]\n");

        var angle = AnsiConsole.Prompt(
            new TextPrompt<double>($"Enter {jointChoice} angle (degrees):"));

        var isReachable = angle >= minDeg && angle <= maxDeg;

        var table = new Table()
            .Border(TableBorder.Rounded)
            .AddColumn("Joint")
            .AddColumn("Requested")
            .AddColumn("Valid Range")
            .AddColumn("Status");

        table.AddRow(
            jointChoice,
            $"{angle:F1}°",
            $"{minDeg:F0}° to {maxDeg:F0}°",
            isReachable ? "[green]✓ REACHABLE[/]" : "[red]✗ OUT OF RANGE[/]");

        AnsiConsole.WriteLine();
        AnsiConsole.Write(table);

        if (!isReachable)
        {
            var clamped = Math.Clamp(angle, minDeg, maxDeg);
            AnsiConsole.MarkupLine($"\n[yellow]Suggestion:[/] Nearest valid angle is [cyan]{clamped:F1}°[/]");
        }
        else
        {
            // Calculate foot position when this joint is at the specified angle
            var coxa = jointChoice == "Coxa" ? angle * Math.PI / 180.0 : leg.CoxaAngle;
            var femur = jointChoice == "Femur" ? angle * Math.PI / 180.0 : leg.FemurAngle;
            var tibia = jointChoice == "Tibia" ? angle * Math.PI / 180.0 : leg.TibiaAngle;
            
            ShowFootPositionForAngles(leg, coxa, femur, tibia);
        }
    }

    private void TestAllJointsTogether(HexapodLeg leg)
    {
        var coxaMinDeg = ToDegrees(leg.CoxaLimits.Min);
        var coxaMaxDeg = ToDegrees(leg.CoxaLimits.Max);
        var femurMinDeg = ToDegrees(leg.FemurLimits.Min);
        var femurMaxDeg = ToDegrees(leg.FemurLimits.Max);
        var tibiaMinDeg = ToDegrees(leg.TibiaLimits.Min);
        var tibiaMaxDeg = ToDegrees(leg.TibiaLimits.Max);

        AnsiConsole.MarkupLine("\n[grey]Enter angles for all three joints:[/]\n");

        var coxaAngle = AnsiConsole.Prompt(
            new TextPrompt<double>($"Coxa angle (degrees) [grey]({coxaMinDeg:F0}° to {coxaMaxDeg:F0}°)[/]:"));

        var femurAngle = AnsiConsole.Prompt(
            new TextPrompt<double>($"Femur angle (degrees) [grey]({femurMinDeg:F0}° to {femurMaxDeg:F0}°)[/]:"));

        var tibiaAngle = AnsiConsole.Prompt(
            new TextPrompt<double>($"Tibia angle (degrees) [grey]({tibiaMinDeg:F0}° to {tibiaMaxDeg:F0}°)[/]:"));

        var coxaReachable = coxaAngle >= coxaMinDeg && coxaAngle <= coxaMaxDeg;
        var femurReachable = femurAngle >= femurMinDeg && femurAngle <= femurMaxDeg;
        var tibiaReachable = tibiaAngle >= tibiaMinDeg && tibiaAngle <= tibiaMaxDeg;
        var allReachable = coxaReachable && femurReachable && tibiaReachable;

        var table = new Table()
            .Border(TableBorder.Rounded)
            .Title(allReachable ? "[green]All Joints Reachable[/]" : "[red]Some Joints Out of Range[/]")
            .AddColumn("Joint")
            .AddColumn("Requested")
            .AddColumn("Valid Range")
            .AddColumn("Status");

        table.AddRow("Coxa", $"{coxaAngle:F1}°", $"{coxaMinDeg:F0}° to {coxaMaxDeg:F0}°",
            coxaReachable ? "[green]✓ OK[/]" : "[red]✗ OUT OF RANGE[/]");
        table.AddRow("Femur", $"{femurAngle:F1}°", $"{femurMinDeg:F0}° to {femurMaxDeg:F0}°",
            femurReachable ? "[green]✓ OK[/]" : "[red]✗ OUT OF RANGE[/]");
        table.AddRow("Tibia", $"{tibiaAngle:F1}°", $"{tibiaMinDeg:F0}° to {tibiaMaxDeg:F0}°",
            tibiaReachable ? "[green]✓ OK[/]" : "[red]✗ OUT OF RANGE[/]");

        AnsiConsole.WriteLine();
        AnsiConsole.Write(table);

        if (allReachable)
        {
            ShowFootPositionForAngles(leg, 
                coxaAngle * Math.PI / 180.0, 
                femurAngle * Math.PI / 180.0, 
                tibiaAngle * Math.PI / 180.0);
        }
        else
        {
            AnsiConsole.MarkupLine("\n[yellow]Clamped values:[/]");
            var clampedCoxa = Math.Clamp(coxaAngle, coxaMinDeg, coxaMaxDeg);
            var clampedFemur = Math.Clamp(femurAngle, femurMinDeg, femurMaxDeg);
            var clampedTibia = Math.Clamp(tibiaAngle, tibiaMinDeg, tibiaMaxDeg);
            AnsiConsole.MarkupLine($"  Coxa:  {clampedCoxa:F1}°");
            AnsiConsole.MarkupLine($"  Femur: {clampedFemur:F1}°");
            AnsiConsole.MarkupLine($"  Tibia: {clampedTibia:F1}°");

            if (AnsiConsole.Confirm("\nShow foot position for clamped angles?"))
            {
                ShowFootPositionForAngles(leg, 
                    clampedCoxa * Math.PI / 180.0, 
                    clampedFemur * Math.PI / 180.0, 
                    clampedTibia * Math.PI / 180.0);
            }
        }
    }

    private void RunPresetTests(HexapodLeg leg)
    {
        var coxaMinDeg = ToDegrees(leg.CoxaLimits.Min);
        var coxaMaxDeg = ToDegrees(leg.CoxaLimits.Max);
        var femurMinDeg = ToDegrees(leg.FemurLimits.Min);
        var femurMaxDeg = ToDegrees(leg.FemurLimits.Max);
        var tibiaMinDeg = ToDegrees(leg.TibiaLimits.Min);
        var tibiaMaxDeg = ToDegrees(leg.TibiaLimits.Max);

        var presets = new[]
        {
            (Name: "Default (0°, 0°, 0°)", Coxa: 0.0, Femur: 0.0, Tibia: 0.0),
            (Name: $"Coxa max left ({coxaMaxDeg:F0}°)", Coxa: coxaMaxDeg, Femur: 0.0, Tibia: 0.0),
            (Name: $"Coxa max right ({coxaMinDeg:F0}°)", Coxa: coxaMinDeg, Femur: 0.0, Tibia: 0.0),
            (Name: "Femur up (30°)", Coxa: 0.0, Femur: 30.0, Tibia: -30.0),
            (Name: "Femur down (-30°)", Coxa: 0.0, Femur: -30.0, Tibia: -30.0),
            (Name: "Leg raised (45°, -45°)", Coxa: 0.0, Femur: 45.0, Tibia: -45.0),
            (Name: "Leg lowered (-30°, -60°)", Coxa: 0.0, Femur: -30.0, Tibia: -60.0),
            (Name: $"Tibia at max ({tibiaMaxDeg:F0}°)", Coxa: 0.0, Femur: 0.0, Tibia: tibiaMaxDeg),
            (Name: $"Tibia at min ({tibiaMinDeg:F0}°)", Coxa: 0.0, Femur: 0.0, Tibia: tibiaMinDeg),
            (Name: "All at maximum", Coxa: coxaMaxDeg, Femur: femurMaxDeg, Tibia: tibiaMaxDeg),
            (Name: "All at minimum", Coxa: coxaMinDeg, Femur: femurMinDeg, Tibia: tibiaMinDeg),
        };

        var table = new Table()
            .Border(TableBorder.Rounded)
            .Title("[bold]Preset Angle Tests[/]")
            .AddColumn("Preset")
            .AddColumn("Coxa")
            .AddColumn("Femur")
            .AddColumn("Tibia")
            .AddColumn("Status");

        foreach (var preset in presets)
        {
            var coxaOk = preset.Coxa >= coxaMinDeg && preset.Coxa <= coxaMaxDeg;
            var femurOk = preset.Femur >= femurMinDeg && preset.Femur <= femurMaxDeg;
            var tibiaOk = preset.Tibia >= tibiaMinDeg && preset.Tibia <= tibiaMaxDeg;
            var allOk = coxaOk && femurOk && tibiaOk;

            table.AddRow(
                preset.Name,
                coxaOk ? $"[green]{preset.Coxa:F0}°[/]" : $"[red]{preset.Coxa:F0}°[/]",
                femurOk ? $"[green]{preset.Femur:F0}°[/]" : $"[red]{preset.Femur:F0}°[/]",
                tibiaOk ? $"[green]{preset.Tibia:F0}°[/]" : $"[red]{preset.Tibia:F0}°[/]",
                allOk ? "[green]✓ VALID[/]" : "[red]✗ INVALID[/]");
        }

        AnsiConsole.Write(table);

        if (AnsiConsole.Confirm("\nCalculate foot position for a preset?"))
        {
            var presetChoice = AnsiConsole.Prompt(
                new SelectionPrompt<string>()
                    .Title("Select preset:")
                    .AddChoices(presets.Select(p => p.Name)));

            var selected = presets.First(p => p.Name == presetChoice);
            ShowFootPositionForAngles(leg, 
                selected.Coxa * Math.PI / 180.0, 
                selected.Femur * Math.PI / 180.0, 
                selected.Tibia * Math.PI / 180.0);
        }
    }

    private void ShowFootPositionForAngles(HexapodLeg leg, double coxaRad, double femurRad, double tibiaRad)
    {
        // Use forward kinematics to calculate foot position
        var pos = leg.ForwardKinematics(coxaRad, femurRad, tibiaRad);

        AnsiConsole.MarkupLine("\n[bold cyan]Calculated Foot Position (Leg 0 - Front Right):[/]");
        AnsiConsole.MarkupLine($"  X = [green]{pos.X * 1000:F1}[/] mm");
        AnsiConsole.MarkupLine($"  Y = [green]{pos.Y * 1000:F1}[/] mm");
        AnsiConsole.MarkupLine($"  Z = [green]{pos.Z * 1000:F1}[/] mm");
        AnsiConsole.MarkupLine($"\n[grey]Joint angles: Coxa={ToDegrees(coxaRad):F1}°, Femur={ToDegrees(femurRad):F1}°, Tibia={ToDegrees(tibiaRad):F1}°[/]");
        AnsiConsole.MarkupLine($"[grey]These coordinates can be used in the inverse kinematics test[/]");
    }

    // Helper methods

    private void AnimateSingleStep(HexapodLeg leg, Vector3 startPos, float stepDirX, float stepDirY, float stepHeight)
    {
        var steps = 40;
        var legId = leg.LegId;
        
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
                        
                        // Execute on hardware if enabled
                        ExecuteJointAngles(legId, angles.Value.Coxa, angles.Value.Femur, angles.Value.Tibia);
                    }
                    Thread.Sleep(HardwareEnabled ? 100 : 50);
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
