using System.Numerics;
using Hexapod.Core.Enums;
using Hexapod.GaitTest;
using Hexapod.Movement.Gait;
using Hexapod.Movement.Kinematics;
using Spectre.Console;

// Configuration
const double CoxaLength = 0.03;   // 30mm
const double FemurLength = 0.08;  // 80mm
const double TibiaLength = 0.12;  // 120mm
const double BodyRadius = 0.08;   // 80mm

// Create hexapod body model
var body = new HexapodBody(CoxaLength, FemurLength, TibiaLength, BodyRadius);
var tester = new GaitTester(body);

// Welcome banner
AnsiConsole.Write(
    new FigletText("Gait Tester")
        .LeftJustified()
        .Color(Color.Green));

AnsiConsole.MarkupLine("[grey]Hexapod Inverse Kinematics & Gait Pattern Test Utility[/]");
AnsiConsole.MarkupLine($"[grey]Leg dimensions: Coxa={CoxaLength * 1000}mm, Femur={FemurLength * 1000}mm, Tibia={TibiaLength * 1000}mm[/]\n");

// Main menu loop
while (true)
{
    var choice = AnsiConsole.Prompt(
        new SelectionPrompt<string>()
            .Title("[bold]Select operation:[/]")
            .PageSize(15)
            .HighlightStyle(Style.Parse("cyan"))
            .AddChoices(
                "🦵 Test leg lift/put down",
                "👣 Test single step",
                "🚶 Run tripod gait sample",
                "🌊 Run wave gait sample",
                "〰️ Run ripple gait sample",
                "🔄 Run metachronal gait sample",
                "📊 Compare all gait patterns",
                "🎯 Test inverse kinematics",
                "📐 Show leg workspace",
                "📋 Show leg configuration",
                "🔧 Interactive foot position",
                "❌ Exit"));

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
        case "📐 Show leg workspace":
            tester.ShowLegWorkspace();
            break;
        case "📋 Show leg configuration":
            tester.ShowLegConfiguration();
            break;
        case "🔧 Interactive foot position":
            tester.InteractiveFootPosition();
            break;
        case "❌ Exit":
            AnsiConsole.MarkupLine("[green]Goodbye![/]");
            return;
    }

    AnsiConsole.WriteLine();
    AnsiConsole.MarkupLine("[grey]Press any key to continue...[/]");
    Console.ReadKey(true);
    AnsiConsole.Clear();
}
