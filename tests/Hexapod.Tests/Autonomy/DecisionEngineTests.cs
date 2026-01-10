using FluentAssertions;
using Hexapod.Autonomy.Decision;
using Hexapod.Core.Configuration;
using Hexapod.Core.Enums;
using Hexapod.Core.Models;
using Hexapod.Core.Services;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Moq;
using Xunit;

namespace Hexapod.Tests.Autonomy;

public class DecisionEngineTests
{
    private readonly Mock<IEventBus> _eventBusMock;
    private readonly Mock<ILogger<DecisionEngine>> _loggerMock;
    private readonly DecisionEngine _engine;

    public DecisionEngineTests()
    {
        _eventBusMock = new Mock<IEventBus>();
        _loggerMock = new Mock<ILogger<DecisionEngine>>();
        
        var config = Options.Create(new HexapodConfiguration
        {
            Autonomy = new AutonomyConfiguration
            {
                ConfirmationRiskThreshold = "Medium",
                ConfirmationTimeoutSeconds = 30,
                TimeoutAction = "SafeAction",
                MinObstacleClearance = 0.3
            }
        });

        _engine = new DecisionEngine(_eventBusMock.Object, config, _loggerMock.Object);
    }

    [Fact]
    public async Task EvaluateAsync_ObstacleAvoidance_ShouldGenerateOptions()
    {
        var context = new DecisionContext
        {
            Type = DecisionType.ObstacleAvoidance,
            SensorState = CreateSensorState(),
            SceneAnalysis = null,
            CurrentMission = null,
            OperationMode = OperationMode.SemiAutonomous
        };

        var decision = await _engine.EvaluateAsync(context);

        decision.Should().NotBeNull();
        decision.Type.Should().Be(DecisionType.ObstacleAvoidance);
        decision.Options.Should().NotBeEmpty();
        decision.Options.Should().Contain(o => o.OptionId == "avoid_left");
        decision.Options.Should().Contain(o => o.OptionId == "avoid_right");
        decision.Options.Should().Contain(o => o.OptionId == "stop");
    }

    [Theory]
    [InlineData(TerrainType.Water, RiskLevel.Critical)]
    [InlineData(TerrainType.Stairs, RiskLevel.High)]
    [InlineData(TerrainType.Slope, RiskLevel.Medium)]
    [InlineData(TerrainType.Flat, RiskLevel.None)]
    public void AssessRisk_Terrain_ShouldReturnAppropriateRisk(TerrainType terrain, RiskLevel expectedRisk)
    {
        var action = new ProposedAction
        {
            ActionType = "Navigate",
            Description = "Navigate to destination",
            TargetTerrain = terrain,
            CrossesBoundary = false,
            IsIrreversible = false
        };

        var sensorState = CreateSensorState();

        var risk = _engine.AssessRisk(action, sensorState);

        risk.Should().Be(expectedRisk);
    }

    [Theory]
    [InlineData(5, RiskLevel.Critical)]
    [InlineData(15, RiskLevel.High)]
    [InlineData(25, RiskLevel.Medium)]
    [InlineData(50, RiskLevel.None)]
    public void AssessRisk_BatteryLevel_ShouldReturnAppropriateRisk(int batteryPercent, RiskLevel expectedMinRisk)
    {
        var action = new ProposedAction
        {
            ActionType = "Navigate",
            Description = "Navigate to destination",
            CrossesBoundary = false,
            IsIrreversible = false
        };

        var sensorState = CreateSensorState(batteryPercent);

        var risk = _engine.AssessRisk(action, sensorState);

        ((int)risk).Should().BeGreaterThanOrEqualTo((int)expectedMinRisk);
    }

    [Fact]
    public void AssessRisk_BoundaryCrossing_ShouldReturnHighRisk()
    {
        var action = new ProposedAction
        {
            ActionType = "Navigate",
            Description = "Cross operational boundary",
            CrossesBoundary = true,
            IsIrreversible = false
        };

        var risk = _engine.AssessRisk(action, CreateSensorState());

        risk.Should().Be(RiskLevel.High);
    }

    [Fact]
    public void AssessRisk_IrreversibleAction_ShouldReturnHighRisk()
    {
        var action = new ProposedAction
        {
            ActionType = "Deploy",
            Description = "Deploy sensor (irreversible)",
            CrossesBoundary = false,
            IsIrreversible = true
        };

        var risk = _engine.AssessRisk(action, CreateSensorState());

        risk.Should().Be(RiskLevel.High);
    }

    [Theory]
    [InlineData(OperationMode.Autonomous, RiskLevel.Medium, false)]
    [InlineData(OperationMode.Autonomous, RiskLevel.Critical, true)]
    [InlineData(OperationMode.SemiAutonomous, RiskLevel.Low, false)]
    [InlineData(OperationMode.SemiAutonomous, RiskLevel.Medium, true)]
    [InlineData(OperationMode.SemiAutonomous, RiskLevel.High, true)]
    [InlineData(OperationMode.RemoteControl, RiskLevel.Critical, false)]
    public void RequiresConfirmation_ShouldReturnCorrectValue(
        OperationMode mode, 
        RiskLevel risk, 
        bool expectedRequiresConfirmation)
    {
        var decision = new Decision
        {
            DecisionId = "test",
            Description = "Test decision",
            RiskLevel = risk,
            Type = DecisionType.PathSelection,
            Options = new List<DecisionOption>().AsReadOnly(),
            CreatedAt = DateTimeOffset.UtcNow,
            RequiresConfirmation = false
        };

        var result = _engine.RequiresConfirmation(decision, mode);

        result.Should().Be(expectedRequiresConfirmation);
    }

    [Fact]
    public async Task EvaluateAsync_EmergencyAction_ShouldIncludeEmergencyStop()
    {
        var context = new DecisionContext
        {
            Type = DecisionType.EmergencyAction,
            SensorState = CreateSensorState(),
            SceneAnalysis = null,
            CurrentMission = null,
            OperationMode = OperationMode.SemiAutonomous
        };

        var decision = await _engine.EvaluateAsync(context);

        decision.Options.Should().Contain(o => o.OptionId == "emergency_stop");
        decision.Options.First(o => o.OptionId == "emergency_stop").Confidence.Should().Be(1.0);
    }

    private static SensorState CreateSensorState(int batteryPercent = 80)
    {
        return new SensorState
        {
            Position = new GeoPosition
            {
                Latitude = 47.6062,
                Longitude = -122.3321,
                Accuracy = 2.5
            },
            Orientation = new Orientation
            {
                Roll = 0,
                Pitch = 0,
                Yaw = 0
            },
            PowerStatus = new PowerStatus
            {
                BatteryPercentage = batteryPercent,
                BatteryVoltage = 12.0,
                BatteryCurrent = 1.5,
                IsCharging = false
            },
            Timestamp = DateTimeOffset.UtcNow
        };
    }
}
