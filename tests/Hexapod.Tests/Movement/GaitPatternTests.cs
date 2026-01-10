using FluentAssertions;
using Hexapod.Core.Enums;
using Hexapod.Movement.Gait;
using Xunit;

namespace Hexapod.Tests.Movement;

public class GaitPatternTests
{
    [Theory]
    [InlineData(typeof(TripodGait), GaitType.Tripod)]
    [InlineData(typeof(WaveGait), GaitType.Wave)]
    [InlineData(typeof(RippleGait), GaitType.Ripple)]
    [InlineData(typeof(MetachronalGait), GaitType.Metachronal)]
    public void GaitType_ShouldReturnCorrectType(Type gaitClass, GaitType expectedType)
    {
        var gait = (IGaitGenerator)Activator.CreateInstance(gaitClass)!;
        
        gait.GaitType.Should().Be(expectedType);
    }

    [Theory]
    [InlineData(typeof(TripodGait))]
    [InlineData(typeof(WaveGait))]
    [InlineData(typeof(RippleGait))]
    [InlineData(typeof(MetachronalGait))]
    public void GetLegPhases_ShouldReturnSixPhases(Type gaitType)
    {
        var gait = (IGaitGenerator)Activator.CreateInstance(gaitType)!;
        
        var phases = gait.GetLegPhases(0.5);
        
        phases.Should().HaveCount(6);
        phases.Should().OnlyContain(p => p.Phase >= 0 && p.Phase < 1);
    }

    [Fact]
    public void TripodGait_ShouldHaveTwoGroups()
    {
        var gait = new TripodGait();
        
        var phases = gait.GetLegPhases(0);
        
        // Group 1 legs (0, 2, 4) should have same phase
        phases[0].Phase.Should().BeApproximately(phases[2].Phase, 0.01);
        phases[2].Phase.Should().BeApproximately(phases[4].Phase, 0.01);
        
        // Group 2 legs (1, 3, 5) should have same phase
        phases[1].Phase.Should().BeApproximately(phases[3].Phase, 0.01);
        phases[3].Phase.Should().BeApproximately(phases[5].Phase, 0.01);
        
        // Groups should be 0.5 phase apart
        Math.Abs(phases[0].Phase - phases[1].Phase).Should().BeApproximately(0.5, 0.01);
    }

    [Theory]
    [InlineData(typeof(TripodGait))]
    [InlineData(typeof(WaveGait))]
    [InlineData(typeof(RippleGait))]
    [InlineData(typeof(MetachronalGait))]
    public void GenerateFootPositions_ShouldReturnSixPositions(Type gaitType)
    {
        var gait = (IGaitGenerator)Activator.CreateInstance(gaitType)!;

        for (double phase = 0; phase <= 1; phase += 0.1)
        {
            var positions = gait.GenerateFootPositions(
                phase: phase,
                direction: 0,
                speed: 0.5,
                turnRate: 0,
                stepLength: 0.05,
                stepHeight: 0.03,
                bodyHeight: 0.08);
            
            positions.Should().HaveCount(6);
        }
    }

    [Theory]
    [InlineData(typeof(TripodGait))]
    [InlineData(typeof(WaveGait))]
    [InlineData(typeof(RippleGait))]
    [InlineData(typeof(MetachronalGait))]
    public void DutyFactor_ShouldBeValid(Type gaitType)
    {
        var gait = (IGaitGenerator)Activator.CreateInstance(gaitType)!;
        
        gait.DutyFactor.Should().BeInRange(0.1, 0.95, "Duty factor should be reasonable");
    }

    [Fact]
    public void TripodGait_Properties_ShouldBeCorrect()
    {
        var gait = new TripodGait();
        
        gait.GaitType.Should().Be(GaitType.Tripod);
        gait.DutyFactor.Should().Be(0.5);
        gait.Stability.Should().Be(0.7);
        gait.EnergyEfficiency.Should().Be(0.6);
    }

    [Fact]
    public void WaveGait_Properties_ShouldBeCorrect()
    {
        var gait = new WaveGait();
        
        gait.GaitType.Should().Be(GaitType.Wave);
        gait.DutyFactor.Should().BeApproximately(5.0 / 6.0, 0.01);
        gait.Stability.Should().Be(0.95);
        gait.EnergyEfficiency.Should().Be(0.9);
    }

    [Fact]
    public void MetachronalGait_Properties_ShouldBeCorrect()
    {
        var gait = new MetachronalGait();
        
        gait.GaitType.Should().Be(GaitType.Metachronal);
        gait.EnergyEfficiency.Should().Be(0.95);
        gait.Stability.Should().Be(0.9);
    }

    [Fact]
    public void RippleGait_Properties_ShouldBeCorrect()
    {
        var gait = new RippleGait();
        
        gait.GaitType.Should().Be(GaitType.Ripple);
        gait.DutyFactor.Should().BeApproximately(2.0 / 3.0, 0.01);
        gait.Stability.Should().Be(0.85);
        gait.EnergyEfficiency.Should().Be(0.75);
    }

    [Theory]
    [InlineData(GaitType.Tripod)]
    [InlineData(GaitType.Wave)]
    [InlineData(GaitType.Ripple)]
    [InlineData(GaitType.Metachronal)]
    public void GaitFactory_Create_ShouldReturnCorrectGait(GaitType gaitType)
    {
        var gait = GaitFactory.Create(gaitType);
        
        gait.GaitType.Should().Be(gaitType);
    }

    [Fact]
    public void GaitFactory_RecommendGait_LowBattery_ShouldReturnMetachronal()
    {
        var recommended = GaitFactory.RecommendGait(TerrainType.Flat, 0.5, 0.2);
        
        recommended.Should().Be(GaitType.Metachronal);
    }

    [Fact]
    public void GaitFactory_RecommendGait_FlatTerrain_HighSpeed_ShouldReturnTripod()
    {
        var recommended = GaitFactory.RecommendGait(TerrainType.Flat, 0.8, 0.8);
        
        recommended.Should().Be(GaitType.Tripod);
    }

    [Fact]
    public void GaitFactory_RecommendGait_RoughTerrain_ShouldReturnWave()
    {
        var recommended = GaitFactory.RecommendGait(TerrainType.Rough, 0.5, 0.8);
        
        recommended.Should().Be(GaitType.Wave);
    }
}
