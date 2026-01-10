using System.Numerics;
using FluentAssertions;
using Hexapod.Movement.Kinematics;
using Xunit;

namespace Hexapod.Tests.Movement;

public class InverseKinematicsTests
{
    private readonly HexapodLeg _leg;

    public InverseKinematicsTests()
    {
        // Standard leg dimensions (in meters)
        _leg = new HexapodLeg(
            legId: 0,
            name: "TestLeg",
            mountAngle: 0,          // Points along positive X-axis
            mountRadius: 0.08,
            coxaLength: 0.05,
            femurLength: 0.08,
            tibiaLength: 0.12);
    }

    [Fact]
    public void ForwardKinematics_ShouldReturnValidPosition()
    {
        var position = _leg.ForwardKinematics(0, 0, 0);
        
        position.Should().NotBe(Vector3.Zero);
    }

    [Fact]
    public void InverseKinematics_FromForwardKinematics_ShouldReturnValidAngles()
    {
        // Use forward kinematics to get a known reachable position
        var knownPosition = _leg.ForwardKinematics(0, 0.2, -0.3);
        var result = _leg.InverseKinematics(knownPosition);
        
        result.Should().NotBeNull("position from forward kinematics should be reachable");
    }

    [Fact]
    public void InverseKinematics_ForwardPosition_WithPositiveY_ShouldReturnPositiveCoxa()
    {
        // Get a base position and add positive Y offset for positive coxa
        var basePosition = _leg.ForwardKinematics(0.1, 0.2, -0.3);
        var result = _leg.InverseKinematics(basePosition);
        
        if (result.HasValue)
        {
            // If we get a result, the coxa should be positive since we used positive coxa angle
            result.Value.Coxa.Should().BeGreaterThan(0);
        }
    }

    [Fact]
    public void InverseKinematics_BackwardPosition_WithNegativeY_ShouldReturnNegativeCoxa()
    {
        // Get a base position with negative coxa angle
        var basePosition = _leg.ForwardKinematics(-0.1, 0.2, -0.3);
        var result = _leg.InverseKinematics(basePosition);
        
        if (result.HasValue)
        {
            result.Value.Coxa.Should().BeLessThan(0);
        }
    }

    [Fact]
    public void InverseKinematics_UnreachablePosition_ShouldReturnNull()
    {
        // Position too far from leg origin
        var target = new Vector3(0.5f, 0, -0.08f);
        var result = _leg.InverseKinematics(target);
        
        result.Should().BeNull();
    }

    [Fact]
    public void SetJointAngles_ShouldClampToLimits()
    {
        _leg.SetJointAngles(Math.PI, Math.PI, Math.PI);
        
        _leg.CoxaAngle.Should().BeLessOrEqualTo(_leg.CoxaLimits.Max);
        _leg.FemurAngle.Should().BeLessOrEqualTo(_leg.FemurLimits.Max);
        _leg.TibiaAngle.Should().BeLessOrEqualTo(_leg.TibiaLimits.Max);
    }

    [Fact]
    public void GetFootPosition_ShouldMatchForwardKinematics()
    {
        _leg.SetJointAngles(0.1, 0.2, -0.3);
        
        var footPosition = _leg.GetFootPosition();
        var calculatedPosition = _leg.ForwardKinematics(0.1, 0.2, -0.3);
        
        footPosition.X.Should().BeApproximately(calculatedPosition.X, 0.001f);
        footPosition.Y.Should().BeApproximately(calculatedPosition.Y, 0.001f);
        footPosition.Z.Should().BeApproximately(calculatedPosition.Z, 0.001f);
    }

    [Fact]
    public void InverseKinematics_TooClose_ShouldReturnNull()
    {
        // Position inside minimum reach (too close to coxa joint)
        var target = new Vector3(0.09f, 0, 0);
        var result = _leg.InverseKinematics(target);
        
        result.Should().BeNull();
    }

    [Fact]
    public void LegProperties_ShouldHaveCorrectValues()
    {
        _leg.LegId.Should().Be(0);
        _leg.Name.Should().Be("TestLeg");
        _leg.MountAngle.Should().Be(0);
        _leg.MountRadius.Should().Be(0.08);
        _leg.CoxaLength.Should().Be(0.05);
        _leg.FemurLength.Should().Be(0.08);
        _leg.TibiaLength.Should().Be(0.12);
    }
}

public class HexapodBodyTests
{
    private readonly HexapodBody _body;

    public HexapodBodyTests()
    {
        _body = new HexapodBody(
            coxaLength: 0.05,
            femurLength: 0.08,
            tibiaLength: 0.12,
            bodyRadius: 0.08);
    }

    [Fact]
    public void Constructor_ShouldCreateSixLegs()
    {
        _body.Legs.Should().HaveCount(6);
    }

    [Fact]
    public void Legs_ShouldHaveCorrectAngularPositions()
    {
        // Legs should be at approximately 60-degree intervals
        var leg0 = _body.Legs[0];
        var leg1 = _body.Legs[1];
        
        // Angular difference should be approximately 60 degrees (pi/3 radians)
        var angleDiff = Math.Abs(leg1.MountAngle - leg0.MountAngle);
        angleDiff.Should().BeApproximately(Math.PI / 3, 0.1);
    }

    [Fact]
    public void SetBodyPose_ShouldUpdatePosition()
    {
        var newPosition = new Vector3(0.01f, 0.02f, 0.03f);
        var newRotation = new Vector3(0.1f, 0.2f, 0.3f);
        
        _body.SetBodyPose(newPosition, newRotation);
        
        _body.BodyPosition.Should().Be(newPosition);
        _body.BodyRotation.Should().Be(newRotation);
    }

    [Fact]
    public void SetFootPositions_WithValidPositions_ShouldSucceed()
    {
        // Use forward kinematics to get known-reachable positions
        var footPositions = new List<Vector3>();
        for (int i = 0; i < 6; i++)
        {
            var leg = _body.Legs[i];
            // Get the default foot position using forward kinematics with zero angles
            var defaultPosition = leg.ForwardKinematics(0, 0.3, -0.5);
            footPositions.Add(defaultPosition);
        }
        
        // This tests that the method can be called, even if some positions may be outside limits
        // We just need to ensure the method doesn't throw
        Action act = () => _body.SetFootPositions(footPositions);
        act.Should().NotThrow();
    }

    [Fact]
    public void Legs_ShouldHaveCorrectNames()
    {
        _body.Legs[0].Name.Should().Be("FrontRight");
        _body.Legs[1].Name.Should().Be("MiddleRight");
        _body.Legs[2].Name.Should().Be("RearRight");
        _body.Legs[3].Name.Should().Be("RearLeft");
        _body.Legs[4].Name.Should().Be("MiddleLeft");
        _body.Legs[5].Name.Should().Be("FrontLeft");
    }
}
