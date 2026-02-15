using System.Numerics;
using Hexapod.Core.Configuration;

namespace Hexapod.Movement.Kinematics;

/// <summary>
/// Represents a single leg of the hexapod with 3 DOF (Coxa, Femur, Tibia).
/// </summary>
public class HexapodLeg
{
    public int LegId { get; }
    public string Name { get; }
    
    /// <summary>
    /// Angle offset from front of body in radians.
    /// </summary>
    public double MountAngle { get; }
    
    /// <summary>
    /// Distance from body center to leg mount point.
    /// </summary>
    public double MountRadius { get; }

    // Segment lengths in meters
    public double CoxaLength { get; }
    public double FemurLength { get; }
    public double TibiaLength { get; }

    // Current joint angles in radians
    public double CoxaAngle { get; private set; }
    public double FemurAngle { get; private set; }
    public double TibiaAngle { get; private set; }

    // Joint limits in radians
    public (double Min, double Max) CoxaLimits { get; } = (-Math.PI / 4, Math.PI / 4);
    public (double Min, double Max) FemurLimits { get; } = (-Math.PI / 3, Math.PI / 2);
    public (double Min, double Max) TibiaLimits { get; } = (Math.PI / 6, 5 * Math.PI / 6);

    public HexapodLeg(
        int legId,
        string name,
        double mountAngle,
        double mountRadius,
        double coxaLength,
        double femurLength,
        double tibiaLength,
        (double Min, double Max)? coxaLimits = null,
        (double Min, double Max)? femurLimits = null,
        (double Min, double Max)? tibiaLimits = null)
    {
        LegId = legId;
        Name = name;
        MountAngle = mountAngle;
        MountRadius = mountRadius;
        CoxaLength = coxaLength;
        FemurLength = femurLength;
        TibiaLength = tibiaLength;
        if (coxaLimits.HasValue) CoxaLimits = coxaLimits.Value;
        if (femurLimits.HasValue) FemurLimits = femurLimits.Value;
        if (tibiaLimits.HasValue) TibiaLimits = tibiaLimits.Value;
    }

    /// <summary>
    /// Gets the current foot position in body-relative coordinates.
    /// </summary>
    public Vector3 GetFootPosition()
    {
        return ForwardKinematics(CoxaAngle, FemurAngle, TibiaAngle);
    }

    /// <summary>
    /// Sets joint angles directly.
    /// </summary>
    public void SetJointAngles(double coxa, double femur, double tibia)
    {
        CoxaAngle = ClampAngle(coxa, CoxaLimits);
        FemurAngle = ClampAngle(femur, FemurLimits);
        TibiaAngle = ClampAngle(tibia, TibiaLimits);
    }

    /// <summary>
    /// Forward kinematics: joint angles to foot position.
    /// </summary>
    public Vector3 ForwardKinematics(double coxa, double femur, double tibia)
    {
        // tibia is stored as servo-friendly angle (0=straight, increases when folding)
        // Relative tibia angle from femur direction (0 keeps alignment, positive folds inward)
        var tibiaRelative = -tibia;
        
        // Calculate leg span from coxa joint
        var legLength = CoxaLength +
                   FemurLength * Math.Cos(femur) +
                   TibiaLength * Math.Cos(femur + tibiaRelative);

        var z = FemurLength * Math.Sin(femur) +
            TibiaLength * Math.Sin(femur + tibiaRelative);

        // World position = mount point + rotated leg vector
        var totalAngle = MountAngle + coxa;
        var mountX = MountRadius * Math.Cos(MountAngle);
        var mountY = MountRadius * Math.Sin(MountAngle);
        var x = mountX + legLength * Math.Cos(totalAngle);
        var y = mountY + legLength * Math.Sin(totalAngle);

        return new Vector3((float)x, (float)y, (float)z);
    }

    /// <summary>
    /// Inverse kinematics: foot position to joint angles.
    /// Returns null if position is unreachable.
    /// </summary>
    public (double Coxa, double Femur, double Tibia)? InverseKinematics(Vector3 targetPosition)
    {
        // Transform target to mount point frame
        var dx = targetPosition.X - MountRadius * Math.Cos(MountAngle);
        var dy = targetPosition.Y - MountRadius * Math.Sin(MountAngle);
        var dz = targetPosition.Z;

        // Calculate coxa angle (rotation in XY plane from mount direction)
        var distanceToTarget = Math.Sqrt(dx * dx + dy * dy);
        var angleToTarget = Math.Atan2(dy, dx);
        var coxa = angleToTarget - MountAngle;

        // Distance in horizontal plane after coxa rotation (from coxa joint)
        var horizontalDist = distanceToTarget - CoxaLength;
        
        // 3D distance from femur joint to target
        var L = Math.Sqrt(horizontalDist * horizontalDist + dz * dz);

        // Check if target is reachable
        var maxReach = FemurLength + TibiaLength;
        var minReach = Math.Abs(FemurLength - TibiaLength);
        
        if (L > maxReach || L < minReach)
        {
            return null; // Target unreachable
        }

        // Law of cosines to find tibia interior angle
        var cosGamma = (FemurLength * FemurLength + TibiaLength * TibiaLength - L * L) / 
                       (2 * FemurLength * TibiaLength);
        cosGamma = Math.Clamp(cosGamma, -1.0, 1.0);
        var tibiaInterior = Math.Acos(cosGamma);  // Interior angle between femur and tibia

        // Calculate femur angle (from horizontal)
        var alpha = Math.Atan2(dz, horizontalDist);  // Angle to target from horizontal
        var cosBeta = (FemurLength * FemurLength + L * L - TibiaLength * TibiaLength) / 
                      (2 * FemurLength * L);
        cosBeta = Math.Clamp(cosBeta, -1.0, 1.0);
        var beta = Math.Acos(cosBeta);  // Angle in triangle at femur joint
        var femur = alpha + beta;

        // Convert tibia interior angle to our convention (0° = straight, 180° = fully bent)
        var tibia = Math.PI - tibiaInterior;

        // Validate against joint limits
        if (!IsWithinLimits(coxa, CoxaLimits) ||
            !IsWithinLimits(femur, FemurLimits) ||
            !IsWithinLimits(tibia, TibiaLimits))
        {
            return null; // Joint limits exceeded
        }

        return (coxa, femur, tibia);
    }

    private static double ClampAngle(double angle, (double Min, double Max) limits)
    {
        return Math.Clamp(angle, limits.Min, limits.Max);
    }

    private static bool IsWithinLimits(double angle, (double Min, double Max) limits)
    {
        return angle >= limits.Min && angle <= limits.Max;
    }
}

/// <summary>
/// Manages the complete hexapod body with all legs.
/// </summary>
public class HexapodBody
{
    public IReadOnlyList<HexapodLeg> Legs { get; }
    
    /// <summary>
    /// Default standing height above ground in meters.
    /// </summary>
    public double DefaultHeight { get; }

    /// <summary>
    /// Body position offset (translation).
    /// </summary>
    public Vector3 BodyPosition { get; private set; }
    
    /// <summary>
    /// Body rotation (roll, pitch, yaw) in radians.
    /// </summary>
    public Vector3 BodyRotation { get; private set; }

    /// <summary>
    /// Creates a HexapodBody from a KinematicsConfiguration with per-leg mount points and dimensions.
    /// Supports irregular (elongated) hexagonal body shapes.
    /// </summary>
    public HexapodBody(KinematicsConfiguration config)
    {
        DefaultHeight = config.DefaultHeight / 1000.0; // mm to meters
        var limits = config.JointLimits;
        var legs = new List<HexapodLeg>();

        for (int i = 0; i < config.Legs.Count; i++)
        {
            var lc = config.Legs[i];
            legs.Add(new HexapodLeg(
                legId: i,
                name: lc.Name,
                mountAngle: lc.MountAngleDeg * Math.PI / 180.0,
                mountRadius: lc.MountRadiusMm / 1000.0,
                coxaLength: lc.CoxaLengthMm / 1000.0,
                femurLength: lc.FemurLengthMm / 1000.0,
                tibiaLength: lc.TibiaLengthMm / 1000.0,
                coxaLimits: (limits.CoxaMinDeg * Math.PI / 180, limits.CoxaMaxDeg * Math.PI / 180),
                femurLimits: (limits.FemurMinDeg * Math.PI / 180, limits.FemurMaxDeg * Math.PI / 180),
                tibiaLimits: (limits.TibiaMinDeg * Math.PI / 180, limits.TibiaMaxDeg * Math.PI / 180)));
        }

        Legs = legs.AsReadOnly();
        BodyPosition = Vector3.Zero;
        BodyRotation = Vector3.Zero;

        // Initialize all legs to standing position
        InitializeToStandingPosition();
    }

    /// <summary>
    /// Legacy constructor: creates a regular hexagonal body with uniform segment lengths.
    /// </summary>
    public HexapodBody(
        double coxaLength,
        double femurLength,
        double tibiaLength,
        double bodyRadius = 0.08,
        double defaultHeight = 0.08)
    {
        DefaultHeight = defaultHeight;

        // Create 6 legs in hexagonal arrangement
        var legs = new List<HexapodLeg>();
        var legNames = new[] { "FrontRight", "MiddleRight", "RearRight", 
                               "RearLeft", "MiddleLeft", "FrontLeft" };
        var angles = new[] { -30.0, -90.0, -150.0, 150.0, 90.0, 30.0 };

        for (int i = 0; i < 6; i++)
        {
            legs.Add(new HexapodLeg(
                legId: i,
                name: legNames[i],
                mountAngle: angles[i] * Math.PI / 180.0,
                mountRadius: bodyRadius,
                coxaLength: coxaLength,
                femurLength: femurLength,
                tibiaLength: tibiaLength));
        }

        Legs = legs.AsReadOnly();
        BodyPosition = Vector3.Zero;
        BodyRotation = Vector3.Zero;

        // Initialize all legs to standing position
        InitializeToStandingPosition();
    }

    /// <summary>
    /// Initializes all legs to a neutral standing position.
    /// Each foot is placed straight out from its mount point at DefaultHeight below the body.
    /// </summary>
    public void InitializeToStandingPosition()
    {
        foreach (var leg in Legs)
        {
            // Neutral foot target: coxa straight out (0° coxa), at ground level
            var reach = leg.CoxaLength + leg.FemurLength; // horizontal reach when standing
            var footX = leg.MountRadius * Math.Cos(leg.MountAngle)
                      + reach * Math.Cos(leg.MountAngle);
            var footY = leg.MountRadius * Math.Sin(leg.MountAngle)
                      + reach * Math.Sin(leg.MountAngle);
            var footZ = -DefaultHeight;

            var target = new Vector3((float)footX, (float)footY, (float)footZ);
            var angles = leg.InverseKinematics(target);
            if (angles.HasValue)
            {
                leg.SetJointAngles(angles.Value.Coxa, angles.Value.Femur, angles.Value.Tibia);
            }
        }
    }

    /// <summary>
    /// Sets the body pose (position and rotation).
    /// </summary>
    public void SetBodyPose(Vector3 position, Vector3 rotation)
    {
        BodyPosition = position;
        BodyRotation = rotation;
    }

    /// <summary>
    /// Calculates all leg joint angles for given foot positions.
    /// </summary>
    public bool SetFootPositions(IReadOnlyList<Vector3> footPositions)
    {
        if (footPositions.Count != Legs.Count)
        {
            throw new ArgumentException("Foot positions count must match leg count");
        }

        var allValid = true;
        for (int i = 0; i < Legs.Count; i++)
        {
            // Transform foot position to account for body pose
            var transformedPosition = TransformToBodyFrame(footPositions[i]);
            var angles = Legs[i].InverseKinematics(transformedPosition);
            
            if (angles.HasValue)
            {
                Legs[i].SetJointAngles(angles.Value.Coxa, angles.Value.Femur, angles.Value.Tibia);
            }
            else
            {
                allValid = false;
            }
        }

        return allValid;
    }

    private Vector3 TransformToBodyFrame(Vector3 worldPosition)
    {
        // Apply body rotation (simplified - full implementation would use rotation matrix)
        var pos = worldPosition - BodyPosition;
        
        // Apply yaw rotation
        var cosYaw = (float)Math.Cos(BodyRotation.Z);
        var sinYaw = (float)Math.Sin(BodyRotation.Z);
        var x = pos.X * cosYaw + pos.Y * sinYaw;
        var y = -pos.X * sinYaw + pos.Y * cosYaw;
        
        return new Vector3(x, y, pos.Z);
    }
}
