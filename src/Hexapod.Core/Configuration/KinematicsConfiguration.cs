namespace Hexapod.Core.Configuration;

/// <summary>
/// Complete kinematics configuration for the hexapod, supporting irregular
/// hexagonal body shapes with per-leg mount points and dimensions.
/// </summary>
public class KinematicsConfiguration
{
    public const string SectionName = "Hexapod:Kinematics";

    /// <summary>
    /// Default body height above ground in mm.
    /// </summary>
    public double DefaultHeight { get; set; } = 45.0;

    /// <summary>
    /// Maximum reach radius for any leg in mm (used for workspace bounding).
    /// </summary>
    public double MaxReachRadius { get; set; } = 200.0;

    /// <summary>
    /// Body shape configuration (thickness, description).
    /// </summary>
    public BodyConfiguration Body { get; set; } = new();

    /// <summary>
    /// Per-leg configuration (mount angle, mount radius, segment lengths).
    /// Exactly 6 entries, ordered: FrontRight, MiddleRight, RearRight, RearLeft, MiddleLeft, FrontLeft.
    /// Each leg can have a different mount radius, allowing irregular (elongated) hexagonal body shapes.
    /// Defaults are defined in config/hexapod.json.
    /// </summary>
    public List<LegConfiguration> Legs { get; set; } = new();

    /// <summary>
    /// Global joint limits (applied to all legs unless overridden per-leg).
    /// </summary>
    public JointLimitsConfiguration JointLimits { get; set; } = new();
}

/// <summary>
/// Body plate configuration.
/// </summary>
public class BodyConfiguration
{
    /// <summary>
    /// Body plate thickness in mm.
    /// </summary>
    public double ThicknessMm { get; set; } = 12.0;

    /// <summary>
    /// Human-readable description of the body shape.
    /// </summary>
    public string Description { get; set; } = "Hexagonal body defined by 6 leg mount points.";
}

/// <summary>
/// Configuration for a single leg's mount point and segment dimensions.
/// </summary>
public class LegConfiguration
{
    /// <summary>
    /// Leg name (e.g. "FrontRight", "MiddleLeft").
    /// </summary>
    public string Name { get; set; } = string.Empty;

    /// <summary>
    /// Mount angle from body front (X-axis) in degrees.
    /// Right-hand rule: positive = counter-clockwise (left side), negative = clockwise (right side).
    /// </summary>
    public double MountAngleDeg { get; set; }

    /// <summary>
    /// Distance from body center to this leg's mount point in mm.
    /// Different radii per leg allow irregular (elongated) hexagonal bodies.
    /// </summary>
    public double MountRadiusMm { get; set; } = 90.0;

    /// <summary>
    /// Coxa (hip) segment length in mm.
    /// </summary>
    public double CoxaLengthMm { get; set; } = 40.0;

    /// <summary>
    /// Femur (thigh) segment length in mm.
    /// </summary>
    public double FemurLengthMm { get; set; } = 60.0;

    /// <summary>
    /// Tibia (shin) segment length in mm.
    /// </summary>
    public double TibiaLengthMm { get; set; } = 135.0;
}

/// <summary>
/// Joint angle limits in degrees.
/// </summary>
public class JointLimitsConfiguration
{
    public double CoxaMinDeg { get; set; } = -45.0;
    public double CoxaMaxDeg { get; set; } = 45.0;
    public double FemurMinDeg { get; set; } = -60.0;
    public double FemurMaxDeg { get; set; } = 90.0;
    public double TibiaMinDeg { get; set; } = 30.0;
    public double TibiaMaxDeg { get; set; } = 150.0;
}

/// <summary>
/// Rendering dimensions for 3D visualization.
/// </summary>
public class RenderingConfiguration
{
    public const string SectionName = "Hexapod:Rendering";

    public double CoxaDiameterMm { get; set; } = 10.0;
    public double FemurDiameterMm { get; set; } = 8.0;
    public double TibiaDiameterMm { get; set; } = 6.0;
    public double JointDiameterMm { get; set; } = 12.0;
    public double FootDiameterMm { get; set; } = 8.0;
}
