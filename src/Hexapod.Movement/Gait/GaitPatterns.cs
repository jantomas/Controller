using System.Numerics;
using Hexapod.Core.Enums;

namespace Hexapod.Movement.Gait;

/// <summary>
/// Represents the phase information for a single leg in a gait cycle.
/// </summary>
public record LegPhase
{
    public int LegId { get; init; }
    public double Phase { get; init; } // 0.0 to 1.0
    public bool IsSwingPhase { get; init; } // true = swing (leg in air), false = stance (leg on ground)
    public double SwingProgress { get; init; } // 0.0 to 1.0 during swing
}

/// <summary>
/// Base interface for gait pattern generators.
/// </summary>
public interface IGaitGenerator
{
    /// <summary>
    /// Gets the gait type.
    /// </summary>
    GaitType GaitType { get; }

    /// <summary>
    /// Gets the duty factor (fraction of cycle leg is on ground).
    /// </summary>
    double DutyFactor { get; }

    /// <summary>
    /// Generates foot positions for a given phase of the gait cycle.
    /// </summary>
    /// <param name="phase">Current phase of gait cycle (0.0 to 1.0)</param>
    /// <param name="direction">Movement direction in radians (0 = forward)</param>
    /// <param name="speed">Movement speed (0.0 to 1.0)</param>
    /// <param name="turnRate">Turn rate (-1.0 to 1.0, positive = right)</param>
    /// <param name="stepLength">Step length in meters</param>
    /// <param name="stepHeight">Step height in meters</param>
    /// <param name="bodyHeight">Body height above ground in meters</param>
    /// <returns>Array of 6 foot positions (one per leg)</returns>
    Vector3[] GenerateFootPositions(
        double phase,
        double direction,
        double speed,
        double turnRate,
        double stepLength,
        double stepHeight,
        double bodyHeight);

    /// <summary>
    /// Gets the phase information for each leg at a given cycle phase.
    /// </summary>
    LegPhase[] GetLegPhases(double cyclePhase);

    /// <summary>
    /// Gets the energy efficiency rating (0.0 to 1.0).
    /// Higher values indicate better energy efficiency.
    /// </summary>
    double EnergyEfficiency { get; }

    /// <summary>
    /// Gets the stability rating (0.0 to 1.0).
    /// Higher values indicate better stability.
    /// </summary>
    double Stability { get; }
}

/// <summary>
/// Abstract base class for gait generators with common functionality.
/// </summary>
public abstract class GaitGeneratorBase : IGaitGenerator
{
    protected const int LegCount = 6;
    protected static readonly double[] LegAngles = { -30, -90, -150, 150, 90, 30 }; // degrees

    public abstract GaitType GaitType { get; }
    public abstract double DutyFactor { get; }
    public abstract double EnergyEfficiency { get; }
    public abstract double Stability { get; }

    /// <summary>
    /// Phase offsets for each leg (0.0 to 1.0).
    /// </summary>
    protected abstract double[] LegPhaseOffsets { get; }

    public Vector3[] GenerateFootPositions(
        double phase,
        double direction,
        double speed,
        double turnRate,
        double stepLength,
        double stepHeight,
        double bodyHeight)
    {
        var positions = new Vector3[LegCount];
        var legPhases = GetLegPhases(phase);

        for (int i = 0; i < LegCount; i++)
        {
            positions[i] = CalculateLegPosition(
                i,
                legPhases[i],
                direction,
                speed,
                turnRate,
                stepLength,
                stepHeight,
                bodyHeight);
        }

        return positions;
    }

    public LegPhase[] GetLegPhases(double cyclePhase)
    {
        var phases = new LegPhase[LegCount];
        
        for (int i = 0; i < LegCount; i++)
        {
            var legPhase = (cyclePhase + LegPhaseOffsets[i]) % 1.0;
            var isSwing = legPhase >= DutyFactor;
            var swingProgress = isSwing 
                ? (legPhase - DutyFactor) / (1.0 - DutyFactor)
                : 0.0;

            phases[i] = new LegPhase
            {
                LegId = i,
                Phase = legPhase,
                IsSwingPhase = isSwing,
                SwingProgress = swingProgress
            };
        }

        return phases;
    }

    protected virtual Vector3 CalculateLegPosition(
        int legIndex,
        LegPhase legPhase,
        double direction,
        double speed,
        double turnRate,
        double stepLength,
        double stepHeight,
        double bodyHeight)
    {
        var legAngle = LegAngles[legIndex] * Math.PI / 180.0;
        var defaultReach = 0.12; // Default leg reach in meters

        // Calculate base position
        var baseX = defaultReach * Math.Cos(legAngle);
        var baseY = defaultReach * Math.Sin(legAngle);

        // Calculate movement offset
        double offsetX = 0, offsetY = 0, offsetZ = 0;

        if (legPhase.IsSwingPhase)
        {
            // Swing phase - leg moves in air
            var swingPhase = legPhase.SwingProgress;
            
            // Forward/backward movement (cosine interpolation)
            var movementPhase = Math.Cos(Math.PI * (1 - swingPhase));
            offsetX = speed * stepLength * 0.5 * movementPhase * Math.Cos(direction);
            offsetY = speed * stepLength * 0.5 * movementPhase * Math.Sin(direction);
            
            // Height trajectory (parabolic)
            offsetZ = stepHeight * 4 * swingPhase * (1 - swingPhase);
            
            // Add turn component
            var turnOffset = turnRate * stepLength * 0.3;
            if (legIndex < 3) // Right side legs
            {
                offsetX -= turnOffset * Math.Sin(legAngle) * movementPhase;
                offsetY += turnOffset * Math.Cos(legAngle) * movementPhase;
            }
            else // Left side legs
            {
                offsetX += turnOffset * Math.Sin(legAngle) * movementPhase;
                offsetY -= turnOffset * Math.Cos(legAngle) * movementPhase;
            }
        }
        else
        {
            // Stance phase - leg pushes against ground
            var stanceProgress = legPhase.Phase / DutyFactor;
            var pushPhase = 2 * stanceProgress - 1; // -1 to 1
            
            offsetX = -speed * stepLength * 0.5 * pushPhase * Math.Cos(direction);
            offsetY = -speed * stepLength * 0.5 * pushPhase * Math.Sin(direction);
            offsetZ = 0;
        }

        return new Vector3(
            (float)(baseX + offsetX),
            (float)(baseY + offsetY),
            (float)(-bodyHeight + offsetZ));
    }
}

/// <summary>
/// Tripod gait - fastest gait, alternating groups of 3 legs.
/// Legs 0, 2, 4 move together, then 1, 3, 5.
/// </summary>
public class TripodGait : GaitGeneratorBase
{
    public override GaitType GaitType => GaitType.Tripod;
    public override double DutyFactor => 0.5;
    public override double EnergyEfficiency => 0.6;
    public override double Stability => 0.7;

    protected override double[] LegPhaseOffsets => new[] { 0.0, 0.5, 0.0, 0.5, 0.0, 0.5 };
}

/// <summary>
/// Wave gait - most stable, legs move in sequence.
/// Each leg has a unique phase offset.
/// </summary>
public class WaveGait : GaitGeneratorBase
{
    public override GaitType GaitType => GaitType.Wave;
    public override double DutyFactor => 5.0 / 6.0; // 5/6 legs always on ground
    public override double EnergyEfficiency => 0.9;
    public override double Stability => 0.95;

    protected override double[] LegPhaseOffsets => new[] 
    { 
        0.0 / 6.0,  // Front Right
        1.0 / 6.0,  // Middle Right
        2.0 / 6.0,  // Rear Right
        3.0 / 6.0,  // Rear Left
        4.0 / 6.0,  // Middle Left
        5.0 / 6.0   // Front Left
    };
}

/// <summary>
/// Ripple gait - balanced between speed and stability.
/// Two legs swing at once on opposite sides.
/// </summary>
public class RippleGait : GaitGeneratorBase
{
    public override GaitType GaitType => GaitType.Ripple;
    public override double DutyFactor => 2.0 / 3.0;
    public override double EnergyEfficiency => 0.75;
    public override double Stability => 0.85;

    protected override double[] LegPhaseOffsets => new[] 
    { 
        0.0 / 3.0,  // Front Right
        2.0 / 3.0,  // Middle Right
        1.0 / 3.0,  // Rear Right
        0.0 / 3.0,  // Rear Left
        2.0 / 3.0,  // Middle Left
        1.0 / 3.0   // Front Left
    };
}

/// <summary>
/// Metachronal gait - smooth wave, energy efficient for sustained travel.
/// </summary>
public class MetachronalGait : GaitGeneratorBase
{
    public override GaitType GaitType => GaitType.Metachronal;
    public override double DutyFactor => 0.75;
    public override double EnergyEfficiency => 0.95;
    public override double Stability => 0.9;

    protected override double[] LegPhaseOffsets => new[] 
    { 
        5.0 / 6.0,  // Front Right - last to lift
        3.0 / 6.0,  // Middle Right
        1.0 / 6.0,  // Rear Right - first to lift
        0.0 / 6.0,  // Rear Left
        2.0 / 6.0,  // Middle Left
        4.0 / 6.0   // Front Left
    };
}

/// <summary>
/// Factory for creating gait generators.
/// </summary>
public static class GaitFactory
{
    public static IGaitGenerator Create(GaitType gaitType)
    {
        return gaitType switch
        {
            GaitType.Tripod => new TripodGait(),
            GaitType.Wave => new WaveGait(),
            GaitType.Ripple => new RippleGait(),
            GaitType.Metachronal => new MetachronalGait(),
            _ => throw new ArgumentException($"Unknown gait type: {gaitType}")
        };
    }

    /// <summary>
    /// Recommends the best gait for given conditions.
    /// </summary>
    public static GaitType RecommendGait(
        TerrainType terrain,
        double speedRequirement, // 0.0 to 1.0
        double batteryLevel)     // 0.0 to 1.0
    {
        // Energy conservation priority when battery is low
        if (batteryLevel < 0.3)
        {
            return GaitType.Metachronal;
        }

        return terrain switch
        {
            TerrainType.Flat when speedRequirement > 0.7 => GaitType.Tripod,
            TerrainType.Flat => GaitType.Ripple,
            TerrainType.Rough or TerrainType.Slope or TerrainType.Stairs => GaitType.Wave,
            TerrainType.Sand or TerrainType.Vegetation => GaitType.Metachronal,
            _ => GaitType.Ripple // Default balanced gait
        };
    }
}
