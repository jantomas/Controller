using System.Numerics;

// Hexapod configuration (from appsettings)
const double CoxaLength = 0.040;   // 40mm
const double FemurLength = 0.060;  // 60mm
const double TibiaLength = 0.135;  // 135mm
const double BodyRadius = 0.090;   // 90mm
const double MountAngle = Math.PI / 4; // 45° for Front Right leg

// Joint limits (radians)
const double CoxaMinRad = -Math.PI / 4;   // -45°
const double CoxaMaxRad = Math.PI / 4;    // 45°
const double FemurMinRad = -Math.PI / 3;  // -60°
const double FemurMaxRad = Math.PI / 2;   // 90°
const double TibiaMinRad = Math.PI / 6;   // 30°
const double TibiaMaxRad = 5 * Math.PI / 6;  // 150°

static Vector3 ForwardKinematics(double coxa, double femur, double tibia)
{
    // tibia angle stored as servo-friendly (0 = straight); relative from femur is negative of tibia
    var tibiaRelative = -tibia;
    
    // Span from coxa joint to foot
    var legLength = CoxaLength +
                   FemurLength * Math.Cos(femur) +
                   TibiaLength * Math.Cos(femur + tibiaRelative);
    
    var z = FemurLength * Math.Sin(femur) +
            TibiaLength * Math.Sin(femur + tibiaRelative);

    // World coordinates: mount point + rotated leg vector
    var totalAngle = MountAngle + coxa;
    var mountX = BodyRadius * Math.Cos(MountAngle);
    var mountY = BodyRadius * Math.Sin(MountAngle);
    var x = mountX + legLength * Math.Cos(totalAngle);
    var y = mountY + legLength * Math.Sin(totalAngle);

    return new Vector3((float)x, (float)y, (float)z);
}

static (double Coxa, double Femur, double Tibia)? InverseKinematics(Vector3 target)
{
    var dx = target.X - BodyRadius * Math.Cos(MountAngle);
    var dy = target.Y - BodyRadius * Math.Sin(MountAngle);
    var dz = target.Z;

    // Calculate coxa angle (rotation in XY plane)
    var coxa = Math.Atan2(dy, dx) - MountAngle;

    // Distance from coxa joint in horizontal plane
    var distanceFromMount = Math.Sqrt(dx * dx + dy * dy);
    var horizontalDist = distanceFromMount - CoxaLength;
    
    // Distance from femur joint to target
    var L = Math.Sqrt(horizontalDist * horizontalDist + dz * dz);

    // Check if target is reachable
    var maxReach = FemurLength + TibiaLength;
    var minReach = Math.Abs(FemurLength - TibiaLength);
    
    Console.WriteLine($"  [DEBUG] dx={dx * 1000:F1}mm, dy={dy * 1000:F1}mm, dz={dz * 1000:F1}mm");
    Console.WriteLine($"  [DEBUG] distFromMount={distanceFromMount * 1000:F1}mm, horizontal={horizontalDist * 1000:F1}mm, L={L * 1000:F1}mm");
    Console.WriteLine($"  [DEBUG] Reach range: {minReach * 1000:F1}mm to {maxReach * 1000:F1}mm");
    Console.WriteLine($"  [DEBUG] Coxa angle={ToDegrees(coxa):F2}°");
    
    if (L > maxReach || L < minReach)
    {
        Console.WriteLine($"  [DEBUG] Failed: L={L * 1000:F1}mm is outside reach range");
        return null;
    }

    // Law of cosines for tibia angle
    var cosGamma = (FemurLength * FemurLength + TibiaLength * TibiaLength - L * L) / 
                   (2 * FemurLength * TibiaLength);
    cosGamma = Math.Clamp(cosGamma, -1.0, 1.0);
    var tibia = Math.PI - Math.Acos(cosGamma);

    // Calculate femur angle
    var alpha = Math.Atan2(dz, horizontalDist);
    var cosBeta = (FemurLength * FemurLength + L * L - TibiaLength * TibiaLength) / 
                  (2 * FemurLength * L);
    cosBeta = Math.Clamp(cosBeta, -1.0, 1.0);
    var beta = Math.Acos(cosBeta);
    var femur = alpha + beta;

    Console.WriteLine($"  [DEBUG] Before limits: femur={ToDegrees(femur):F2}°, tibia={ToDegrees(tibia):F2}°");
    
    if (coxa < CoxaMinRad || coxa > CoxaMaxRad ||
        femur < FemurMinRad || femur > FemurMaxRad ||
        tibia < TibiaMinRad || tibia > TibiaMaxRad)
    {
        Console.WriteLine($"  [DEBUG] Failed: Joint limits exceeded");
        Console.WriteLine($"  [DEBUG]   Coxa: {ToDegrees(coxa):F2}° (limits: {ToDegrees(CoxaMinRad):F0}° to {ToDegrees(CoxaMaxRad):F0}°)");
        Console.WriteLine($"  [DEBUG]   Femur: {ToDegrees(femur):F2}° (limits: {ToDegrees(FemurMinRad):F0}° to {ToDegrees(FemurMaxRad):F0}°)");
        Console.WriteLine($"  [DEBUG]   Tibia: {ToDegrees(tibia):F2}° (limits: {ToDegrees(TibiaMinRad):F0}° to {ToDegrees(TibiaMaxRad):F0}°)");
        return null;
    }

    return (coxa, femur, tibia);
}

static double ToDegrees(double rad) => rad * 180.0 / Math.PI;

Console.WriteLine("====================================");
Console.WriteLine("INVERSE KINEMATICS VERIFICATION TEST");
Console.WriteLine("====================================\n");
Console.WriteLine($"Configuration: Coxa={CoxaLength * 1000:F1}mm, Femur={FemurLength * 1000:F1}mm, Tibia={TibiaLength * 1000:F1}mm");
Console.WriteLine($"Mount: Radius={BodyRadius * 1000:F1}mm, Angle={ToDegrees(MountAngle):F1}°\n");

// Test positions
var tests = new[]
{
    (Name: "Basic position", X: 180.0, Y: 180.0, Z: -45.0),
    (Name: "Max reach", X: 220.0, Y: 220.0, Z: -20.0),
    (Name: "Close position", X: 130.0, Y: 130.0, Z: -80.0),
    (Name: "Lifted foot", X: 170.0, Y: 170.0, Z: 30.0),
    (Name: "Rest position", X: 165.0, Y: 165.0, Z: -60.0)
};

foreach (var test in tests)
{
    Console.WriteLine($"Test: {test.Name}");
    Console.WriteLine($"  Target: ({test.X:F1}, {test.Y:F1}, {test.Z:F1}) mm");
    
    var target = new Vector3((float)(test.X / 1000.0), (float)(test.Y / 1000.0), (float)(test.Z / 1000.0));
    var result = InverseKinematics(target);
    
    if (result.HasValue)
    {
        Console.WriteLine($"  Result: ✓ VALID");
        Console.WriteLine($"    Coxa:  {ToDegrees(result.Value.Coxa):F2}° ({result.Value.Coxa:F4} rad)");
        Console.WriteLine($"    Femur: {ToDegrees(result.Value.Femur):F2}° ({result.Value.Femur:F4} rad)");
        Console.WriteLine($"    Tibia: {ToDegrees(result.Value.Tibia):F2}° ({result.Value.Tibia:F4} rad)");
        
        // Verify with forward kinematics
        var fkPos = ForwardKinematics(result.Value.Coxa, result.Value.Femur, result.Value.Tibia);
        var error = Vector3.Distance(target, fkPos) * 1000.0;
        
        Console.WriteLine($"  FK Check: ({fkPos.X * 1000:F1}, {fkPos.Y * 1000:F1}, {fkPos.Z * 1000:F1}) mm");
        Console.WriteLine($"  Error: {error:F4} mm {(error < 0.1 ? "✓ EXCELLENT" : error < 1.0 ? "⚠ ACCEPTABLE" : "✗ POOR")}");
    }
    else
    {
        Console.WriteLine($"  Result: ✗ UNREACHABLE");
    }
    Console.WriteLine();
}
