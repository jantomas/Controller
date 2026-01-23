// Quick calculation of reachable positions for Hexapod Leg 0

// Configuration (in meters)
double coxaLength = 0.040;   // 40mm
double femurLength = 0.060;  // 60mm
double tibiaLength = 0.135;  // 135mm
double bodyRadius = 0.090;   // 90mm

// Leg 0 mount angle: -30 degrees
double mountAngle = -30.0 * Math.PI / 180.0;

// Joint limits
var coxaLimits = (-Math.PI / 4, Math.PI / 4);      // -45° to +45°
var femurLimits = (-Math.PI / 3, Math.PI / 2);     // -60° to +90°
var tibiaLimits = (-Math.PI / 2, Math.PI / 6);     // -90° to +30°

// Forward Kinematics function
(double x, double y, double z) ForwardKinematics(double coxa, double femur, double tibia)
{
    var legLength = coxaLength + 
                   femurLength * Math.Cos(femur) + 
                   tibiaLength * Math.Cos(femur + tibia);
    
    var z = femurLength * Math.Sin(femur) + 
            tibiaLength * Math.Sin(femur + tibia);

    var totalAngle = mountAngle + coxa;
    var x = (bodyRadius + legLength) * Math.Cos(totalAngle);
    var y = (bodyRadius + legLength) * Math.Sin(totalAngle);

    return (x, y, z);
}

// Calculate default position (all angles = 0)
var defaultPos = ForwardKinematics(0, 0, 0);
Console.WriteLine($"Default position (coxa=0, femur=0, tibia=0):");
Console.WriteLine($"  X = {defaultPos.x * 1000:F1} mm");
Console.WriteLine($"  Y = {defaultPos.y * 1000:F1} mm");
Console.WriteLine($"  Z = {defaultPos.z * 1000:F1} mm");

// Sample various angle combinations within limits
Console.WriteLine("\nSample reachable positions:");
var samples = new[]
{
    (0.0, 0.0, 0.0),
    (0.0, 0.3, -0.5),   // femur up, tibia bent
    (0.0, -0.3, -0.3),  // femur down, tibia bent
    (0.2, 0.0, 0.0),    // coxa rotated
    (-0.2, 0.0, 0.0),   // coxa rotated other way
    (0.0, 0.5, -1.0),   // more extreme
};

foreach (var (c, f, t) in samples)
{
    if (c >= coxaLimits.Item1 && c <= coxaLimits.Item2 &&
        f >= femurLimits.Item1 && f <= femurLimits.Item2 &&
        t >= tibiaLimits.Item1 && t <= tibiaLimits.Item2)
    {
        var pos = ForwardKinematics(c, f, t);
        Console.WriteLine($"  Angles ({c*180/Math.PI:F0}°, {f*180/Math.PI:F0}°, {t*180/Math.PI:F0}°) => X={pos.x*1000:F1}, Y={pos.y*1000:F1}, Z={pos.z*1000:F1}");
    }
}
