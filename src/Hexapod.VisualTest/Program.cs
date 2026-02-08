using System.Numerics;
using System.Text.Json;
using Hexapod.Movement.Kinematics;

var builder = WebApplication.CreateBuilder(args);

builder.Services.AddCors();
builder.Services.Configure<Microsoft.AspNetCore.Http.Json.JsonOptions>(o =>
    o.SerializerOptions.PropertyNamingPolicy = null); // preserve PascalCase

var app = builder.Build();

app.UseCors(policy => policy.AllowAnyOrigin().AllowAnyMethod().AllowAnyHeader());
app.UseStaticFiles();

// Load kinematics config from appsettings (values in mm, convert to meters for IK engine)
var kinSection = app.Configuration.GetSection("Hexapod:Kinematics");
var renderSection = app.Configuration.GetSection("Hexapod:Rendering");

double coxaMm = kinSection.GetValue<double>("CoxaLength", 40.0);
double femurMm = kinSection.GetValue<double>("FemurLength", 60.0);
double tibiaMm = kinSection.GetValue<double>("TibiaLength", 135.0);
double bodyRadiusMm = kinSection.GetValue<double>("BodyRadius", 90.0);
double defaultHeightMm = kinSection.GetValue<double>("DefaultHeight", 45.0);

// Rendering dimensions (mm)
double bodyThicknessMm = renderSection.GetValue<double>("BodyThicknessMm", 12.0);
double coxaDiameterMm = renderSection.GetValue<double>("CoxaDiameterMm", 10.0);
double femurDiameterMm = renderSection.GetValue<double>("FemurDiameterMm", 8.0);
double tibiaDiameterMm = renderSection.GetValue<double>("TibiaDiameterMm", 6.0);
double jointDiameterMm = renderSection.GetValue<double>("JointDiameterMm", 12.0);
double footDiameterMm = renderSection.GetValue<double>("FootDiameterMm", 8.0);

var body = new HexapodBody(coxaMm / 1000.0, femurMm / 1000.0, tibiaMm / 1000.0, bodyRadiusMm / 1000.0);

// --- API endpoints ---

// GET /api/config — body, leg geometry, rendering dimensions
app.MapGet("/api/config", () =>
{
    var legs = body.Legs.Select(l => new
    {
        l.LegId,
        l.Name,
        MountAngleDeg = l.MountAngle * 180.0 / Math.PI,
        MountRadiusMm = l.MountRadius * 1000.0,
        CoxaLengthMm = l.CoxaLength * 1000.0,
        FemurLengthMm = l.FemurLength * 1000.0,
        TibiaLengthMm = l.TibiaLength * 1000.0,
        CoxaLimits = new { MinDeg = l.CoxaLimits.Min * 180 / Math.PI, MaxDeg = l.CoxaLimits.Max * 180 / Math.PI },
        FemurLimits = new { MinDeg = l.FemurLimits.Min * 180 / Math.PI, MaxDeg = l.FemurLimits.Max * 180 / Math.PI },
        TibiaLimits = new { MinDeg = l.TibiaLimits.Min * 180 / Math.PI, MaxDeg = l.TibiaLimits.Max * 180 / Math.PI }
    });

    return Results.Ok(new
    {
        Body = new
        {
            RadiusMm = bodyRadiusMm,
            ThicknessMm = bodyThicknessMm,
            DefaultHeightMm = defaultHeightMm
        },
        Rendering = new
        {
            CoxaDiameterMm = coxaDiameterMm,
            FemurDiameterMm = femurDiameterMm,
            TibiaDiameterMm = tibiaDiameterMm,
            JointDiameterMm = jointDiameterMm,
            FootDiameterMm = footDiameterMm
        },
        Legs = legs
    });
});

// POST /api/ik — compute IK for one leg
app.MapPost("/api/ik", (IkRequest req) =>
{
    if (req.LegId < 0 || req.LegId >= body.Legs.Count)
        return Results.BadRequest("Invalid leg ID");

    var leg = body.Legs[req.LegId];
    var target = new Vector3((float)(req.X / 1000.0), (float)(req.Y / 1000.0), (float)(req.Z / 1000.0));
    var result = leg.InverseKinematics(target);

    if (result is null)
        return Results.Ok(new IkResponse(false, null, null, "Position unreachable"));

    // Compute joint positions for visualization via FK segments
    var joints = ComputeJointPositions(leg, result.Value.Coxa, result.Value.Femur, result.Value.Tibia);

    var angles = new JointAngles(
        result.Value.Coxa * 180 / Math.PI,
        result.Value.Femur * 180 / Math.PI,
        result.Value.Tibia * 180 / Math.PI);

    return Results.Ok(new IkResponse(true, angles, joints, null));
});

// POST /api/fk — compute FK from joint angles
app.MapPost("/api/fk", (FkRequest req) =>
{
    if (req.LegId < 0 || req.LegId >= body.Legs.Count)
        return Results.BadRequest("Invalid leg ID");

    var leg = body.Legs[req.LegId];
    var coxa = req.CoxaDeg * Math.PI / 180;
    var femur = req.FemurDeg * Math.PI / 180;
    var tibia = req.TibiaDeg * Math.PI / 180;

    var joints = ComputeJointPositions(leg, coxa, femur, tibia);
    var foot = leg.ForwardKinematics(coxa, femur, tibia);

    return Results.Ok(new
    {
        FootMm = new Vec3(foot.X * 1000, foot.Y * 1000, foot.Z * 1000),
        Joints = joints
    });
});

// Fallback to index.html
app.MapFallbackToFile("index.html");

app.Run();

// --- Helpers ---

static JointPositions ComputeJointPositions(HexapodLeg leg, double coxa, double femur, double tibia)
{
    var mountX = leg.MountRadius * Math.Cos(leg.MountAngle);
    var mountY = leg.MountRadius * Math.Sin(leg.MountAngle);

    // Body center
    var bodyCenter = new Vec3(0, 0, 0);

    // Mount point (coxa joint)
    var coxaJoint = new Vec3(mountX * 1000, mountY * 1000, 0);

    // End of coxa / start of femur
    var totalAngle = leg.MountAngle + coxa;
    var femurJointX = mountX + leg.CoxaLength * Math.Cos(totalAngle);
    var femurJointY = mountY + leg.CoxaLength * Math.Sin(totalAngle);
    var femurJoint = new Vec3(femurJointX * 1000, femurJointY * 1000, 0);

    // End of femur / start of tibia
    var tibiaJointX = femurJointX + leg.FemurLength * Math.Cos(femur) * Math.Cos(totalAngle);
    var tibiaJointY = femurJointY + leg.FemurLength * Math.Cos(femur) * Math.Sin(totalAngle);
    var tibiaJointZ = leg.FemurLength * Math.Sin(femur);
    var tibiaJoint = new Vec3(tibiaJointX * 1000, tibiaJointY * 1000, tibiaJointZ * 1000);

    // Foot (end of tibia)
    var tibiaRelative = -tibia;
    var tibiaHoriz = leg.TibiaLength * Math.Cos(femur + tibiaRelative);
    var tibiaVert = leg.TibiaLength * Math.Sin(femur + tibiaRelative);
    var footX = femurJointX + (leg.FemurLength * Math.Cos(femur) + tibiaHoriz) * Math.Cos(totalAngle);
    var footY = femurJointY + (leg.FemurLength * Math.Cos(femur) + tibiaHoriz) * Math.Sin(totalAngle);
    var footZ = leg.FemurLength * Math.Sin(femur) + tibiaVert;
    var foot = new Vec3(footX * 1000, footY * 1000, footZ * 1000);

    return new JointPositions(bodyCenter, coxaJoint, femurJoint, tibiaJoint, foot);
}

// --- Records ---

record IkRequest(int LegId, double X, double Y, double Z);
record FkRequest(int LegId, double CoxaDeg, double FemurDeg, double TibiaDeg);
record Vec3(double X, double Y, double Z);
record JointAngles(double CoxaDeg, double FemurDeg, double TibiaDeg);
record JointPositions(Vec3 BodyCenter, Vec3 CoxaJoint, Vec3 FemurJoint, Vec3 TibiaJoint, Vec3 Foot);
record IkResponse(bool Reachable, JointAngles? Angles, JointPositions? Joints, string? Error);
