# Kinematics Configuration Reference

Visual guide for every parameter in the `Hexapod.Kinematics` configuration section.

All configuration lives in [`config/hexapod.json`](../config/hexapod.json).
Environment-specific overrides can be placed in `config/hexapod.{Environment}.json`
(e.g. `hexapod.Development.json`). Later files override earlier values.

---

## Configuration Loading Order

```
hexapod.json                          ← shared base (all environments)
hexapod.{Environment}.json            ← environment override (optional)
appsettings.json                      ← project-specific (optional)
appsettings.{Environment}.json        ← project + environment (optional)
```

Each subsequent file merges into the previous, so you only need to specify
the values you want to override.

---

## Table of Contents

| Section | Description |
|---------|-------------|
| [Coordinate System](#coordinate-system) | Right-hand rule axes used throughout |
| [DefaultHeight](#defaultheight) | Standing height above ground |
| [MaxReachRadius](#maxreachradius) | Workspace bounding radius |
| [Body](#body) | Body plate parameters |
| [Legs](#legs) | Per-leg mount points and segment dimensions |
| [JointLimits](#jointlimits) | Servo angle constraints |

---

## Coordinate System

The hexapod uses a **right-hand coordinate frame**:

```
          Z (up)
          │
          │
          │
          └──────── X (forward)
         ╱
        ╱
       Y (left)
```

- **X** — forward (front of robot)
- **Y** — left (right-hand rule)
- **Z** — up

All angles follow the right-hand rule: **positive = counter-clockwise** when
viewed from above (Z-axis). Right-side legs have negative mount angles,
left-side legs have positive mount angles.

---

## DefaultHeight

```json
"DefaultHeight": 45.0
```

The nominal standing height of the body center above the ground plane, in mm.
Used by the gait engine to compute default foot Z targets.

```
                    ┌─────────────────┐
                    │     Body        │
                    └────────┬────────┘
                             │
                 DefaultHeight│ 45 mm
                             │
    ═════════════════════════╧════════════════  Ground
```

---

## MaxReachRadius

```json
"MaxReachRadius": 200.0
```

The maximum horizontal distance (in mm) from the body center that any foot
can reach. Used for workspace bounding checks and gait planning.

```
    Top view:

                     MaxReachRadius
              ◄────────────────────────►

              ╭╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╮
            ╌╌                           ╌╌
          ╌╌         ┌─────────┐           ╌╌
         ╌      ╱────┤  Body   ├────╲        ╌
        ╌    leg     └─────────┘     leg      ╌
        ╌      ╲                   ╱          ╌
         ╌      ╲─────────────────╱          ╌
          ╌╌                               ╌╌
            ╌╌                           ╌╌
              ╰╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╯

         Dashed circle = MaxReachRadius workspace boundary
```

---

## Body

```json
"Body": {
    "ThicknessMm": 12.0,
    "Description": "..."
}
```

### ThicknessMm

The vertical thickness of the body plate in mm. The body top surface sits at
Z = 0 in the robot frame; the bottom surface is at Z = −ThicknessMm.

```
    Side view:

                ThicknessMm = 12 mm
              ┌──────────────────────┐ ← Z = 0  (body top)
              │       Body plate     │
              └──────────────────────┘ ← Z = −12 (body bottom)
                         │
                    DefaultHeight
                         │
    ═════════════════════╧══════════════  Ground
```

---

## Legs

Each leg is defined independently, allowing irregular hexagonal body shapes.

```json
"Legs": [
    {
        "Name": "FrontRight",
        "MountAngleDeg": -30.0,
        "MountRadiusMm": 90.0,
        "CoxaLengthMm": 40.0,
        "FemurLengthMm": 60.0,
        "TibiaLengthMm": 135.0
    },
    ...
]
```

### Leg Ordering & Names

Legs are ordered clockwise starting from front-right when viewed from above:

```
    Top view (looking down):

                      X (forward)
                      ▲
                     ╱ ╲
           FL  30°  ╱   ╲  −30°  FR        Index 5         Index 0
                   ╱     ╲
                  ╱ Body  ╲
    ML  90° ────┤  center  ├──── −90°  MR   Index 4         Index 1
                  ╲       ╱
                   ╲     ╱
           RL 150°  ╲   ╱  −150° RR         Index 3         Index 2
                     ╲ ╱
                      ▼
```

| Index | Name | MountAngleDeg |
|:-----:|------|:-------------:|
| 0 | FrontRight  |  −30° |
| 1 | MiddleRight | −90° |
| 2 | RearRight   | −150° |
| 3 | RearLeft    | +150° |
| 4 | MiddleLeft  |  +90° |
| 5 | FrontLeft   |  +30° |

---

### MountAngleDeg

The angle from the body's forward axis (X) to the leg mount point,
measured in degrees. Positive = counter-clockwise (left side).

```
    Top view:

                        X (forward)
                        ▲
                       ╱│
                      ╱ │
                     ╱  │
           Mount ●  ╱ angle = −30°
          point    ╱    │
                  ╱     │
                 ╱      │
                ● Body center
```

---

### MountRadiusMm

The distance from the body center to the leg mount point, in mm.
Each leg can have a **different** radius, enabling irregular (elongated)
hexagonal body shapes.

```
    Top view — regular hexagon (all radii = 90 mm):

                  ●─────────●
                 ╱     90    ╲
                ╱             ╲
               ●    ● center   ●
                ╲             ╱
                 ╲     90    ╱
                  ●─────────●


    Top view — elongated hexagon (front/rear = 110 mm, middle = 80 mm):

                ●─────────────────●
               ╱   110          110╲
              ╱                     ╲
             ● 80    ● center   80   ●
              ╲                     ╱
               ╲   110          110╱
                ●─────────────────●
```

---

### Leg Segments: CoxaLengthMm, FemurLengthMm, TibiaLengthMm

Each leg consists of three rigid segments connected by revolute joints:

```
    Side view of a single leg (femur=0°, tibia=90°):

                           CoxaLengthMm         FemurLengthMm
                         ◄──── 40 mm ────►  ◄───── 60 mm ─────►
    ┌─────────┐
    │  Body   │         ●──────────────────●
    └─────────┘    Coxa joint          Femur joint
                   (mount point)            │
                                            │ TibiaLengthMm
                                            │    135 mm
                                            │
                                            │
                                            │
                                            ● Foot
    ══════════════════════════════════════════════════════════  Ground
```

```
    Top view — Coxa rotates the leg around the mount point:

                          Coxa swing
                        ╱  (±45°)
                       ╱
           ┌──────────●────────────── ●──── ● ── ● ←foot
           │  Body    │ mount      coxa   femur  tibia
           │          │ point    segment  joint  joint
           └──────────┘

    Coxa angle = 0° → leg extends radially outward from mount point
```

```
    Side view — Femur & Tibia articulation:

                   Femur = +45°             Femur = 0°            Femur = −30°

                       ● femur joint          ● femur joint        ● femur joint
                      ╱                       │                     ╲
                     ╱ femur                  │ femur                ╲ femur
                    ╱                         │                       ╲
                   ● tibia joint              ● tibia joint            ● tibia joint
                   │                          │                         ╲
                   │ tibia                    │ tibia                    ╲ tibia
                   │                          │                           ╲
                   ● foot                     ● foot                      ● foot
```

---

## JointLimits

```json
"JointLimits": {
    "CoxaMinDeg": -45.0,
    "CoxaMaxDeg": 45.0,
    "FemurMinDeg": -60.0,
    "FemurMaxDeg": 90.0,
    "TibiaMinDeg": 30.0,
    "TibiaMaxDeg": 150.0
}
```

Applied globally to all legs. The IK engine rejects solutions outside these
ranges. `SetJointAngles()` clamps to these limits.

### Coxa Limits (−45° to +45°)

Controls horizontal leg swing around the mount point.

```
    Top view from above:

                             X (forward)
                             ▲
                  +45°      │      −45°
                     ╲      │      ╱
                      ╲     │     ╱
                       ╲    │    ╱
                        ╲   │   ╱
         Max CCW ────────●──●──●──────── Max CW
                         mount point
                     ◄── 90° range ──►
```

### Femur Limits (−60° to +90°)

Controls vertical tilt of the upper leg. 0° = horizontal.

```
    Side view:

                +90° (straight up)
                  │
                  │       ╱ +45°
                  │     ╱
                  │   ╱
     femur joint  ● ────────── 0° (horizontal)
                  │ ╲
                  │   ╲
                  │     ╲ −60° (reaching down/back)
```

### Tibia Limits (30° to 150°)

The tibia angle uses a **servo-friendly convention**: 0° = leg fully
straight (tibia continues inline with femur), increasing angle = tibia
folds toward femur.

```
    Side view:

    femur joint ●
                │
                │ femur
                │
   tibia joint  ●─────────── 0° (straight, not reachable — below min)
                │╲
                │  ╲ 30° (min — nearly straight)
                │    ╲
                │      ╲
                │        ╲ 90° (right angle — default standing)
                │          │
                │          │
                │          ╲
                │            ╲  150° (max — folded tight)
```

---

## Rendering

```json
"Rendering": {
    "CoxaDiameterMm": 10.0,
    "FemurDiameterMm": 8.0,
    "TibiaDiameterMm": 6.0,
    "JointDiameterMm": 12.0,
    "FootDiameterMm": 8.0
}
```

These are **visual-only** parameters for the 3D VisualTest viewer. They do
not affect kinematics calculations.

```
                  JointDiameterMm = 12
                      ╭──╮
                      │●─│─────────────── CoxaDiameterMm = 10
                      ╰──╯               (cylinder radius = 5)
                        │
                  ╭─────┼─────╮
                  │     ●     │────────── FemurDiameterMm = 8
                  ╰─────┼─────╯
                        │
                  ╭─────┼─────╮
                  │     ●     │────────── TibiaDiameterMm = 6
                  ╰─────┼─────╯
                        │
                      ╭──╮
                      │●─│───────────────  FootDiameterMm = 8
                      ╰──╯
```

---

## Full JSON Example

See the complete configuration in [`config/hexapod.json`](../config/hexapod.json).

To create an environment override (e.g. for a different robot frame):

```jsonc
// config/hexapod.Development.json
{
  "Hexapod": {
    "Kinematics": {
      "Legs": [
        { "MountRadiusMm": 110.0 },    // FrontRight — larger frame
        { "MountRadiusMm": 80.0 },     // MiddleRight
        { "MountRadiusMm": 110.0 },    // RearRight
        { "MountRadiusMm": 110.0 },    // RearLeft
        { "MountRadiusMm": 80.0 },     // MiddleLeft
        { "MountRadiusMm": 110.0 }     // FrontLeft
      ]
    }
  }
}
```

Only the overridden values need to be specified — all other fields
inherit from the base `hexapod.json`.
