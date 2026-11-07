# FTC BIOBUZZ™ — Robot & Autonomous Systems Guide

This repository contains the complete control system, sensor pipelines, and autonomous programs for the **BioBuzz** FTC robot. The software stack is built using:
- **NextFTC** (`https://nextftc.dev/`): Command-based and event-driven architecture.
- **Pedro Pathing 3.0**: Dynamic Bézier curve following and trajectory generation.
- **goBILDA Pinpoint Odometry Computer**: High-frequency dual-pod localizer with onboard IMU.
- **Limelight 3A Vision Processor**: AprilTag detection, target pose inversion, and real-time vision re-localization.

---

## Table of Contents
1. [Robot Physical Architecture](#1-robot-physical-architecture)
2. [Field Layout & Game Geometry](#2-field-layout--game-geometry)
3. [AprilTag Clusters & Targeting Perspective](#3-apriltag-clusters--targeting-perspective)
4. [Gamepad Driver Controls (TeleOp)](#4-gamepad-driver-controls-teleop)
5. [Autonomous OpModes & Sequences](#5-autonomous-opmodes--sequences)
   - [Red Alliance Auto](#red-alliance-auto)
   - [Blue Alliance Auto](#blue-alliance-auto)
   - [Showcase Exhibition Tour Auto](#showcase-exhibition-tour-auto)
   - [Park-Only Auto](#park-only-auto)
6. [Telemetry & Dashboard Diagnostics](#6-telemetry--dashboard-diagnostics)

---

## 1. Robot Physical Architecture

The robot has an **$18.0″ \times 18.0″$ square chassis** ($457.2\text{ mm} \times 457.2\text{ mm}$). All four Mecanum wheels and both tracking pods are mounted flush along the outer perimeter edges.

### Top-Down Robot ASCII Map

```text
                                  FRONT (+Y)
                 +───────────────────────────────────────+
                 │ [Front Left Mecanum]  [Front Right Mecanum] │
                 │ (Outer Edge X=0.0")   (Outer Edge X=18.0")  │
                 │                                       │
                 │   [Left Launcher]       [Right Launcher]│
                 │   (Pollen Flywheel)    (Nectar Flywheel)│
                 │   Left Feeder Servo   Right Feeder Servo│
                 │                                       │
                 │           ┌─────────────────┐         │
                 │           │  LIMELIGHT 3A   │         │
                 │           │ (Angled ~110°)  │         │
                 │           └────────▲────────┘         │
                 │                    │                  │
                 │                  (0, 0)               │
                 │              [Robot Center]           │
                 │                                       │
                 │   ┌─────────┐           ┌─────────┐   │
-1.5" Back ──────┼───│  X Pod  │           │  Y Pod  │───┼─► Rolls Left/Right ◄►
(-38.1 mm)       │   │ (Forward│           │ (Strafe │   │   (+8.0" / +203.2 mm X)
                 │   │Tracking)│           │Tracking)│   │
                 │   └────▲────┘           └─────────┘   │
                 │        │                              │
                 │   (Rolls Fwd/Back)                    │
                 │   (-8.0" / -203.2 mm X)               │
                 │                                       │
                 │ [Rear Left Mecanum]   [Rear Right Mecanum]  │
                 +───────────────────────────────────────+
                                   REAR (-Y)
```

### Component Details
* **Dual Flywheel Launchers**:
  * **Pollen Launcher (Left)**: Target $1550\text{ RPM}$, Min $1500\text{ RPM}$.
  * **Nectar Launcher (Right)**: Target $1550\text{ RPM}$, Min $1500\text{ RPM}$.
* **Positional Feeder Servos**:
  * **Right Servo (Nectar)**: Rest $= 0.50$, Launch $= 0.15$.
  * **Left Servo (Pollen)**: Rest $= 0.15$, Launch $= 0.50$.
  * *Safety Interlock*: Servos will only actuate into launch position when the respective flywheel is confirmed at or above minimum target velocity ($\ge 1500\text{ RPM}$).
* **Tracking Pods (Pinpoint Computer)**:
  * Mounted symmetrically **$1.5″$ backward from center** ($Y = -38.1\text{ mm}$).
  * **X Pod (Forward Tracking)**: $8.0″$ left of center ($+203.2\text{ mm}$ lateral offset).
  * **Y Pod (Strafe Tracking)**: $8.0″$ right of center ($-38.1\text{ mm}$ longitudinal offset).
* **Limelight 3A**:
  * Rigidly mounted near the front center of the upper deck.
  * Pointed upward at an angle of **$\approx 110^\circ$** from horizontal (angled up $\approx 20^\circ$) to spot elevated Hive targets ($44″$ high) from close and mid range.

---

## 2. Field Layout & Game Geometry

The field is $144″ \times 144″$ ($6 \times 6$ grid of $24″$ soft foam tiles). The center Hive structure sits at $(72.0″, 72.0″)$. Around the perimeter walls are the **four (4) FLOWER assemblies** pre-staged with Pollen.

```text
Y (Inches)                                  [TOP WALL FLOWER] (X=96, Y=144)
144" +---------------+---------------+---------------+=== FLOWER ===-+---------------+---------------+
     |  [Col 1, R6]  |  [Col 2, R6]  |  [Col 3, R6]  |  [Col 4, R6]  |  [Col 5, R6]  |  [Col 6, R6]  |
     | (0 to 24")    | (24 to 48")   | (48 to 72")   | (72 to 96")   | (96 to 120")  | (120 to 144") |
     |               |               |  Red Launch 2 |               |  Blue Launch 2|               |
125" |               |               |   (55, 125)   |               |   (89, 125)   |               |
120" +---------------+---------------+---------------+---------------+---------------+---------------+
     |  [Col 1, R5]  |  [Col 2, R5]  |  [Col 3, R5]  |  [Col 4, R5]  |  [Col 5, R5]  |  [Col 6, R5]  |
105" |  RED PARK     |               | [Tags 30-33]  | [Tags 42-45]  |               |               |
     |  (13, 105)    |               | RED SCORING   | BLUE SCORING  |               |               |
 96" +=== FLOWER ===-+---------------+=== METAL BAR =+=== METAL BAR =+---------------+---------------+
     | [LEFT WALL]   |  [Col 2, R4]  |  [Col 3, R4]  |  [Col 4, R4]  |  [Col 5, R4]  |  [Col 6, R4]  |
     | (X=0, Y=96)   |               ║               |               ║               |               |
     |  LEFT OUTER   |  TRANSITION   ║     RED HIVE  |   BLUE HIVE   ║  TRANSITION   |  RIGHT OUTER  |
 72" |   CORRIDOR    |   CORRIDOR    ║     ASSEMBLY  |   ASSEMBLY    ║   CORRIDOR    |   CORRIDOR    |
     |  (0 to 24")   |  (24 to 48")  ║               |               ║  (96 to 120") | (120 to 144") |
     |               |               ║   Base Width: 49.46" (X axis) ║               |               |
 48" +---------------+---------------+=== METAL BAR =+=== METAL BAR =+---------------+=== FLOWER ===-+
     |  [Col 1, R2]  |  [Col 2, R2]  |  [Col 3, R2]  |  [Col 4, R2]  |  [Col 5, R2]  | [RIGHT WALL]  |
 39" |               |               | [Tags 34-37]  | [Tags 38-41]  |               |  (X=144, Y=48)|
     |               |               | RED AUDIENCE  | BLUE AUDIENCE |               |  BLUE PARK    |
     |               |               |               |               |               |  (131, 39)    |
 24" +---------------+=== FLOWER ===-+---------------+---------------+---------------+---------------+
     |  [Col 1, R1]  | [BOTTOM WALL] |  [Col 3, R1]  |  [Col 4, R1]  |  [Col 5, R1]  |  [Col 6, R1]  |
  9" |               |  (X=48, Y=0)  |  RED START    |  BLUE START   |               |               |
     |               |               |   (63, 9)     |   (81, 9)     |               |               |
  0" +---------------+---------------+---------------+---------------+---------------+---------------+
     0"             24"             48"             72"             96"            120"            144"
     ◄────── Col 1 ──►◄────── Col 2 ──►◄────── Col 3 ──►◄────── Col 4 ──►◄────── Col 5 ──►◄────── Col 6 ──►
                                     [ALLIANCE WALL]

### Flower Assembly Specifications & Locations:
* **Quantity**: Exactly **four (4) FLOWERS** attached to the perimeter walls over panel seams (2 tiles / 48″ in from each corner).
* **Locations**:
  1. **Alliance Wall Flower**: $X = 96.0″, Y = 0.0″$ (Intersection of Tile 4 and Tile 5).
  2. **Left Wall Flower**: $X = 0.0″, Y = 48.0″$ (Intersection of Tile 7 and Tile 13 against left wall).
  3. **Top (Scoring) Wall Flower**: $X = 48.0″, Y = 144.0″$ (Intersection of Tile 32 and Tile 33).
  4. **Right Wall Flower**: $X = 144.0″, Y = 96.0″$ (Intersection of Tile 24 and Tile 30 against right wall).
* **Physical Dimensions**:
  * **Top Opening**: $4.0″$ diameter circle positioned **$21.5″$ above the tile floor** with a $1.25″$ vertical backstop.
  * **Ground Clearance / Base Ring**: The Flower bottom ring mounts through the tile seam and features a $3.55″$ tall extraction opening at the bottom.
  * **Pre-Staging**: Pre-loaded with **4 yellow Pollen balls** inside each flower at match start.
* **Autonomous Avoidance**:
  * The Left Wall Flower sits at $(0, 96)$. Red Park is at $(13, 105)$ ($9″$ above the flower seam). The robot's $X=13″$ positioning gives a safe **$9″$ corridor** off the wall bracket.
  * The Right Wall Flower sits at $(144, 48)$. Blue Park is at $(131, 39)$ ($9″$ below the flower seam). The robot's $X=131″$ positioning keeps it safely clear of the bracket.

### Safety & Clearance Rules:
1. **Metal Ground Bars**: Extend along $X \approx 48.0″$ and $X \approx 96.0″$ from $Y \approx 52.5″$ to $91.5″$.
2. **Left Clear Corridor (Red Park Bypass)**: Always kept inside $X \le 24.0″$ ($X = 13″ \rightarrow 18″$) to provide $\ge 21″$ of clearance away from the metal bar and $\ge 4″$ off the perimeter wall.
3. **Right Clear Corridor (Blue Park Bypass)**: Always kept inside $X \ge 120.0″$ ($X = 126″ \rightarrow 131″$) to provide $\ge 21″$ clearance away from the metal bar and $\ge 4″$ off the perimeter wall.

---

## 3. AprilTag Clusters & Targeting Perspective

The rotating Hive Cells use **AprilTags (36h11 family, $3.25″ \times 3.25″$)** arranged in 4-tag clusters on the exterior underside of each Cell:

```text
       ◄──────────────────────── 10.25" Total Cluster Width ────────────────────────►
       ┌───────────┐      ┌───────────┐    ▲    ┌───────────┐      ┌───────────┐
       │   OUTER   │      │   INNER   │    │    │   INNER   │      │   OUTER   │
       │   LEFT    │      │   LEFT    │ Center  │   RIGHT   │      │   RIGHT   │
       │ (e.g. 34) │      │ (e.g. 35) │ Aimline │ (e.g. 36) │      │ (e.g. 37) │
       └───────────┘      └───────────┘    │    └───────────┘      └───────────┘
             ◄── 3.25" ──►       ◄ 1.875" ─┼─ 1.875" ►       ◄── 3.25" ──►
             ◄────── 5.125" ───────────────┼─────────────── 5.125" ──────►
```

| Cluster Name | Alliance | Hive Orientation | Tag IDs | Aim Centerline |
| :--- | :---: | :--- | :---: | :---: |
| **RED AUDIENCE** | Red | South Face (facing $Y=0″$) | **34, 35, 36, 37** | Midpoint between 35 & 36 |
| **RED SCORING** | Red | North Face (facing $Y=144″$) | **30, 31, 32, 33** | Midpoint between 31 & 32 |
| **BLUE AUDIENCE** | Blue | South Face (facing $Y=0″$) | **38, 39, 40, 41** | Midpoint between 39 & 40 |
| **BLUE SCORING** | Blue | North Face (facing $Y=144″$) | **42, 43, 44, 45** | Midpoint between 43 & 44 |

### Vision Re-Localization & Pose Estimation:
When an upright tag cluster is in view, `LimelightAimingSubsystem.estimateRobotPose()` reconstructs the robot's $(X, Y)$ coordinates and distance to Hive center:
$$\text{Dist to Hive Center} = \sqrt{(72.0 - X_{robot})^2 + (72.0 - Y_{robot})^2}$$
This soft-fuses (80% odometry / 20% vision) into the Pedro Pathing follower to continuously wipe out cumulative wheel slippage.

---

## 4. Gamepad Driver Controls (TeleOp)

The teleoperated mode supports **1-Driver or 2-Driver operation** automatically via `DriverControls.kt`. Controls only become active **after** the Driver Station Start button is pressed (preventing movement during INIT).

### Gamepad Visual Control Layouts

```text
================================ GAMEPAD 1: DRIVETRAIN & PRIMARY DRIVER ================================

                [LB: Intake Rev (Solo)]                       [RB: Intake Fwd (Solo)]
                [LT: Fire Pollen (Solo)]                      [RT: Fire Nectar (Solo)]
                             ┌───────────────────────────────┐
                             │       FTC LOGITECH / XBOX     │
                             │                               │
            (D-PAD)          │        [BACK]    [START]      │            (ACTION BUTTONS)
          ┌─────────┐        │       Reset Odom              │               ┌─────────┐
          │  ▲ Up   │        │                               │               │   (Y)   │  Flywheels (Solo)
    ◄─────┼─────────┼─────►  │     (LS)             (RS)     │         ◄─────┼─────────┼─────►
          │  ▼ Down │        │  Drive/Strafe      Turn/Rotate│               │   (A)   │  Flywheels (Solo)
          └─────────┘        │   (Forward/Rev)    (Spin L/R) │               └─────────┘
       Up/Down: Pulse Time   └───────────────────────────────┘            (X)/(B): Auxiliary
       Left/Right: Servo Pos     \                       /
                                  \                     /
                                   \                   /

================================ GAMEPAD 2: SUBSYSTEM OPERATOR (DUAL DRIVER) ============================

                [LB: Intake Reverse]                          [RB: Intake Forward]
                [LT: Fire Pollen (Speed Gated)]               [RT: Fire Nectar (Speed Gated)]
                             ┌───────────────────────────────┐
                             │       OPERATOR CONTROLLER     │
                             │                               │
            (D-PAD)          │        [BACK]    [START]      │            (ACTION BUTTONS)
          ┌─────────┐        │                               │               ┌─────────┐
          │  ▲ Up   │        │                               │               │   (Y)   │
    ◄─────┼─────────┼─────►  │                               │         ◄─────┼─────────┼─────►
          │  ▼ Down │        │                               │               │   (A)   │  Spin Up Flywheels
          └─────────┘        │                               │               └─────────┘  (1550 RPM Target)
     Up/Down: Nectar RPM     └───────────────────────────────┘
     Left/Right: Pollen RPM      \                       /
                                  \                     /
                                   \                   /
```

### Gamepad 1: Drivetrain & Primary Driver
| Control | Function | Details |
| :--- | :--- | :--- |
| **Left Stick Y** | Forward / Backward | Exponential curve with deadband |
| **Left Stick X** | Strafe Left / Right | Exponential curve with deadband |
| **Right Stick X** | Rotate Left / Right | Exponential curve with deadband |
| **Drive Mode** | Robot-Centric (Fixed) | Locked to Robot-Centric mode |
| **Button Back** | Reset Odometry Heading | Re-zeros robot heading to $0.0^\circ$ |
| **D-Pad Up / Down** | Adjust Pulse Duration *(Dual Driver)* / Nectar RPM *(Solo Driver)* | $\pm 10\text{ ms}$ or $\pm 50\text{ RPM}$ |
| **D-Pad Left / Right**| Adjust Servo Launch Position | Coerced in range $0.0 \rightarrow 1.0$ |
| **Button A / Y** *(Solo)*| Spin Up Flywheels | Both flywheels to $1550\text{ RPM}$ |
| **Right Trigger** *(Solo)*| Fire Nectar Launcher (Right) | Feed servo actuates when flywheel $\ge 1500\text{ RPM}$ |
| **Left Trigger** *(Solo)*| Fire Pollen Launcher (Left) | Feed servo actuates when flywheel $\ge 1500\text{ RPM}$ |
| **Right Bumper** *(Solo)*| Intake Forward | Power $= 1.0$ |
| **Left Bumper** *(Solo)*| Intake Reverse (Unjam) | Power $= -1.0$ |

### Gamepad 2: Subsystem Operator (When Connected)
| Control | Function | Details |
| :--- | :--- | :--- |
| **Button A** | Spin Up Flywheels | Spools Nectar & Pollen to $1550\text{ RPM}$ |
| **Right Trigger** | Fire Nectar Launcher (Right) | Gated by speed ($\ge 1500\text{ RPM}$) |
| **Left Trigger** | Fire Pollen Launcher (Left) | Gated by speed ($\ge 1500\text{ RPM}$) |
| **Right Bumper** | Intake Forward | Power $= 1.0$ |
| **Left Bumper** | Intake Reverse (Unjam) | Power $= -1.0$ |
---

## 5. Autonomous OpModes & Trajectory Maps

### Autonomous Motion & Safety Corridor Architecture
All automated driving uses **Pedro Pathing 3.0 Bézier curves** with dynamic lookahead sampling. The corridors are strictly engineered around the physical obstacles:
- **Central Hive / Submersible**: Ground metal bars located at $X \approx 48.0″$ and $X \approx 96.0″$ ($Y: 48″ \rightarrow 96″$).
- **Red Safe Corridor**: Locked to Tile Column 1 ($X \le 24.0″$). Robot center travels along $X = 13.0″ \rightarrow 18.0″$, guaranteeing $> 21″$ clearance from the left metal bar and $> 4″$ clearance off the left wall.
- **Blue Safe Corridor**: Locked to Tile Column 6 ($X \ge 120.0″$). Robot center travels along $X = 126.0″ \rightarrow 131.0″$, guaranteeing $> 21″$ clearance from the right metal bar and $> 4″$ clearance off the right wall.

---

### Red Alliance Auto (`AprilTagRedAutoOpMode`)

#### Simplified Trajectory Map
```text
Y (Inches) ^ ===================== [TOP FLOWER (96, 144)] =====================
           |
    125" - |                 [OPPOSITE LAUNCH]      [INTERMEDIATE]
           |                 (55, 125) ◄─────────── (63, 125)
           |                 Aim: 278.0° (v)        Heading: 90.0° (^)
           |                 [Shoot 5.0s]                  ▲
           |                      │                        │
    105" - |  [RED PARK]          │                        │ (Branch B: Straight North
           |  (13, 105)           │                        │  along X = 63.0" line)
           |  Heading: 90° / 270° │                        │
           |  ◄───────────────────┘ (Branch B Line)        │
     96" - | [LEFT FLOWER]                                 ║ GROUND BAR
           | (0, 96)                                       ║ (ON Y-AXIS)
           |      ▲                                        ║ X = 48.0"
     75" - |      │  Ctrl 2                                ║ Y: 48" -> 96"
           |      │  (15, 75)                              ║ [24" lateral buffer]
           |      (  [Safe Outer Corridor]                 ║
           |       \                                       ║
     48" - |        \                                      ║
           |         `──_  Ctrl 1                          ║
     24" - |               `── (20, 24)                    │
           |                    `──_                       │
      9" - |   (Branch A: Bézier    `───────────────────── [RED START / SEARCH]
           |    S-Curve avoiding bar)                      (63, 9)
           |                                               Aim: 94.3° | Shoot 5.0s
      0" --+==================== [BOTTOM FLOWER (48, 0)] ======================> X (Inches)
           0"   13" 15" 20"          48"                 63" 72"                 144"
         (Left Wall)                                       (Center)
```

#### Routine Steps:
* **Start Pose**: `(63.0, 9.0, 90.0°)` (Back-right wheel flush with $X=72.0″$).
1. **Tag Search (0–5s)**: Searches for Red tags (31–37).
2. **Branch A (Tag Detected at Start)**:
   - Rotates in-place from $90.0^\circ \rightarrow 94.3^\circ$ to center on Red Audience Cell.
   - Launches elements for **5.0 seconds**; flywheels/intake shut down completely.
   - Realigns back to $90.0^\circ$.
   - Follows Bézier curve sweeping through $(20, 24)$ and up along the outer corridor through $(15, 75)$ ($24.0″$ lateral clearance to the ground bar at $X=48″$).
   - Curves into **Red Park $(13.0, 105.0)$**, safely $9″$ above the Left Wall Flower at $(0, 96)$.
3. **Branch B (No Tag Detected)**:
   - Follows straight path North along $X=63.0″$ ($15.0″$ right of the ground bar at $X=48″$).
   - Reaches $(63.0, 125.0)$ (above the top edge of the ground bar at $Y=96″$).
   - Curves into Opposite Launch Pose **$(55.0, 125.0, 270.0°)$**.
   - Rotates in-place to $278.0^\circ$ toward Red Scoring Cell, launches 5.0 seconds, realigns to $270.0^\circ$.
   - Curves down along the perimeter via $(28, 120)$ and $(15, 112)$ into **Red Park $(13.0, 105.0)$**.
4. **Match Cutoff**: Automatically cuts active launching and proceeds to park when 10 seconds remain in auto.

---

### Blue Alliance Auto (`AprilTagBlueAutoOpMode`)

#### Simplified Trajectory Map
```text
Y (Inches) ^ ===================== [TOP FLOWER (96, 144)] =====================
           |
    125" - |                 [INTERMEDIATE]        [OPPOSITE LAUNCH]
           |                 (81, 125) ───────────► (89, 125)
           |                 Heading: 90.0° (^)    Aim: 262.0° (v)
           |                        ▲              [Shoot 5.0s]
           |                        │                   │
           | (Branch B: Straight    │                   │
           |  North along X=81.0")  │                   │  (Branch B: Bézier sweep
           |                        │                   ║   avoiding ground bar)
     96" - |                        │                   ║
           |                        │                   ║ GROUND BAR (ON Y-AXIS)
     75" - |                        │                   ║ X = 96.0", Y: 48" -> 96"
           |                        │                   ║ Blue Ctrl 1: (129, 75)
           |                        │                   ║ [24" lateral buffer]
           |                        │                   ║            │
     48" - |                        │                   ║ [RIGHT     │
           |                        │                   ║  FLOWER]   │Blue Ctrl 2:
     39" - |                        │                   ║ (144, 48)  │(129, 45)
           |                        │                   ║            ▼
           |                        │                   ║       [BLUE PARK]
           |                        │                           (131, 39)
           |                        │                           Heading: 90° / 270°
           |                        │                                ▲
           |                        │                                │ (Branch A Direct Path)
      9" - |                        [BLUE START/SEARCH]──────────────┴───────────►
           |                        (81, 9)
           |                        Aim: 85.7° | Shoot 5.0s
      0" --+==================== [BOTTOM FLOWER (48, 0)] ======================> X (Inches)
           0"                      48" 72" 81"         89" 96"      129" 131"     144"
         (Left Wall)                 (Center)                                  (Right Wall)
```

#### Routine Steps:
* **Start Pose**: `(81.0, 9.0, 90.0°)` in Tile `[Col 4, Row 1]` (Back-left wheel flush with $X=72.0″$).
1. **Tag Search (0–5s)**: Searches for Blue tags (38–45).
2. **Branch A (Tag Detected at Start)**:
   - Rotates from $90.0^\circ \rightarrow 85.7^\circ$ to center on Blue Audience Cell.
   - Launches elements for **5.0 seconds**, shuts off, realigns to $90.0^\circ$.
   - Drives directly rightward into **Blue Park $(131.0, 39.0)$** (staying below the ground bar at $Y=48″$ and $9″$ below the Right Flower at $(144, 48)$).
3. **Branch B (No Tag Detected)**:
   - Follows path North along $X=81.0″$ through Tile `[Col 4]` ($15.0″$ left of the ground bar at $X=96″$).
   - Reaches $(81.0, 125.0)$ (above the top edge of the ground bar at $Y=96″$).
   - Curves into Opposite Launch Pose **$(89.0, 125.0, 270.0°)$**.
   - Rotates in-place to $262.0^\circ$ toward Blue Scoring Cell, launches 5.0 seconds, realigns to $270.0^\circ$.
   - Follows Bézier curve around the right side of the ground bar through $(129, 75)$ and $(129, 45)$ (giving $24.0″$ lateral clearance to the bar at $X=96″$) into **Blue Park $(131.0, 39.0)$**.

---

## 5. Active Driver Station Autonomous OpModes & Valid Starting Locations

To keep the Driver Station dropdown menu completely clean and uncluttered, there are **only two primary match OpModes**:

### 1. 🔴 Red Alliance Autonomous (`AprilTagRedAutoOpMode`)
* **Valid Starting Locations**:
  1. **Tile 2 Angled Start**: **`(39.00″, 16.00″, 82.0°)`** (Tile `[Col 2, Row 1]`, BL wheel 7″ from wall $Y=0$, BR wheel 7″ from $X=48$ seam).
  2. **Tile 3 Straight North Start**: **`(63.00″, 9.00″, 90.0°)`** (Tile `[Col 3, Row 1]`, facing straight North).

### 2. 🔵 Blue Alliance Autonomous (`AprilTagBlueAutoOpMode`)
* **Valid Starting Locations**:
  1. **Tile 35 Top Angled Start**: **`(105.00″, 128.00″, 262.0°)`** (Tile `[Col 5, Row 6]`, facing Southwest $262.0^\circ$ toward Blue Scoring Cell).
  2. **Tile 34 Top Straight South Start**: **`(89.00″, 135.00″, 270.0°)`** (Tile `[Col 4, Row 6]`, facing straight South $270.0^\circ$).

---

### Showcase Exhibition Tour Auto (`ShowcaseExhibitionAutoOpMode`)
An extended demonstration mode unconstrained by match time:
1. **Launch 1** at Start $(63, 9)$ facing North (4s).
2. **Bézier curve** around the Hive into **Red Park $(13, 105)$** (pauses 1.5s).
3. **Bézier curve** up to **Opposite Launch $(55, 125)$**, turning to $270^\circ$.
4. **Launch 2** at $(55, 125)$ facing South (4s).
5. **Bézier curve** down the entire right perimeter into **Blue Park $(131, 39)$** (pauses 1.5s).
6. **Bézier curve** along the alliance corridor back home to **Start $(63, 9)$**!

---

### Park-Only Auto (`ParkAutoOpMode`)
* Focuses exclusively on driving the perimeter parking trajectory with individual status updates per segment:
  - `Step 1`: Moving to Waypoint 1 `(24, 24)`
  - `Step 2`: Moving along perimeter to Waypoint 2 `(24, 105)`
  - `Step 3`: Parking at End Position `(13, 105)`

---

## 6. Telemetry & Dashboard Diagnostics

Access live robot camera feed, graph plots, and field canvas overlays via FTC Dashboard at:
```text
http://192.168.43.1:8080/dash
```

### Live Streamed Variables & Field Overlays:
* **Position & Odometry**: Real-time $(X, Y)$ and Heading plotted on the interactive field overlay.
* **AprilTag Status**: Detected Tag ID, horizontal angle offset (`tx`), 3D distance, and calculated **`Dist to Hive Center`**.
* **Subsystem Telemetry**: Real-time Nectar and Pollen launcher motor velocities (vs. target velocity), feed servo positions, and intake power.
* **Limelight Diagnostics**: Framerate (FPS), CPU usage, targeting latency, and camera MJPEG stream.
