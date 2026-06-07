# FTC Autonomous Stage Simulator & Tactical Evaluator

### DECODE<sup>SM</sup> presented by RTX &bull; 2025–2026 Season

*A high-fidelity 3D digital twin for pre-match autonomous strategy validation,
built on real FTC hardware specifications and the official Onshape CAD assembly
(am-5700).*

---

## Motivation

FTC teams invest hundreds of hours designing, building, and programming their
robots — yet **autonomous-stage debugging remains a bottleneck**.  Physical field
time is scarce; sensor noise, battery sag, and mechanical variance make
reproducible testing difficult.  Coaches and drivers cannot easily answer
questions like:

- *Will our 18-inch chassis clear the obelisk when approaching the classifier
  from the alliance station?*
- *Does our flywheel launcher, spinning at 312 RPM with a 45° elevation angle,
  land the relic inside the 8-inch-radius goal rim at 38.75 inches?*
- *What happens if we switch from goBILDA 19.2:1 motors to 30:1 — does the
  speed loss cost us a scoring opportunity in the 30-second window?*

This simulator answers those questions **before the robot ever touches the
field**.  It combines A\* pathfinding, continuous collision detection,
Mecanum kinematics, and 3D projectile physics into a single real-time
environment driven by a JavaFX 3D viewport.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    SimulatorApp (JavaFX UI)              │
│  ┌──────────────┐  ┌──────────────────────────────────┐ │
│  │ Control Panel │  │  3D SubScene (PerspectiveCamera) │ │
│  │ - Dimensions  │  │  - Field mesh + 6×6 tile grid   │ │
│  │ - Motor picker│  │  - Triangular obelisk (MeshView) │ │
│  │ - OpMode text │  │  - Perimeter walls (11 panels)  │ │
│  │ - .java upload│  │  - Robot Box + trail + A* path  │ │
│  │ - Start/Reset │  │  - Ball Sphere + parabola trail │ │
│  └──────────────┘  └──────────────────────────────────┘ │
├─────────────────────────────────────────────────────────┤
│                AutoSimulationEngine                      │
│  ┌──────────┐  ┌────────────┐  ┌──────────────────────┐ │
│  │ Cmd Parser│  │ A* PathFind│  │ BallProjectileEngine │ │
│  │ MOVE_TO   │  │ 8-dir grid │  │ g = 386.09 in/s²    │ │
│  │ INTAKE    │  │ Diagonal h │  │ elastic rim bounce   │ │
│  │ LAUNCH    │  │ Bresenham  │  │ goal-plane scoring   │ │
│  │ DETECT    │  │ LOS smooth │  │ 3D trajectory log    │ │
│  │ WAIT      │  └────────────┘  └──────────────────────┘ │
│  └──────────┘                                            │
├─────────────────────────────────────────────────────────┤
│                     Model Layer                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐ │
│  │Robot     │  │RobotProf.│  │FieldMap  │  │MotorType│ │
│  │kinematics│  │motor spec│  │144×144   │  │RPM+torque│ │
│  │collision │  │launcher  │  │grid+cad  │  │enum     │ │
│  └──────────┘  └──────────┘  └──────────┘  └─────────┘ │
└─────────────────────────────────────────────────────────┘
```

### MVC Separation

| Layer | Package | Responsibility |
|-------|---------|---------------|
| **Model** | `simulator.model` | Robot kinematics, `RobotProfile` (motor specs, dimensions, capabilities), `FieldMap` (144×144 grid from CAD BOM), `MotorType` enum |
| **Engine** | `simulator.engine` | Command parsing, A\* path planning, continuous collision detection with sliding, projectile physics, 30-second match clock |
| **Algorithm** | `simulator.algorithm` | A\* on 144×144 grid, diagonal-distance heuristic, Bresenham line-of-sight path smoothing |
| **Physics** | `simulator.physics` | 3D parabolic projectile motion, elastic goal-rim rebound, gravity = 386.09 in/s² |
| **I/O** | `simulator.io` | Regex-based FTC Java OpMode parser (extracts `moveTo`, `shoot`, `sleep`, `intake`) |
| **View** | `simulator.ui` | JavaFX `SubScene` with `PerspectiveCamera` orbit controls, `TriangleMesh` obelisk, real-time `AnimationTimer` loop |

---

## Core Mathematical Formulas & Algorithms

### 1. Mecanum Wheel Kinematics

**Inverse kinematics** (robot velocity → wheel speeds):

```
┌ FL ┐   ┌ 1  -1  -R ┐ ┌ vx  ┐
│ FR │   │ 1   1   R │ │ vy  │      R = halfTrack + halfWheelbase
│ BL │ = │ 1   1  -R │ │ vω  │
└ BR ┘   └ 1  -1   R ┘ └     ┘
```

**Forward kinematics** (wheel speeds → robot velocity) — Moore-Penrose
pseudoinverse of the 4×3 matrix:

```
vx      = (FL + FR + BL + BR) / 4
vy      = (−FL + FR + BL − BR) / 4
vω      = (−FL + FR − BL + BR) / (4·R)
```

### 2. Continuous Collision Detection (CCD) with Sub-step Sampling

To prevent *tunnelling* through thin obstacles (e.g. the 1.25" perimeter wall
panels) at high speed, each frame's displacement `step = v_max · dt` is
subdivided into `N = 3` micro-steps.  Collision is checked at every sub-step
boundary:

```
for i ∈ [0, N):
    pose ← pose + (dir · subStep)
    if checkCollision(field):   // 8-point vertex + edge-midpoint sampling
        undo subStep
        trigger slide response
        break
```

### 3. Oriented Bounding-Box Vertex Sampling

The robot's rotated footprint is sampled at **8 points** (4 corners + 4 edge
midpoints).  Each local coordinate `(f, ℓ)` (forward, left) is rotated into
world frame via the heading angle `θ`:

```
worldX = x + f·cos(θ) − ℓ·sin(θ)
worldY = y + f·sin(θ) + ℓ·cos(θ)
```

Collision is declared if any sample point falls inside an obstacle cell in the
144×144 grid.

### 4. Surface-Sliding Collision Response

When a collision is detected, the robot does **not** simply reverse direction.
Instead, 8 candidate escape directions `{0°, 45°, …, 315°}` are probed.  The
direction with the highest dot product against the intended velocity vector
`d̂` that is also collision-free is selected:

```
d_slide = argmax_{a ∈ angles}  [ cos(a)·d̂_x + sin(a)·d̂_y ]
          subject to:  pose + step·(cos(a), sin(a))  is collision-free
```

If no direction is free, the robot is declared **stuck** and the `MOVE_TO`
command aborts.

### 5. A\* Pathfinding with Diagonal Heuristic

**State space:**  144 × 144 grid cells (1 inch resolution).

**Neighbourhood:**  8-directional (cardinal cost = 1.0, diagonal cost = √2).
Diagonal moves include a *corner-cutting guard*: if either adjacent cardinal
cell is blocked, the diagonal move is disallowed.

**Heuristic (admissible & consistent):**

```
h = max(dr, dc) + (√2 − 1)·min(dr, dc)
```

This is the exact optimal-path cost on an unblocked 8-connected grid,
guaranteeing A\* optimality.

**Path smoothing:**  Raw grid-path waypoints are collapsed via
**Bresenham line-of-sight**.  For each anchor waypoint `P_i`, the furthest
visible successor `P_j` is found such that every cell along the Bresenham
rasterised line `P_i → P_j` is obstacle-free.  Intermediate waypoints are
discarded, yielding a compact, natural path.

### 6. 3D Ball Projectile Physics

The ball obeys Newtonian projectile motion under uniform gravity:

```
x(t) = x₀ + v₀·cos(φ)·cos(θ)·t
y(t) = y₀ + v₀·cos(φ)·sin(θ)·t
z(t) = z₀ + v₀·sin(φ)·t − ½·g·t²
```

where `g = 386.09 in/s²`, `φ` is the launch elevation angle (default 45°),
`θ` is the robot heading, and `v₀` is derived from the launcher motor RPM and
flywheel radius:

```
v₀ = (RPM / 60) · 2π · r_flywheel
```

**Goal-plane scoring:**  When `z(t) ≤ 38.75"` (rim height from CAD
`am-5735`), the horizontal distance to the goal centre is computed.  If
`d ≤ R_inner` (8"), the ball scores.  If `R_inner < d ≤ R_inner + 6"`, the
velocity vector undergoes **elastic reflection** off the rim normal `n̂`:

```
v' = v − (1 + e)·(v·n̂)·n̂,    e = 0.58  (coefficient of restitution)
```

The ball continues its flight with the new initial conditions, producing a
realistic rim-deflection arc.

### 7. Motor → Physics Mapping

Chassis maximum linear velocity is derived from motor RPM and drive-wheel
radius:

```
v_max = (RPM_chassis / 60) · 2π · r_wheel
```

Three motors are modelled with real FTC specifications:

| Motor | RPM | Rated Torque (N·m) |
|-------|-----|-------------------|
| goBILDA 5203 19.2:1 | 312 | 1.5 |
| goBILDA 5203 30:1 | 196 | 2.4 |
| REV HD Hex 20:1 | 300 | 1.2 |

---

## CAD Verification

The field geometry is validated against the official Onshape CAD assembly
**am-5700** (`DECODE™ presented by RTX Full Field`).  A Python scanner
(`cad_scanner.py`) extracts the Bill of Materials from the Parasolid `.x_t`
binary, identifying **51 unique parts** across **94 instances**:

| Part | Qty | Dimensions (in) | Description |
|------|-----|-----------------|-------------|
| am-2160b | 11 | 48 × 1.25 × 12.125 | Perimeter wall panels |
| am-2600b | 4 | 3 × 3 × 12.125 | Corner brackets |
| am-5715 | 2 | 11 × 11 × 4 | Obelisk base |
| am-5716 | 2 | 11 △ × 19 | Obelisk top (total 23") |
| am-5735 | 2 | 20 × 20 × 38.75 | Goal assemblies |
| am-5718 | 2 | 18 × 12 × 12 | Classifier boxes |
| am-5707 | 2 | 24 × 12 × 6 | Ramps |
| am-5704 | 3 | 24 × 12 × 18 | Gate assemblies |

> **Note:**  The Parasolid kernel format prevents direct vertex extraction
> without a licensed solver.  The bounding-box positions in `FieldMap.java`
> are derived from the FTC Game Manual §9 ARENA drawings cross-referenced
> with the BOM instance counts.  Teams with Onshape access can export precise
> centroid coordinates to further refine `FieldMap` placement constants.

---

## How to Run

### Prerequisites

- **JDK 17+** (the bundled Gradle wrapper downloads Gradle 8.5 automatically)
- A graphical display (JavaFX 3D requires a windowing system)

### Build & Launch

```bash
# Compile, test, and package
./gradlew build

# Launch the 3D simulator
./gradlew run
```

On Windows, replace `./gradlew` with `gradlew.bat`.

### Using the Simulator

1. **Configure the robot** in the left panel:
   - Set chassis **Width** and **Length** (inches, max 18" per FTC rules).
   - Select **Chassis Motor** and **Launcher Motor** from the dropdowns.
   - Toggle **Vision Sensor**, **Intake Type**, and **Scoring Capability**.

2. **Write an autonomous program** in the text area (one command per line):
   ```
   MOVE_TO(24, 24)
   INTAKE
   MOVE_TO(-48, -12)
   LAUNCH
   ```
   Or click **Upload .java OpMode** to parse a real FTC autonomous file.

3. **Click Start** — the 30-second match clock begins.  Observe:
   - The robot (green box) navigating via the A\*-planned path (green dots).
   - Collision-sliding behaviour when the robot grazes obstacles.
   - Ball trajectory (golden sphere) during `LAUNCH` commands.
   - Real-time log messages in the status bar.

4. **Camera controls:**
   - **Drag** to orbit around the field centre.
   - **Scroll** to zoom in/out.

### Running Tests

```bash
./gradlew test
```

Verifies Mecanum kinematics (pure-forward, pure-rotation, round-trip
consistency) and forward/inverse kinematic duality.

---

## Project Structure

```
src/main/java/simulator/
├── Main.java                         # JavaFX Application entry point
├── algorithm/
│   └── PathFinder.java               # A* with diagonal heuristic + LOS smoothing
├── engine/
│   └── AutoSimulationEngine.java      # Command parser + 30s match driver
├── io/
│   └── JavaCodeParser.java           # Regex FTC OpMode → command translator
├── model/
│   ├── Pose2d.java                   # (x, y, heading)
│   ├── PoseVelocity2d.java           # (vx, vy, vω)
│   ├── Robot.java                    # Mecanum kinematics + collision detection
│   ├── RobotProfile.java             # Configurable hardware profile
│   ├── MotorType.java               # FTC motor enum (RPM, torque)
│   └── FieldMap.java                # 144×144 grid, CAD-verified elements
├── physics/
│   └── BallProjectileEngine.java     # 3D projectile + elastic rim rebound
└── ui/
    └── SimulatorApp.java             # JavaFX 3D viewport + control panel

src/test/java/simulator/model/
└── KinematicsTest.java               # JUnit 5 kinematics unit tests (4 tests)
```

---

## License

This project is an independent educational tool and is not affiliated with or
endorsed by *FIRST*<sup>&reg;</sup>, *AndyMark*, or *Onshape*.  All trademarks
belong to their respective owners.

DECODE<sup>SM</sup> is a service mark of *FIRST*<sup>&reg;</sup>.

---

*Built with Java 17, JavaFX 3D, Gradle 8.5, and the FTC DECODE official CAD
assembly (am-5700).*
