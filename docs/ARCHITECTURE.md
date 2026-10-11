# FlyFree architecture

This document sets the **scale and direction** of FlyFree. It is meant to change: we will revise it as we iterate together.

## 1. Product in one sentence

A visionOS app where you sit in the cockpit of a craft you chose, and fly it with your bare hands held out **flat**. A plane is fitted to each hand's fingers. The rotation of each plane relative to a calibrated neutral pose is modelled in 3D and applied in the craft's current frame of reference to produce a *desired craft state*. A craft-specific translator then turns that state into engine, thruster and control-surface commands.

Your dominant hand is the stick:
- tilting the fingertips up or down pitches the craft,
- turning the wrist horizontally yaws it,
- rotating the forearm rolls it.

Your other hand is a throttle lever: tip it up for forward, down for reverse.

## 2. Guiding principles

1. **Separate intent from mechanism.** The pilot says *what* they want (for example "pitch up at 20°/s, go 80 m/s"). Each craft decides *how* to do it. The same hands fly a spacecraft, a jet or a drone.
2. **A pure, testable core.** Everything from hand joints to physics lives in `FlyFreeCore`. It has no ARKit, RealityKit or UI dependencies, so it runs headless on a Mac in CI.
3. **Craft are data.** A `CraftDefinition` holds mass properties, actuator layout, handling profile, controller gains and cockpit style. New craft should rarely need new code.
4. **Flat hands, measured as planes.** A least-squares plane through about 20 finger joints is much more robust than any single finger. A unit test shows it is clearly less sensitive to joint jitter than reading the index finger alone.
5. **Safe by default.** A flat hand engages the stick and a closed hand releases it, with attitude hold on release. Closing the throttle hand locks cruise. Throttle has a grace period on tracking loss. Visuals are comfort-first.
6. **Runs without a headset.** *Simulated Hands* feed synthetic skeletons through the real pipeline in the simulator.

## 3. The pipeline

```
┌──────────────┐  HandFrame×2   ┌──────────────────────┐  PilotInput    ┌──────────────┐
│ Hand source  │ ─────────────► │ PilotInputInterpreter│ ─────────────► │ IntentMapper │
│ ARKit / Sim  │  joint xyz     │ plane fit, neutral,1€│  normalized    │ ControlProfile│
└──────────────┘                └──────────────────────┘  -1…1, 0…1     └──────┬───────┘
                                                                              │ DesiredCraftState
                                                                              ▼ (rates, speed/thrust)
┌──────────────┐  RigidBodyState ┌──────────────────┐  Wrench demand  ┌──────────────────┐
│ Renderer     │ ◄────────────── │ Rigid-body       │ ◄────────────── │ FlightController │
│ cockpit+world│   + telemetry   │ integrator + Env │   achieved      │ rate / velocity  │
└──────────────┘                 └────────▲─────────┘                 └────────┬─────────┘
                                          │ achieved wrench                    │ demand
                                          └─────────── ControlAllocator ◄──────┘
                                                       actuator commands 0…1
```

`FlightPipeline` ties the stages together. Input and intent run once per rendered frame (about 90 Hz). Control, allocation and physics run on a fixed 180 Hz sub-step, so flight does not depend on frame rate.

### 3.0 Session flow
1. **Hangar window:** choose the craft and options, then press **Launch**. The window closes itself.
2. **Calibration**, in a mixed immersive space over passthrough, modelled on the Vision Pro hand setup. The prompt says *"Hold out both hands, flat."* A translucent plate floats on each hand's fitted plane. It is orange while the hand isn't ready and turns green when locked. `CalibrationSession` needs each hand to be:
   - detected,
   - flat: curl ≤ 0.3 and plane RMS ≤ 12 mm,
   - palm-down: normal within about 53° of vertical,
   - steady: under 0.5 rad/s.

   Both hands must meet these for 1.5 s. The orientation averaged over that hold becomes each hand's **neutral frame**.
3. **Fade-in:** over 2.5 s the cockpit, the universe and an opaque backdrop fade in, and the room disappears.
4. **Flight:** **Recenter**, **Reset** and **Land** are buttons on the dashboard. Recenter repeats step 2 with the cockpit still visible. Land, or leaving with the Digital Crown, ends the space and reopens the hangar.

### 3.1 Hand source → `HandFrame`
- **ARKit (`HandTrackingService`)** reads `HandTrackingProvider.latestAnchors` once per frame and converts each joint into world-space positions.
- **Simulated (`SyntheticHand`)** builds skeletons from pitch, yaw, roll and curl. Unit tests and the in-scene simulator panel use it.
- `HandFrame` holds only positions, so any future hand source (gloves, recorded replays) only has to produce `HandFrame`s.

### 3.2 Hand planes → `PilotInput` (normalized and craft-agnostic)
- **`HandPlane`:** a least-squares plane through every joint of the index, middle, ring and little fingers (20 points). The normal is the smallest eigenvector of the covariance matrix. The forward direction is the knuckle → fingertip direction projected into the plane, and the lateral direction is their cross product. The result is a full orientation (X lateral, Y back-of-hand, −Z fingertips) plus an RMS residual, which measures how flat the hand is.
- **Stick (dominant hand):** `r = log(neutral⁻¹ · current)` is a rotation vector expressed in the neutral hand frame, so its components map directly to body axes:
  - `r.x`: the fingertips' plane tilting front-to-back → **pitch**
  - `r.y`: horizontal wrist rotation → **yaw**
  - `r.z`: forearm rotation → **roll**

  Each component is divided by its full deflection (0.45, 0.5 and 0.7 rad). Because the rotation is modelled as one 3D rotation rather than three independently measured angles, combined motions stay consistent.
- **Dead-man:** the stick engages while the hand is flat (curl ≤ 0.3) and releases when it closes (curl ≥ 0.5). Authority ramps in over about 60 ms after engaging so there is no jolt.
- **Throttle (lever, the default):** the throttle hand's plane pitch relative to its neutral pose. Fingertips up gives +1 (forward), level gives 0, down gives −1 (reverse). Closing the hand **locks** the current setting (cruise), and flattening it takes control back. *Finger curl* (open = 0, fist = 1) remains available as an alternative.
- **Filtering:** a 1€ filter on every axis, tuned for responsiveness: about 33 ms resting time constant for the stick and 40 ms for the throttle. The values are in `InterpreterConfiguration`.
- **Tracking loss:** losing the stick hand releases it (attitude hold). Losing the throttle hand holds the throttle for 0.5 s, then sets it to 0.

### 3.3 Intent → `DesiredCraftState`, in the craft's current frame
- `IntentMapper` applies the craft's `ControlProfile` (deadzone, expo, maximum rates) and works in the craft's **current body frame**.
- **Rate-command / attitude-hold:**
  - With the stick deflected, the desired state is a body angular rate.
  - With the stick centered or released, the craft's attitude becomes a *target to hold* once rotation has settled. Capturing it only after settling avoids snap-back when the stick is released mid-turn.
- **Flight assist:**
  - `velocityHold` (default): throttle sets the desired forward speed; negative throttle reverses at 25% of maximum speed. Lateral drift is nulled.
  - `off`: throttle is a fraction of the main engines' thrust; negative throttle fires the retro thrusters.

### 3.4 Control → body `Wrench` demand
- **Rotation:** `ω* = ω_cmd + clamp(K_att · log(q⁻¹ · q_target))`, then `τ = I·K_rate·(ω* − ω) + ω × Iω` (gyroscopic compensation).
- **Translation:** `F = m·K·(v* − v)` in the body frame, or a direct thrust fraction when assist is off.
- Later: PID with integral and anti-windup, and gain scheduling for aircraft (dynamic pressure).

### 3.5 Allocation ("the translator") → actuator commands
- Each actuator has a body position, a force direction, a maximum force, and an allocation priority.
- Groups are solved in priority order. Each group solves a **bounded weighted least-squares** problem, `min ‖W(Bu − w)‖²` with `0 ≤ u ≤ 1`, against whatever demand the earlier groups left over. The solver is projected gradient descent with the step size set from the largest eigenvalue, warm-started each frame.
- With main engines in group 0 and RCS in group 1, large and small thrusters do not fight each other, and any layout works without hand-written mixing tables.
- The *achieved* wrench (not the demanded one) drives physics, so saturation shows up honestly.
- Later actuator kinds: aerodynamic surfaces (aileron, elevator, rudder, with forces that depend on airspeed), gimbaled engines, reaction wheels, rotors, and on/off RCS with PWM.

### 3.6 Dynamics
- 6-DoF rigid body with diagonal inertia: Euler's rotation equations and semi-implicit Euler integration.
- A `FlightEnvironment` protocol supplies external loads: `DeepSpace` (now) and `UniformGravity` (ready). Later: atmosphere and drag, wind, and orbital gravity.

### 3.7 Presentation (visionOS)
- **Hangar window:** craft selection, handedness, stick style, assist mode, input source, simulated-hand sliders, telemetry.
- **Full `ImmersiveSpace`:** the pilot stays at the origin inside a procedural cockpit. visionOS does not let the app move the camera, so the universe moves by the inverse of the craft's pose:
  - a *sky* layer that only rotates (stars and the sun at infinity),
  - a *world* layer that rotates and translates (asteroids and gates, for a sense of speed).
- **Dashboard:** Recenter, Reset and Land, mounted on the main console.
- **HUD:** a SwiftUI attachment showing stick state, commands, rates, throttle, speed and hand-tracking status.
- **Hologram:** a mini craft on the console, with one dot per actuator that grows with its command. It is a live view of the allocator.

## 4. Module map

| Area | Location | Key types |
|---|---|---|
| Math | `FlyFreeCore/Math` | `Vec3`, `Quat`, `Wrench`, `BodyAxis` |
| Input | `FlyFreeCore/Input` | `HandFrame`, `HandPlane`, `HandMetrics`, `CalibrationSession`, `HandCalibration`, `PilotInputInterpreter`, `OneEuroFilter`, `SyntheticHand` |
| Intent | `FlyFreeCore/Intent` | `ControlProfile`, `AxisShaping`, `DesiredCraftState`, `IntentMapper` |
| Control | `FlyFreeCore/Control` | `FlightController`, `ControllerGains` |
| Allocation | `FlyFreeCore/Allocation` | `Actuator`, `ControlAllocator` |
| Dynamics | `FlyFreeCore/Dynamics` | `RigidBodyState`, `MassProperties`, `RigidBodyIntegrator`, `FlightEnvironment` |
| Craft | `FlyFreeCore/Craft` | `CraftDefinition`, `CraftCatalog`, `CockpitStyle` |
| Pipeline | `FlyFreeCore/Pipeline` | `FlightPipeline`, `FlightTelemetry` |
| App | `App/FlyFree` | `AppModel`, `HandTrackingService`, `CockpitScene`, `HangarView` |

## 5. Conventions
- **Body frame:** +X right, +Y up, −Z forward (the same as RealityKit). Positive rotation follows the right-hand rule: +X is pitch up, +Y is yaw left, +Z is roll left. Moving your hand in a direction moves the craft the same way.
- **Units:** SI (m, kg, s, N, N·m, rad). The UI shows degrees.
- **Precision:** the core uses `Double`. Values are converted to `Float` only at the RealityKit boundary.

## 6. Open questions for alignment
1. **Throttle lever.** Should a level hand mean zero speed (current), or should the lever be offset so a resting hand cruises? Should the throttle hand's yaw and roll drive RCS strafing?
2. **Pitch source.** Should it be the whole fitted plane (wrist plus finger bend), or should wrist flexion and knuckle bend be separate signals, for example a finer trim from the fingers?
3. **Neutral re-capture.** Recenter is manual today. Should the app also recenter automatically when both hands are held flat and still with the stick released?
4. **Seated or standing pilot,** and should the cockpit scale to the pilot's measured hand height from calibration?
5. **Craft priority** after the spacecraft: atmospheric jet, lander, or drone?
6. **Fidelity target:** arcade, "Kerbal-ish", or study-level?
