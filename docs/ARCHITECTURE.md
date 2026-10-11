# FlyFree architecture

This document sets the **scale and direction** of FlyFree. It is meant to change: we will revise it as we iterate together.

## 1. Product in one sentence

A visionOS app where you sit in the cockpit of a craft you chose, and fly it with your bare hands. Your dominant hand is a virtual stick for attitude. Your other hand's finger curl is the throttle. Finger and hand angles become a *desired craft state*, and a craft-specific translator turns that into engine, thruster and control-surface commands.

## 2. Guiding principles

1. **Separate intent from mechanism.** The pilot says *what* they want (for example "pitch up at 20°/s, go 80 m/s"). Each craft decides *how* to do it. The same hands fly a spacecraft, a jet or a drone.
2. **A pure, testable core.** Everything from hand joints to physics lives in `FlyFreeCore`. It has no ARKit, RealityKit or UI dependencies, so it runs headless on a Mac in CI.
3. **Craft are data.** A `CraftDefinition` holds mass properties, actuator layout, handling profile, controller gains and cockpit style. New craft should rarely need new code.
4. **Safe by default.** A dead-man grip, attitude hold when you let go, a throttle grace period on tracking loss, and comfort-first visuals.
5. **Runs without a headset.** *Simulated Hands* feed synthetic skeletons through the real pipeline in the simulator.

## 3. The pipeline

```
┌──────────────┐  HandFrame×2   ┌──────────────────────┐  PilotInput    ┌──────────────┐
│ Hand source  │ ─────────────► │ PilotInputInterpreter│ ─────────────► │ IntentMapper │
│ ARKit / Sim  │  joint xyz     │ grip, neutral, 1€    │  normalized    │ ControlProfile│
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

### 3.1 Hand source → `HandFrame`
- **ARKit (`HandTrackingService`)** reads `HandTrackingProvider.latestAnchors` once per frame and converts each joint into world-space positions.
- **Simulated (`SyntheticHand`)** builds skeletons from pitch, yaw, roll and curl. Unit tests use it too.
- `HandFrame` holds only positions (wrist plus the finger chains). Any future hand source, such as gloves or a recorded replay, only has to produce `HandFrame`s.

### 3.2 Interpretation → `PilotInput` (normalized and craft-agnostic)
- **Finger curl** is the sum of the joint bend angles along each finger. It is normalized to 0…1 using the per-hand `CurlCalibration`.
- **Engage (dead-man):** the stick engages when the dominant hand's middle, ring and little finger curl rises above 0.55, and releases below 0.35. The gap between the two thresholds (hysteresis) stops it flickering.
- **Neutral:** `captureOnEngage` by default. The pose at the moment you grip becomes center, so there is no calibration and no drift. A `fixed` neutral is available for a calibrated, real-stick feel.
- **Attitude:** the hand basis (fingers forward, knuckle line sideways) relative to neutral, converted to pitch, yaw and roll. Full deflection is 0.6, 0.6 and 0.7 rad. Two stick styles:
  - `palm`: the whole hand moves as a fist.
  - `pointer`: the index finger aims and the palm rolls.
- **Throttle:** the mean curl of the non-dominant hand's four fingers.
- **Filtering:** a 1€ filter on every axis, for low jitter at rest and low lag in motion.
- **Tracking loss:** losing the stick hand disengages, which means attitude hold. Losing the throttle hand holds the throttle for 0.5 s, then sets it to 0.

### 3.3 Intent → `DesiredCraftState`
- `IntentMapper` applies the craft's `ControlProfile`: a deadzone, expo, and the maximum rate on each axis.
- **Flight assist:**
  - `velocityHold` (default): throttle sets the desired forward speed, and lateral drift is nulled.
  - `off`: throttle is a fraction of the main engines' thrust (pure Newtonian flight).
- Later: attitude-hold variants, horizon-level for aircraft, and hover or altitude-hold for landers and drones.

### 3.4 Control → body `Wrench` demand
- **Rotation:** rate command with a proportional gain, plus gyroscopic compensation (`τ = I·K·(ω* − ω) + ω × Iω`). A zero command holds attitude.
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
- **HUD:** a SwiftUI attachment showing stick state, commands, rates, throttle, speed and hand-tracking status.
- **Hologram:** a mini craft on the console, with one dot per actuator that grows with its command. It is a live view of the allocator.

## 4. Module map

| Area | Location | Key types |
|---|---|---|
| Math | `FlyFreeCore/Math` | `Vec3`, `Quat`, `Wrench`, `BodyAxis` |
| Input | `FlyFreeCore/Input` | `HandFrame`, `HandMetrics`, `PilotInputInterpreter`, `OneEuroFilter`, `SyntheticHand` |
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
1. **Stick metaphor.** Should the stick be the whole fist (current default), the pointing index finger, or finger *angles only*, with the wrist ignored to reduce fatigue?
2. **Throttle metaphor.** Curl is absolute (open = 0). Alternatives:
   - a slide along the throttle hand's position,
   - incremental ("ratchet") throttle,
   - a thumb gesture for reverse or retro thrust.
3. **Translation (RCS strafe).** Should it come from the throttle hand's position offset, or from a modifier pinch?
4. **Neutral policy.** Should the neutral pose be re-captured on every grip (current), or set once with a calibration step?
5. **Seated or standing pilot,** and should the cockpit scale to the user's real seated height?
6. **Craft priority** after the spacecraft: atmospheric jet, lander, or drone?
7. **Fidelity target:** arcade, "Kerbal-ish", or study-level?
8. **Multiplayer and spectators:** SharePlay formation flying?
