# FlyFree architecture

This document sets the **scale and direction** of FlyFree. It is meant to change: we will revise it as we iterate together.

## 1. Product in one sentence

A visionOS app where you sit in the cockpit of a craft you chose, and fly it with your bare hands held out **flat**. A plane is fitted to each hand's fingers. The rotation of each plane relative to a calibrated neutral pose is modelled in 3D and applied in the craft's current frame of reference to produce a *desired craft state*. A craft-specific translator then turns that state into engine, thruster and control-surface commands.

Your dominant hand is the stick:
- tilting the fingertips up or down pitches the craft,
- turning the wrist horizontally yaws it,
- rotating the forearm rolls it.

Your other hand is a throttle pedal, like a car's: push the fingertips down to accelerate, hold it level to keep your speed, and tip it up to brake.

Closing a hand is a reset gesture. A closed stick hand returns the craft to its starting orientation, and a closed throttle hand brakes to a full stop.

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

  **Starting pose (`StartingPose`).** Hands are always palm down, never turned sideways. The spacecraft and drone start with **both hands flat** (`bothFlat`); the jet starts with the **stick hand bent up ~45° like a push and the throttle hand bent down ~45°** (`stickUpThrottleDown`). Calibration only completes when each hand's plane is within about 35° of its pose (`HandReadiness.wrongPose` otherwise), and a faint outline of the target floats on each hand, turning green when it matches. The next simulation in the auto-advance cycle recalibrates before its scene fades in.

### 3.1 Hand source → `HandFrame`
- **ARKit (`HandTrackingService`)** reads `HandTrackingProvider.latestAnchors` once per frame and converts each joint into world-space positions.
- **Simulated (`SyntheticHand`)** builds skeletons from pitch, yaw, roll and curl. Unit tests and the in-scene simulator panel use it.
- `HandFrame` holds only positions, so any future hand source (gloves, recorded replays) only has to produce `HandFrame`s.

### 3.2 Hand planes → `PilotInput` (normalized and craft-agnostic)
- **`HandPlane`:** a least-squares plane through every joint of the index, middle, ring and little fingers (20 points). The normal is the smallest eigenvector of the covariance matrix. The forward direction is the knuckle → fingertip direction projected into the plane, and the lateral direction is their cross product. The result is a full orientation (X lateral, Y back-of-hand, −Z fingertips) plus an RMS residual, which measures how flat the hand is.
- **Stick (dominant hand):** `r = log(current · neutral⁻¹)` is the hand's rotation since calibration, expressed in the **pilot's frame**. Measuring in the pilot's frame keeps pitch, yaw and roll aligned with the pilot's own axes even if the calibrated flat hand is turned in slightly. Its components map directly to body axes:
  - `r.x`: the fingertips' plane tilting front-to-back → **pitch**
  - `r.y`: horizontal wrist rotation → **yaw**
  - `r.z`: forearm rotation → **roll**

  Each component is divided by its full deflection, ±90° (π/2) on every axis. The profile then applies an **exponential response**, `(e^(k·n) − 1)/(e^k − 1)` with k = 2.5:
  - 10° of hand rotation gives under 4% of full rate;
  - 45° gives about 22%;
  - full rate needs the full 90°.

  This gives very fine control near center. Because the rotation is modelled as one 3D rotation rather than three independently measured angles, combined motions stay consistent.
- **Stick hand states:**
  - flat (curl ≤ 0.3): flying, with authority ramping in over about 60 ms so there is no jolt;
  - closed (curl ≥ 0.5): *reset orientation*;
  - not tracked: attitude hold.
- **Throttle (pedal):** the throttle hand's plane pitch relative to its neutral pose, sign-flipped. Fingertips **down** gives +1 (accelerate), level gives 0, up gives −1 (brake, then reverse). Closing the hand means *brake to a stop*.
- **Filtering:** a 1€ filter on every axis, tuned for responsiveness: about 33 ms resting time constant for the stick and 40 ms for the throttle. The values are in `InterpreterConfiguration`.
- **Tracking loss:** losing the stick hand releases it (attitude hold). Losing the throttle hand holds the throttle for 0.5 s, then sets it to 0.

### 3.3 Intent → `DesiredCraftState`, in the craft's current frame
- `IntentMapper` applies the craft's `ControlProfile` (deadzone, expo, maximum rates) and works in the craft's **current body frame**.
- **Smooth references with low momentum.** The mapper produces *reference* signals, and the controller tracks them tightly using acceleration feed-forward. Rotation and translation have **separate** limits, and braking is about 3× stronger than speeding up, so the craft eases into motion but settles quickly.

  | | Speed up | Slow down |
  |---|---|---|
  | Rotation (pitch, yaw, roll) | 1.2, 1.2, 1.8 rad/s² (full rate in about 0.5 s) | 2.4, 2.4, 3.6 rad/s² (stops in about 0.25 s) |
  | Translation | 4 m/s² | 12 m/s² |
- **Quaternion navigation, with no Euler angles anywhere in the control path.**
  - Attitude is a unit quaternion. It is integrated with exact axis-angle steps and renormalized every step.
  - Every frame change is a quaternion acting on a vector.
  - Errors are quaternion logs (rotation vectors): hand-relative rotation, attitude-hold error, and reset error.
  - The pedal's tilt comes from a swing-twist decomposition of the relative hand quaternion.

  There is therefore no gimbal lock at any attitude, and tests tumble the ship through more than 360° about oblique axes to confirm it.
- **The stick commands deltas relative to the ship's current frame.** Hand deflection becomes a rotation *rate about the ship's own axes*, so a yaw gesture yaws the ship the same way whatever its attitude. A world-fixed ("global") stick was tried and rejected: it made the same gesture do different things depending on the ship's attitude.
- **Rotation:** the target rate slews as a *vector*. The step size lies on the ellipsoid of per-axis limits, so the rotation axis never bends while rates build up.
  - Centered or released: once rotation has settled, the current attitude becomes a *target to hold*. Capturing it only after settling avoids snap-back when the stick is released mid-turn.
  - Closed stick hand: the craft returns to its reset attitude (the launch attitude), using the same limits.
- **Translation in velocity-hold mode (default):** the pedal *integrates* the forward-speed reference: down accelerates, level holds speed, up brakes and then reverses (reverse is capped at 25% of top speed). Lateral drift is nulled.
- **Translation in Newtonian mode (`off`):** the pedal is a thrust fraction.
- **Closed throttle hand (either mode):** the velocity reference ramps to zero at the braking limit.

### 3.4 Control → body `Wrench` demand
- **Rotation:** `ω* = ω_ref + clamp(K_att · log(q⁻¹ · q_target))`, then `τ = I·(α_ff + K_rate·(ω* − ω)) + ω × Iω`. Here `α_ff` is the feed-forward acceleration and the last term is gyroscopic compensation.
- **Translation:** `F = m·(a_ff + K_v·(v_ref − v))`.
- **Thruster sizing:** the RCS (50 kN per thruster) and retro thrusters (75 kN each) are sized so the braking limits are physically reachable.
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
- **Speedometer:** a round dial on the console, scaled per craft by `GaugeScale` (in the `Instruments/` folder of the core).
  - Jet: linear 0–350 m/s airspeed, with a tick every 10 m/s and a labelled tick every 50 m/s.
  - Spacecraft: logarithmic, 0 to 10 m/s linear and then one segment per decade up to 100 km/s.
  - Both have a red line at the top end, a needle and a digital readout.
- **Hand plates:** holographic rings on each fitted hand plane (rim, halo, compass ticks, forward chevron). Colour shows state: amber means waiting, cyan means flying, violet means resetting, red means braking.
- **Hand holograms**, which follow each hand:
  - over the stick hand, a miniature ship at its true attitude relative to launch, with a level reference ring and a rotation-rate vector;
  - over the throttle hand, the thruster layout with plumes sized by each actuator's command, and a pedal gauge.
- **Course (`Mission/`):** `CourseTracker` detects gate passes by checking whether the craft's path crosses the gate plane in the right direction inside the ring's radius. It scores passes (streaks for in-order gates, plus a finish bonus). Gates show as pending (cyan), next (amber, pulsing), just-passed (white flash) and passed (green).
- **Score hologram:** extruded holographic text in the upper right of the windscreen, showing score, gates and streak, with "+points" pop-ups.
- **Environment:** a starfield in three tints, a sun with corona and light, a ringed gas giant, nebula clouds, and wrapping asteroids and space dust for speed cues.
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

## 5a. Aircraft (the jet)

The jet runs through the same pipeline, with more physics and a fly-by-wire layer added to the intent stage:

- **Environment (`Atmosphere`):** gravity, exponential air density, wind, and flat ground at y = 0.
- **Airframe (`AerodynamicModel`):**
  - Air data: airspeed, angle of attack α, sideslip β and dynamic pressure q̄.
  - Lift, with a linear range and a post-stall drop; drag, made up of parasite, induced, stall and speed-brake drag; and side force.
  - Stability and damping moments: pitch stiffness `Cmα`, `Cmq`; dihedral and roll damping; weathervane and yaw damping.
- **Control surfaces:** the elevator, ailerons and rudder are `aeroSurface` actuators. Each is a pure moment per pascal, with commands from −1 to 1. The allocator receives per-actuator *effectiveness* (q̄ for surfaces) every physics step, so the same bounded least-squares solver flies vacuum thrusters and airspeed-dependent surfaces.
- **Dynamic inversion:** the controller subtracts the airframe's own moments, so the surfaces only supply what is missing.
- **Landing gear:** three spring-damper wheels with rolling, braking and cornering friction.
- **Flight computer (`IntentMapper.aircraftReference`):**
  - stick deltas plus coordinated-turn feed-forward (`g·tanφ/V` about world up);
  - releasing the stick holds flight-path angle and bank, while heading follows the turn;
  - α protection with pitch-rate look-ahead;
  - closed stick: wings level, level flight;
  - pedal: an autothrottle airspeed target; closed throttle hand: idle, speed brake and wheel brakes.
- **Scenery:**
  - sky dome and horizon haze, sun;
  - an endless plain with a runway and edge lights;
  - repeating trees, rocks and houses;
  - snow-capped mountains and clouds;
  - the Valley Run course (five gates facing along the route).

## 5b. Multirotor (the passenger drone)

- **Rotors** are `rotor` actuators. Each produces lift along body up plus a reaction torque about up; its spin direction sets the torque's sign. The allocator mixes them exactly like a quadcopter mixer, with no hand-written mixing table. For small actuator groups it starts from the exact least-squares solution, because rotor yaw torque is tiny next to lift.
- **Physics:** `Atmosphere` (gravity and air), `BodyDrag` (bluff-body drag and rotational damping), and four landing skids.
- **Throttle hand is the collective (`ThrottleHandMode.position`).** Calibration records each hand's neutral *position* as well as its orientation. The throttle hand's height above or below that spot, scaled so ±15 cm is full command, becomes `PilotInput.translation.y`.
- **Flight computer (`IntentMapper.multirotorReference`): RC angle mode with a raw collective,** modelled on Betaflight / PX4 angle (stabilized) mode with a Mode 2 split — thrust on one hand, attitude on the other:
  - **Collective:** hand height → total rotor thrust. Centre is hover thrust (as with PX4's default rescaled throttle curve, `MPC_THR_CURVE`), full up is 85 % of the rotors' total thrust (the rest is kept as attitude headroom, like a mixer's desaturation), full down is zero. There is no altitude hold: vertical speed persists at centre, and tilting spends lift.
  - **Attitude:** stick-hand tilt (pitch and roll, in the pilot's frame) → target tilt of the rotor disc, 0.6× the hand's tilt, capped at `maxTilt` (Betaflight's `angle_limit`). A flat hand levels the drone, which then coasts.
  - **Yaw:** stick-hand twist → yaw rate; heading is held when centred.
  - Closed throttle hand: hover assist (collective that drives vertical speed to zero, compensating for tilt). Closed stick hand: level and turn back to the launch heading. On the pad, tilt and yaw are ignored and a centred or lowered hand idles the rotors.

  The chain from there is the standard cascade: the controller turns attitude error into a rate command (angle P) and rate error into torque (rate loop), and the allocator mixes collective + torque into **individual rotor thrusts**.
- **Starting pose:** both hands flat.
- **Scene:**
  - a procedural city on a 100 m street grid, with towers that get taller downtown and use shared generated window textures;
  - a park and a helipad at the origin;
  - rotor arms, ducts and spinning blades visible from the bubble cockpit;
  - the Rooftop Run course: six gates hung over the streets.

## 6. Flight-sim direction

See [`FLIGHT_MODEL.md`](FLIGHT_MODEL.md). It plans orientation changes **without** acceleration (spacecraft: decoupled, and thrust-coupled with a coupling slider) and **with** acceleration (aircraft: atmosphere, aerodynamic model, aero-surface allocation, fly-by-wire control modes), with milestones F1–F7.

## 7. Open questions for alignment
1. **Throttle hand's other axes.** Should the throttle hand's yaw and roll drive RCS strafing?
2. **Pitch source.** Should it be the whole fitted plane (wrist plus finger bend), or should wrist flexion and knuckle bend be separate signals, for example a finer trim from the fingers?
3. **Neutral re-capture.** Recenter is manual today. Should the app also recenter automatically when both hands are held flat and still with the stick released?
4. **Seated or standing pilot,** and should the cockpit scale to the pilot's measured hand height from calibration?
5. **Craft priority** after the spacecraft: atmospheric jet, lander, or drone?
6. **Fidelity target:** arcade, "Kerbal-ish", or study-level?
