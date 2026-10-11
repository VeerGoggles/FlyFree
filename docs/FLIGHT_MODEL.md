# Flight model plan: orientation changes with and without acceleration

FlyFree has to support two families of craft:

- **Spacecraft**, where turning the nose changes nothing about where you are going.
- **Aircraft**, where turning the nose *is* how you change where you are going: wings turn attitude into acceleration.

This document plans how one hands → intent → control → allocation → physics pipeline grows to cover both, so the flight-sim side can be built step by step.

## 1. The core distinction

Two quantities are always separate in the physics, even when a flight mode couples them on purpose:

- **Attitude:** where the nose points, `q`, and how fast it turns, `ω`.
- **Velocity:** where the craft is actually going, `v`.

The *flight-path angle* is the angle between the nose and the velocity vector. The way a flight model handles that angle defines its character:

| Regime | Turning the nose… | What changes velocity | Feels like |
|---|---|---|---|
| **A. Decoupled (Newtonian)** | rotates attitude only, with no acceleration | thrust only | real spaceflight; drift |
| **B. Thrust-coupled (assisted)** | makes the controller thrust so velocity follows the nose | thrusters, closed loop | arcade space; "drive it like a plane" |
| **C. Aerodynamic** | changes angle of attack, which creates lift, which accelerates the craft | lift, drag, thrust | real aircraft |

**Status today:** A is `FlightAssistMode.off`. B is `.velocityHold`: the velocity reference is held in the *body* frame, so turning re-aims it and the thrusters accelerate the craft along the new direction. C is not built yet.

## 2. Orientation change *without* acceleration (regimes A and B)

**What to keep.** The rotation pipeline is the same for every craft: the stick sets a target rate, the rate reference slews under the angular acceleration and deceleration limits, the controller tracks it with feed-forward, and a released stick holds attitude.

**What to add for spacecraft:**
1. **Coupling slider.** A `velocityCoupling` value from 0 to 1 in `ControlProfile`, which sets how much the velocity reference follows the nose:
   - 0 gives pure regime A (with velocity hold still nulling drift that you command).
   - 1 gives the current regime B.
   - Values in between give a "drifty arcade" feel.

   Implementation: the velocity reference lives in the world frame, and each frame it is rotated toward the body frame by `coupling × Δattitude`.
2. **Flight-path marker:** a HUD symbol (and a marker on the stick-hand hologram) showing where the craft is going, as opposed to where it points. This is essential once the two differ.
3. **Drift-kill.** Today a closed throttle hand stops everything. Add a variant that nulls only the sideways velocity, keeping forward speed.
4. **Lateral acceleration limit.** The rate at which a turn can re-aim velocity is limited by available side thrust. Expose it as `maxTurnAcceleration` (m/s²) so that turning at speed carries momentum and turns get wider as speed grows.

## 3. Orientation change *with* acceleration (regime C: the flight sim)

### 3.1 Physics additions (`Dynamics/`)
- **`Atmosphere: FlightEnvironment`:** density from altitude (ISA table), wind, and turbulence (Dryden model, seeded).
- **`AerodynamicModel`:** given air-relative velocity in the body frame, compute angle of attack α, sideslip β, and dynamic pressure `q̄ = ½ρV²`. From those, compute lift, drag, side force and moments from coefficient tables `C_L(α)`, `C_D(α)`, `C_Y(β)`, `C_m(α, δe)`, `C_l(β, δa, p)` and `C_n(β, δr, r)`. Damping derivatives (`C_mq`, `C_lp`, `C_nr`) give natural rotational damping.
- **Stall:** `C_L` drops off past α_crit, with hysteresis.
- **Ground:** a ground plane with landing-gear spring-damper contact.

### 3.2 Actuator additions (`Allocation/`)
- **`aeroSurface` actuators** (aileron, elevator, rudder, flaps). Effectiveness scales with `q̄`, so the allocator's B matrix becomes **state-dependent** and is rebuilt each physics step from `q̄`.
- **`gimbaledEngine`** for thrust vectoring, a common bridge between regimes B and C.
- **Mixed allocation:** priorities become aero surfaces → engines → RCS (useful for space-plane re-entry).

### 3.3 Control-law additions (`Control/`)
Aircraft are usually flown with *outcome* commands, not raw rates. These are the fly-by-wire modes to add:

| Mode | Stick pitch commands | Stick roll commands | Yaw / pedal |
|---|---|---|---|
| **Rate (current)** | pitch rate | roll rate | yaw rate |
| **G-command (C\*)** | normal load factor, from 1 g at center to ±n_max | roll rate | auto-coordination (β → 0) |
| **Flight-path command** | flight-path angle rate | bank angle | auto-coordination |
| **Attitude** | pitch attitude | bank attitude | heading hold |

Supporting work:
- **Gain scheduling** by `q̄`: higher dynamic pressure needs less surface deflection.
- **Envelope protection:** α-limit, g-limit, over-speed. These replace the simple rate clamps used for spacecraft.
- **Coordinated-turn feed-forward:** at bank angle φ, the required yaw rate is `r = g·sin φ / V`.

### 3.4 Hand mapping for aircraft (same gestures, different intent)
| Gesture | Spacecraft (now) | Aircraft (planned) |
|---|---|---|
| Stick hand pitch | pitch rate | g / flight-path command |
| Stick hand roll | roll rate | roll rate, with bank hold on release |
| Stick hand yaw | yaw rate | sideslip / rudder trim, auto-coordinated by default |
| Pedal down / up | accelerate / brake | throttle up / down (thrust, not speed) |
| Closed stick hand | reset to launch attitude | wings-level recovery |
| Closed throttle hand | stop | idle plus speed brakes, wheel brakes on ground |

### 3.5 Pilot feedback
- **Over the stick hand:** the attitude hologram gains a flight-path marker, an AoA bracket, a g readout and a stall warning tint.
- **Over the throttle hand:** the thruster hologram shows the engines, control-surface deflections, and an airspeed tape.
- **HUD:** pitch ladder, heading tape, altitude, airspeed and vertical-speed tapes.

## 4. Milestones

| # | Milestone | Proves |
|---|---|---|
| F1 | `velocityCoupling` slider plus flight-path marker | Orientation without acceleration is controllable and visible |
| F2 | `maxTurnAcceleration`; drift-kill gesture | Momentum and turn radius in space |
| F3 | `Atmosphere`, `AerodynamicModel`, glider craft (no engine) | Attitude → lift → acceleration; stable trimmed glide |
| F4 | Aero-surface actuators with q̄-scheduled allocation | Surfaces fly the glider through the same allocator |
| F5 | G-command and flight-path modes, auto-coordination, envelope protection | Feels like a modern fly-by-wire jet |
| F6 | Jet craft, runway, landing gear, take-off and landing | Full atmospheric sortie |
| F7 | Space-plane: blended aero/RCS allocation, re-entry | Both regimes in one flight |

**Testing approach**, the same as today: every milestone lands with headless tests. Examples:
- a trimmed glide reaches steady state;
- a 60° bank gives about 2 g in a level turn;
- the stall breaks at α_crit;
- the coordinated-turn sideslip stays under 1°.
