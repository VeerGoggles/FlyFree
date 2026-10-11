# FlyFree ✋🚀

**Fly a spacecraft or a jet with your bare hands on Apple Vision Pro.**

Hold your hands out **flat**. FlyFree fits a plane to each hand's fingers and measures how it rotates from your calibrated neutral pose.

- **Stick hand:** fingertips up or down to pitch, turn the wrist to yaw, rotate the forearm to roll.
- **Throttle hand:** works like a car's gas pedal. Push the fingertips down to accelerate, hold it level to keep your speed, and tip it up to brake.
- **Close either hand** for a quick reset: a closed stick hand returns the craft to its starting orientation, and a closed throttle hand brakes to a full stop.

That 3D hand motion becomes a *desired craft state* in the craft's current frame. A translator then works out which engines, thrusters or control surfaces will achieve it. You sit in a cockpit that matches the craft you picked; it fades in around you once your hands lock in.

> 🛠 Originated at **SundAI 144**. Open source under the MIT license.

---

## Status: v0.1, the first slice through the whole system

| Works today | Next up |
|---|---|
| Generic 12 t spacecraft: 2 main engines and a 14-thruster RCS, in deep space with no gravity, drag or wind | Calibration flow, RCS strafing, more craft |
| Flat-hand stick from a plane fitted to about 20 finger joints | Atmospheric jet with aero surfaces |
| "Hold out both hands, flat" calibration, then the cockpit and space fade in | Haptic and audio cues |
| Flat-hand throttle pedal: push down to accelerate, level to hold speed, tip up to brake | |
| Closed hands: reset orientation (stick), brake to a stop (throttle) | |
| Separate acceleration limits for rotation and speed; braking about 3× stronger than speeding up | |
| Gate course with flown-through states, a holographic score and a course clock | |
| Completion screen: time, score and best time; goes straight on to the next craft (or Retry) | |
| **Atmospheric jet**: lift, drag and stall; control surfaces; landing gear; fly-by-wire; runway take-off; Valley Run gates | |
| Flight-path marker showing where you're actually going | |
| Speedometer with a scale on every craft (linear airspeed / logarithmic space speed) | |
| Quaternion navigation: stick deltas apply about the ship's current axes, identically at any attitude | |
| Hand holograms: ship attitude over the stick hand, live thrusters over the throttle hand | |
| Holographic hand plates; sun, ringed planet, nebula, space dust | |
| Rate-command / attitude-hold control in the craft's own frame | |
| Hangar window hides during flight; Land, Reset and Recenter are on the dashboard | |
| Velocity-hold flight assist, or fully Newtonian flight | Comfort options (vignette, horizon cues) |
| Generic thrust allocator that works for any thruster layout | |
| Procedural cockpit, starfield, asteroids and a gate course | |
| Cockpit hologram of the craft that shows each thruster firing | |
| **Simulated Hands** input, so the whole pipeline runs in the visionOS Simulator | |

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the design, [`docs/FLIGHT_MODEL.md`](docs/FLIGHT_MODEL.md) for the flight-sim plan, and [`docs/ROADMAP.md`](docs/ROADMAP.md) for the overall plan.

## How it works (one paragraph)

```
Hands ─► Interpret ─► Intent ─► Control ─► Allocate ─► Physics ─► Render
ARKit    PilotInput   Desired    Wrench     Actuator    Rigid      Cockpit +
joints   (normalized) CraftState demand     commands    body       moving world
```

Each stage is a small, pure-Swift, unit-tested type in [`FlyFreeCore`](Packages/FlyFreeCore). The craft-specific parts (mass, actuator layout, handling, cockpit) are data in a `CraftDefinition`, so adding a craft is mostly configuration.

## Getting started

Requirements: Xcode 26+ with the visionOS SDK. Runs on visionOS 2.0+.

```bash
git clone https://github.com/VeerGoggles/FlyFree.git
cd FlyFree
open FlyFree.xcodeproj          # or: brew install xcodegen && xcodegen generate
```

- **Simulator:** pick an Apple Vision Pro simulator and Run. Input defaults to *Simulated Hands*, so calibration completes on its own; use the sliders panel to the right of the cockpit. Passing the launch argument `-FlyFreeAutoLaunch` skips the hangar.
- **Device:** set your signing team, then Run. Allow hand tracking when asked. Press **Launch**, hold both hands out flat until they lock in, and fly.

Run the core tests (they run on a Mac, no headset needed):

```bash
cd Packages/FlyFreeCore && swift test
```

## Controls

| Hand | Motion | Effect |
|---|---|---|
| Both | Hold out flat, palms down, and keep still for 1.5 s | Calibrate the neutral pose; the cockpit fades in |
| Stick | Tip fingertips up or down (the plane's front-to-back angle) | Pitch (exponential response: fine near center, full rate at 90°) |
| Stick | Turn the wrist horizontally | Yaw |
| Stick | Rotate the forearm | Roll |
| Stick | Close the hand | Return to the starting orientation |
| Throttle | Push fingertips down | Accelerate (4 m/s² at full pedal) |
| Throttle | Hold level | Keep current speed |
| Throttle | Tip fingertips up | Brake, then reverse |
| Throttle | Close the hand | Brake to a full stop (12 m/s²) |

Dashboard buttons: **Recenter** (re-calibrate), **Reset** (back to start), **Land** (return to the hangar).

## Repo layout

```
App/FlyFree/            visionOS app: SwiftUI, RealityKit, ARKit
  Input/                ARKit hand tracking → HandFrame
  Model/                AppModel (settings + pipeline), simulated hands
  Immersive/            Cockpit scene and moving universe
  Views/                Hangar window and telemetry
Packages/FlyFreeCore/   Platform-independent flight pipeline and tests
docs/                   Architecture and roadmap
project.yml             XcodeGen spec (FlyFree.xcodeproj is generated from it)
```

## Contributing

Issues and PRs are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md).

## Credits

FlyFree was started at **SundAI 144**. Thanks to everyone who hacked on it.
