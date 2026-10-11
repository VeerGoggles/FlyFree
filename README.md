# FlyFree ✋🚀

**Fly an aircraft or spacecraft with your bare hands on Apple Vision Pro.**

Your **dominant hand is the stick**: grip a virtual control stick and tilt, turn and roll your hand to pitch, yaw and roll.
Your **other hand is the throttle**: curl your fingers to add thrust.
FlyFree reads the angles of your fingers and hand, works out what you want the craft to do, and then works out which engines, thrusters or control surfaces will do it. You sit in a cockpit that matches the craft you picked.

> 🛠 Originated at **SundAI 144**. Open source under the MIT license.

---

## Status: v0.1, the first slice through the whole system

| Works today | Next up |
|---|---|
| Generic 12 t spacecraft: 2 main engines and a 14-thruster RCS, in deep space with no gravity, drag or wind | Calibration flow, RCS strafing, more craft |
| Grip-to-engage stick that re-centers each time you grip (no calibration needed) | Atmospheric jet with aero surfaces |
| Finger-curl throttle with a grace period if tracking drops out | Haptic and audio cues |
| Velocity-hold flight assist, or fully Newtonian flight | Comfort options (vignette, horizon cues) |
| Generic thrust allocator that works for any thruster layout | |
| Procedural cockpit, starfield, asteroids and a gate course | |
| Cockpit hologram of the craft that shows each thruster firing | |
| **Simulated Hands** input, so the whole pipeline runs in the visionOS Simulator | |

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the design and [`docs/ROADMAP.md`](docs/ROADMAP.md) for the plan.

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

- **Simulator:** pick an Apple Vision Pro simulator and Run. Input defaults to *Simulated Hands*: toggle **Grip stick**, then move the sliders.
- **Device:** set your signing team, then Run. Allow hand tracking when asked. Press **Launch**, make a fist with your dominant hand, and fly.

Run the core tests (they run on a Mac, no headset needed):

```bash
cd Packages/FlyFreeCore && swift test
```

## Controls

| Hand | Gesture | Effect |
|---|---|---|
| Dominant | Make a fist (middle, ring and little fingers curled) | Engage the stick; this pose becomes center |
| Dominant | Tilt knuckles up or down | Pitch |
| Dominant | Turn hand left or right | Yaw |
| Dominant | Rotate the wrist | Roll |
| Dominant | Open hand | Release; the craft holds its attitude |
| Other | Curl fingers, from open to fist | Throttle 0 → 100% (target speed in velocity-hold mode) |

*Finger pointer* stick style: grip with your middle, ring and little fingers, and point the index finger to aim pitch and yaw.

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
