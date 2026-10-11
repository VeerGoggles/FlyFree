# FlyFree roadmap

Each phase ends with something you can fly. Checked items are already in the repo.

## Phase 0: Foundation (v0.1) ✅
- [x] `FlyFreeCore` pipeline: hands → input → intent → control → allocation → physics
- [x] Generic spacecraft in deep space: 2 main engines, 14 RCS thrusters
- [x] Grip-to-engage stick, recentered on each grip; finger-curl throttle
- [x] Velocity-hold and Newtonian flight assist
- [x] Generic bounded least-squares thrust allocator
- [x] Procedural cockpit, starfield, asteroids, gate course, thruster hologram, HUD
- [x] Simulated Hands for the visionOS Simulator
- [x] Unit tests on macOS

## Phase 1: Feel and feedback (v0.2)
- [ ] On-device tuning pass: thresholds, full-deflection angles, filter cutoffs, expo
- [ ] Calibration flow: open and fist per hand, fixed-neutral option, saved per user
- [ ] Visual stick and throttle "ghost" gauges near the hands
- [ ] Spatial audio: engine rumble scaled to thrust, RCS puffs, engage and release clicks
- [ ] Comfort: optional vignette during fast rotation, a stable cockpit frame, reduced-motion mode
- [ ] Settings persistence
- [ ] Lap timer through the gate course

## Phase 2: Full 6-DoF spacecraft (v0.3)
- [ ] RCS translation (strafe up, down, left, right) from the throttle hand's offset or a modifier gesture
- [ ] Retro thrust and reverse gesture; kill-rotation and kill-velocity panic gesture
- [ ] On/off RCS with PWM, thruster-out failure injection
- [ ] Docking target practice
- [ ] Per-craft Reality Composer Pro cockpit assets

## Phase 3: Atmosphere (v0.4)
- [ ] `Atmosphere` environment: density by altitude, wind, turbulence
- [ ] Aerodynamic-surface actuators (aileron, elevator, rudder, flaps), with effectiveness that scales with dynamic pressure
- [ ] Atmospheric jet craft with a fighter canopy cockpit; horizon-level assist
- [ ] Gain scheduling in the controller
- [ ] Terrain or ground plane, runway, landing gear contact

## Phase 4: More craft (v0.5+)
- [ ] Lunar lander (uniform gravity, hover and altitude hold)
- [ ] Quadcopter (rotor actuators)
- [ ] Community craft format (JSON or Swift package) and a craft-select gallery

## Phase 5: Beyond
- [ ] SharePlay formation flying and spectator mode
- [ ] Replay recording and playback of `HandFrame` streams for regression tests
- [ ] Physical controller fallback (game controller)
- [ ] Accessibility: one-handed mode, adjustable dead-man behaviour
