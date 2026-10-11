# Contributing to FlyFree

1. **Keep the core pure.** Anything that can be expressed without ARKit, RealityKit or SwiftUI belongs in `Packages/FlyFreeCore` and needs a test.
2. **New craft = new data.** Add a `CraftDefinition` (mass, actuators, profile, cockpit style). Only add code when you need a new actuator kind or a new environment model.
3. **Run the tests:** `cd Packages/FlyFreeCore && swift test`.
4. **Changed `project.yml`?** Run `xcodegen generate` and commit the regenerated `FlyFree.xcodeproj`.
5. **Coordinates:** body frame is +X right, +Y up, −Z forward. Rotations use the right-hand rule (+X pitch up, +Y yaw left, +Z roll left).
6. **Comfort is a feature.** Anything that moves the pilot's view should consider motion sickness.

By contributing, you agree that your contributions are licensed under the MIT license.
