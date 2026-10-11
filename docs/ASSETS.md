# Scenery and assets

FlyFree ships **no binary assets**. Every scene is built from code in two layers.

## Layer 1: FlyFree procedural (instant)

`App/FlyFree/Immersive/CockpitScene.swift` builds every scene immediately from RealityKit primitives and custom meshes:
- cockpit, starfield, nebula, ringed planet, asteroids and dust;
- sky, haze, ground, runway, trees and houses, mountains and clouds;
- gates and instruments.

This layer is always present and is the fallback.

## Layer 2: RealityHD (photoreal, built at load time)

[RealityHD](https://github.com/hunterh37/RealityHD) (MIT) generates photoreal geometry on the CPU and synthesizes PBR texture sets on the GPU when the app runs. `App/FlyFree/Immersive/HDScenery.swift` builds the upgrades in the background while you hold your hands out to calibrate. `CockpitScene.upgradeToHD` then swaps them in, hiding the procedural pieces they replace. If anything fails, the procedural scene stays. The hangar's **HD scenery** toggle turns the layer off.

| Scene | RealityHD upgrade |
|---|---|
| Both cockpits | Structural shell in `composite.carbon`, `plastic.dash`, `rubber.floor-mat`, and a `metal.titanium` (capsule) or `metal.anodized-black` (fighter) canopy frame; gold `insulation.foil-gold` blankets in the capsule. Emissive trims, screens and instruments stay procedural. |
| Deep space | Asteroids become faceted `Boulder`s in basalt, granite, weathered granite, sandstone and slate, with LOD distances tuned for kilometre-scale viewing. Aerial-perspective fog is off (vacuum). |
| Earth (jet) | Physically based `SunSky` sky, sun and image-based lighting, rotating with the aircraft's attitude, with thin aerial-perspective haze. A 24 km `ground.meadow` heightfield, moved in 120 m steps so its texture stays world-locked. An `asphalt.road` runway with `paint.lane-white` centre line and thresholds. About 1,600 instanced spruce, Scots pine and oak trees in forest patches, including woods lining both sides of the airfield. 70 instanced boulder outcrops. 46 displaced `rock.mountain` peaks with `ground.snow` caps, built at a 150 m base radius so the rock texture is not stretched. |

All textures use metre-scale UVs (`Prim`), so materials keep real-world texel density on a 3 km runway and on kilometre-sized mountains.

### Why RealityHD is vendored

The vendored copy is in `Vendor/RealityHD`. RealityHD targets a newer Xcode than the 26.0 / 26.1 toolchains, which time out type-checking a handful of long SIMD expressions and fail some type inferences.

`Vendor/RealityHD` is a copy of the upstream library targets at commit `c01e31b`, with these changes:
- The animal targets, the CLI targets, the tests, the docs and the images are removed.
- `Package.swift` lists only `RealCore`, `RealMaterials`, `RealKit` and `RealLibrary`.
- Each failing expression is split into typed sub-expressions with identical math. Every patch is marked `// FlyFree patch:`.

To update it, copy the upstream `Sources/{RealCore,RealMaterials,RealKit,RealLibrary}` over the vendored copy and re-apply the marked patches. A one-line fix to `project.yml` switches back to the upstream package once Xcode compiles it as-is:

```yaml
RealityHD:
  url: https://github.com/hunterh37/RealityHD.git
  revision: <commit>
```

Requirements: RealityHD needs visionOS 26, so FlyFree's deployment target is visionOS 26.0.
