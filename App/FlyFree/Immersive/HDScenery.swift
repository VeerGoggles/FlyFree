import FlyFreeCore
import RealCore
import RealKit
import RealLibrary
import RealityKit
import UIKit

/// Photoreal upgrades from RealityHD (https://github.com/hunterh37/RealityHD): procedural
/// geometry built on the CPU and PBR textures synthesized on the GPU at load time — no asset
/// files ship with the app.
///
/// Every scene starts with FlyFree's own lightweight procedural visuals; these builders run in
/// the background (during hand calibration) and the scene swaps the results in when ready. If
/// anything fails, the procedural scene simply stays.
@MainActor
enum HDScenery {
    static func setup() {
        RealityHD.setup(.balanced)
    }

    // MARK: - Cockpit

    /// One rounded box with metre-scale UVs and a RealityHD material.
    static func box(_ size: SIMD3<Float>, radius: Float = 0.01, material: MaterialKey) async throws -> ModelEntity {
        try await Model(name: material, surfaces: [Prim.roundedBox(size, radius: radius, material: material)]).modelEntityAsync()
    }

    /// The cockpit's structural shell (floor, consoles, canopy frame) in PBR materials. Mirrors
    /// the procedural layout in `CockpitScene.buildCockpit`; emissive trims and instruments stay.
    static func cockpitShell(style: CockpitStyle) async throws -> Entity {
        let shell = Entity()
        let frameMaterial = style == .fighterCanopy ? "metal.anodized-black" : "metal.titanium"

        func add(_ size: SIMD3<Float>, _ position: SIMD3<Float>, _ material: MaterialKey, tilt: Float = 0) async throws {
            let e = try await box(size, material: material)
            e.position = position
            e.orientation = simd_quatf(angle: tilt, axis: [1, 0, 0])
            shell.addChild(e)
        }
        func strut(from a: SIMD3<Float>, to b: SIMD3<Float>) async throws {
            let e = try await box([0.045, simd_length(b - a), 0.045], material: frameMaterial)
            e.position = (a + b) / 2
            e.orientation = simd_quatf(from: [0, 1, 0], to: simd_normalize(b - a))
            shell.addChild(e)
        }

        try await add([1.8, 0.03, 2.0], [0, 0, -0.2], "rubber.floor-mat")
        try await add([1.3, 0.05, 0.4], [0, 0.82, -0.6], "composite.carbon", tilt: 0.35)
        try await add([1.3, 0.8, 0.05], [0, 0.42, -0.78], "plastic.dash")
        for x: Float in [-0.68, 0.68] {
            try await add([0.3, 0.06, 0.8], [x, 0.7, -0.15], "composite.carbon")
            try await add([0.3, 0.7, 0.05], [x, 0.35, -0.55], "plastic.dash")
        }
        let height: Float = style == .fighterCanopy ? 2.0 : 2.05
        let sill: [SIMD3<Float>] = [[-0.8, 0.9, -0.8], [0.8, 0.9, -0.8], [0.9, 0.9, 0.4], [-0.9, 0.9, 0.4]]
        for i in 0..<sill.count { try await strut(from: sill[i], to: sill[(i + 1) % sill.count]) }
        try await strut(from: [-0.8, 0.9, -0.8], to: [-0.45, height, -0.35])
        try await strut(from: [0.8, 0.9, -0.8], to: [0.45, height, -0.35])
        try await strut(from: [-0.45, height, -0.35], to: [0.45, height, -0.35])
        try await strut(from: [-0.45, height, -0.35], to: [-0.6, height, 0.5])
        try await strut(from: [0.45, height, -0.35], to: [0.6, height, 0.5])
        try await strut(from: [-0.9, 0.9, 0.4], to: [-0.6, height, 0.5])
        try await strut(from: [0.9, 0.9, 0.4], to: [0.6, height, 0.5])
        if style == .spacecraftCapsule {
            // Gold-foil insulation blankets behind the side consoles.
            for x: Float in [-0.86, 0.86] { try await add([0.02, 0.5, 0.9], [x, 1.15, 0.0], "insulation.foil-gold") }
        }
        return shell
    }

    // MARK: - Space

    /// Asteroid prototypes: faceted boulders in basalt, granite and sandstone, ~1 m across, with
    /// LOD distances suited to kilometre-scale viewing. Clone and scale per asteroid.
    static func asteroidPrototypes() async throws -> [Entity] {
        let materials: [MaterialKey] = ["rock.basalt", "rock.granite-weathered", "rock.granite", "rock.sandstone", "rock.basalt", "rock.slate"]
        var prototypes: [Entity] = []
        for (i, material) in materials.enumerated() {
            let rock = Boulder().with {
                $0.size = [1.0, 0.75 + 0.05 * Float(i % 3), 0.9]
                $0.material = material
                $0.facets = 10 + i
                $0.chips = 14
                $0.roughness = 0.45
                $0.lodDistances = [250, 900]
            }
            prototypes.append(try await RealityHD.entity(rock, seed: UInt64(11 + i * 7)))
        }
        return prototypes
    }

    // MARK: - Earth

    struct Earth {
        var environment: RealEnvironment
        var ground: Entity
        var runway: Entity
        var forests: Entity
        var mountains: Entity
    }

    static func earth() async throws -> Earth {
        // Physically based sky + sun + IBL. Aerial-perspective fog thinned for kilometre-scale flight.
        var sky = SunSky(elevation: 38, azimuth: 215, turbidity: 2.4)
        sky.fogDensity = 0.00006
        sky.shadowDistance = 120
        let environment = try RealEnvironment(sky, skybox: true, skyboxRadius: 13000)

        // Ground: one large meadow heightfield (flat; UVs in metres keep texture scale real).
        let ground = try await Model(name: "ground", surfaces: [
            Prim.terrain(size: [24000, 24000], segments: 48, material: "ground.meadow") { _ in 0 },
        ]).modelEntityAsync()

        // Runway: road asphalt, painted centre line and threshold bars.
        let runway = Entity()
        let asphalt = try await box([45, 0.1, 3000], radius: 0.02, material: "asphalt.road")
        asphalt.position = [0, 0.03, -1300]
        runway.addChild(asphalt)
        let dash = try await box([0.9, 0.02, 30], radius: 0.005, material: "paint.lane-white")
        for i in 0..<48 {
            let d = dash.clone(recursive: true)
            d.position = [0, 0.09, 150 - Float(i) * 60]
            runway.addChild(d)
        }
        let bar = try await box([1.8, 0.02, 30], radius: 0.005, material: "paint.lane-white")
        for end: Float in [170, -2770] {
            for k in 0..<8 {
                let b = bar.clone(recursive: true)
                b.position = [Float(k - 4) * 4.5 + 2.25, 0.09, end]
                runway.addChild(b)
            }
        }

        // Forests: patches of instanced spruce, Scots pine and oak around the valley, kept off
        // the runway and its approaches.
        var rng = SeededRandom(seed: 77)
        var spruce: [simd_float4x4] = [], pine: [simd_float4x4] = [], oak: [simd_float4x4] = []
        for patch in 0..<38 {
            var center = SIMD2<Float>(rng.next() * 16000 - 5000, rng.next() * 16000 - 12000)
            if patch < 12 {
                // Woods lining the airfield, close enough to see from the runway.
                let side: Float = patch % 2 == 0 ? 1 : -1
                center = SIMD2(side * (220 + rng.next() * 900), -3000 + rng.next() * 3300)
            }
            if abs(center.x) < 350, center.y > -3600, center.y < 900, patch >= 12 { center.x += 900 }
            let radius = 120 + rng.next() * 260
            for _ in 0..<44 {
                let a = rng.next() * 2 * .pi, r = radius * rng.next().squareRoot()
                let p = center + SIMD2(cos(a), sin(a)) * r
                if abs(p.x) < 120, p.y > -3300, p.y < 500 { continue }
                let t = Transform(scale: SIMD3(repeating: 0.8 + rng.next() * 0.5),
                                  rotation: simd_quatf(angle: rng.next() * 2 * .pi, axis: [0, 1, 0]),
                                  translation: [p.x, 0, p.y]).matrix
                switch rng.next() {
                case ..<0.45: spruce.append(t)
                case ..<0.75: pine.append(t)
                default: oak.append(t)
                }
            }
        }
        var options = RealInstancing.Options()
        options.cellSize = 400
        options.cullDistance = 2400
        let forests = Entity()
        forests.addChild(try await RealityHD.field(SpruceTree(), seed: 3, transforms: spruce, options: options))
        forests.addChild(try await RealityHD.field(ScotsPine(), seed: 5, transforms: pine, options: options))
        forests.addChild(try await RealityHD.field(OakTree(), seed: 9, transforms: oak, options: options))

        // Rock outcrops scattered near the airfield.
        var rocks: [simd_float4x4] = []
        for _ in 0..<70 {
            var p = SIMD2<Float>(rng.next() * 3000 - 1500, rng.next() * 4000 - 3200)
            if abs(p.x) < 90 { p.x += p.x < 0 ? -120 : 120 }
            rocks.append(Transform(scale: SIMD3(repeating: 1 + rng.next() * 2.5),
                                   rotation: simd_quatf(angle: rng.next() * 6.28, axis: [0, 1, 0]),
                                   translation: [p.x, 0, p.y]).matrix)
        }
        var rockOptions = RealInstancing.Options()
        rockOptions.cellSize = 300
        rockOptions.cullDistance = 1800
        forests.addChild(try await RealityHD.field(Boulder().with { $0.lodDistances = [60, 250] }, seed: 21, transforms: rocks, options: rockOptions))

        // Mountains: displaced cube-spheres in mountain rock with snow caps. Built at a 150 m base
        // radius (metre UVs) and scaled up ~5–13×, so the rock texture reads as boulder-scale
        // detail on a kilometre-high peak instead of being stretched a thousandfold.
        let base: Float = 150
        let mountains = Entity()
        var prototypes: [ModelEntity] = []
        for seed in 0..<5 {
            let s = Float(seed) * 1.7
            let surface = Prim.cubeSphere(subdivisions: 22, material: "rock.mountain") { d in
                let ridges = sin(d.x * 7 + s) * sin(d.z * 6.3 - s) * 0.5 + 0.5
                let bumps = sin(d.x * 17 + d.z * 13 + s * 3) * 0.04
                return d * (base * (1 + 0.16 * ridges * ridges + bumps))
            }
            prototypes.append(try await Model(name: "mountain", surfaces: [surface]).modelEntityAsync())
        }
        let snowCap = try await Model(name: "snow", surfaces: [
            Prim.cubeSphere(subdivisions: 12, material: "ground.snow") { $0 * base },
        ]).modelEntityAsync()
        var mrng = SeededRandom(seed: 1903)
        for i in 0..<46 {
            let a = Float(i) / 46 * 2 * .pi + mrng.next() * 0.1
            let distance = 7500 + mrng.next() * 4000
            let radius = 700 + mrng.next() * 1300
            let height = radius * (0.6 + mrng.next() * 0.7)
            let mountain = prototypes[i % prototypes.count].clone(recursive: true)
            mountain.scale = [radius / base, height / base, radius * (0.8 + mrng.next() * 0.4) / base]
            mountain.position = [cos(a) * distance + 3000, 0, sin(a) * distance - 5000]
            if height > 1100 {
                let cap = snowCap.clone(recursive: true)
                cap.scale = [0.44, 0.27, 0.44]
                cap.position = [0, 0.85 * base, 0]
                mountain.addChild(cap)
            }
            mountains.addChild(mountain)
        }

        return Earth(environment: environment, ground: ground, runway: runway, forests: forests, mountains: mountains)
    }
}
