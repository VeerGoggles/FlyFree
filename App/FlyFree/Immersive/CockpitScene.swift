import FlyFreeCore
import RealityKit
import UIKit

/// Builds and animates the immersive scene. Everything is procedural so the project needs
/// no binary assets; swap in Reality Composer Pro cockpits per craft later.
///
///     root
///     ├─ flight (fades in)            backdrop, sky, field, world, cockpit
///     │   ├─ sky    rotates only      stars, sun, planet, nebula
///     │   ├─ field  rotates; wraps    asteroids, space dust
///     │   ├─ world  rotates+moves     gate course
///     │   └─ cockpit fixed            frame, consoles, score hologram, HUD, dashboard
///     └─ guide (never faded)          hand plates, hand holograms, calibration panel
@MainActor
final class CockpitScene {
    let root = Entity()
    private let guide = Entity()
    private let flight = Entity()
    let cockpit = Entity()
    private let sky = Entity()
    private let world = Entity()
    private let field = Entity()

    private var asteroids: [(entity: ModelEntity, base: Vec3)] = []
    private var dust: [(entity: ModelEntity, base: Vec3)] = []
    private let fieldSize = 3000.0
    private let dustSize = 90.0

    private var gates: [(gate: Gate, ring: ModelEntity, halo: ModelEntity)] = []
    private var gateStyles: [Int: GateStyle] = [:]

    private var plates: [FlyFreeCore.Chirality: HandPlate] = [:]
    private let attitude = AttitudeHologram()
    private let thrusters: ThrusterHologram
    private let score = ScoreHologram()

    /// The point the universe pivots around — roughly a seated pilot's head.
    private let pivot = SIMD3<Float>(0, 1.2, 0)

    init(craft: CraftDefinition, course: Course) {
        thrusters = ThrusterHologram(craft: craft)
        root.addChild(flight)
        root.addChild(guide)
        flight.components.set(OpacityComponent(opacity: 0))
        for e in [sky, field, world, cockpit] { flight.addChild(e) }

        buildBackdrop()
        buildSky()
        buildField()
        buildCourse(course)
        buildCockpit(style: craft.cockpit)

        for chirality in FlyFreeCore.Chirality.allCases {
            let plate = HandPlate()
            guide.addChild(plate.entity)
            plates[chirality] = plate
        }
        guide.addChild(attitude.entity)
        guide.addChild(thrusters.entity)
        // Upper right of the windscreen, angled toward the pilot, clear of the flight path.
        score.entity.position = [0.48, 1.58, -1.05]
        score.entity.orientation = simd_quatf(angle: -0.4, axis: [0, 1, 0])
        cockpit.addChild(score.entity)
    }

    func mountInCockpit(_ entity: Entity?, position: SIMD3<Float>, yaw: Float, pitch: Float) {
        mount(entity, in: cockpit, position: position, yaw: yaw, pitch: pitch)
    }

    func mountInGuide(_ entity: Entity?, position: SIMD3<Float>, yaw: Float, pitch: Float) {
        mount(entity, in: guide, position: position, yaw: yaw, pitch: pitch)
    }

    private func mount(_ entity: Entity?, in parent: Entity, position: SIMD3<Float>, yaw: Float, pitch: Float) {
        guard let entity else { return }
        entity.position = position
        entity.orientation = simd_quatf(angle: yaw, axis: [0, 1, 0]) * simd_quatf(angle: pitch, axis: [1, 0, 0])
        parent.addChild(entity)
    }

    // MARK: - Per frame

    func apply(_ model: AppModel) {
        flight.components.set(OpacityComponent(opacity: model.sceneOpacity))
        updateHands(model)
        score.update(course: model.course, lastEvent: model.lastGateEvent, now: model.clock)
        updateGates(model)
        guard let telemetry = model.telemetry else { return }

        let s = telemetry.state
        let q = s.orientation
        let inverse = simd_quatf(ix: Float(-q.imag.x), iy: Float(-q.imag.y), iz: Float(-q.imag.z), r: Float(q.real))
        let position = SIMD3<Float>(Float(s.position.x), Float(s.position.y), Float(s.position.z))

        sky.orientation = inverse
        sky.position = pivot
        field.orientation = inverse
        field.position = pivot
        world.orientation = inverse
        world.position = pivot + inverse.act(-position)

        // Wrap in Double so positions stay precise however far the craft has flown.
        wrap(asteroids, around: s.position, size: fieldSize)
        wrap(dust, around: s.position, size: dustSize)
    }

    private func wrap(_ items: [(entity: ModelEntity, base: Vec3)], around position: Vec3, size: Double) {
        for (entity, base) in items {
            var offset = base - position
            offset -= size * ((offset + size / 2) / size).rounded(.down)
            entity.position = SIMD3(Float(offset.x), Float(offset.y), Float(offset.z))
        }
    }

    private func updateHands(_ model: AppModel) {
        let input = model.telemetry?.input
        var calibrating = false
        if case .calibrating = model.phase { calibrating = true }

        for (chirality, plate) in plates {
            let style: HandPlate.Style
            if calibrating {
                let readiness = chirality == .left ? model.calibrationProgress.left : model.calibrationProgress.right
                style = readiness == .ready ? .ready : .waiting
            } else if chirality == model.dominantHand {
                style = input?.stickClosed == true ? .resetting : (input?.engaged == true ? .engaged : .released)
            } else {
                style = input?.throttleClosed == true ? .braking : .engaged
            }
            plate.update(plane: model.handPlanes[chirality], style: style, time: model.clock)
        }

        let flying = !calibrating && model.telemetry != nil
        attitude.follow(model.handPlanes[model.dominantHand], visible: flying)
        attitude.update(model.telemetry)
        thrusters.follow(model.handPlanes[model.dominantHand.opposite], visible: flying)
        thrusters.update(model.telemetry)
    }

    // MARK: - Course

    private enum GateStyle: Equatable { case pending, next, passed, flash }

    private func buildCourse(_ course: Course) {
        let ringMesh = MeshFactory.torus(major: 26, minor: 0.7)
        let haloMesh = MeshFactory.torus(major: 26, minor: 2.2, sides: 8)
        for gate in course.gates {
            let ring = ModelEntity(mesh: ringMesh)
            let halo = ModelEntity(mesh: haloMesh)
            let c = gate.center
            for e in [ring, halo] {
                e.position = SIMD3(Float(c.x), Float(c.y), Float(c.z))
                world.addChild(e)
            }
            // Four marker lights on the ring.
            for k in 0..<4 {
                let a = Float(k) * .pi / 2 + .pi / 4
                let light = ModelEntity(mesh: .generateSphere(radius: 1.3), materials: [MeshFactory.glow(.white)])
                light.position = SIMD3(cos(a) * 26, sin(a) * 26, 0)
                ring.addChild(light)
            }
            gates.append((gate, ring, halo))
        }
    }

    private func updateGates(_ model: AppModel) {
        let course = model.course
        for (gate, ring, halo) in gates {
            var style: GateStyle
            switch course.state(of: gate) {
            case .pending: style = .pending
            case .next: style = .next
            case .passed(let at): style = model.clock - at < 0.5 ? .flash : .passed
            }
            // The next gate breathes so it's easy to find.
            if style == .next {
                let pulse = 1 + 0.04 * Float(sin(model.clock * 5))
                halo.scale = SIMD3(pulse, pulse, 1)
            } else {
                halo.scale = .one
            }
            guard gateStyles[gate.id] != style else { continue }
            gateStyles[gate.id] = style
            let (color, ringOpacity, haloOpacity): (UIColor, Float, Float) = switch style {
            case .pending: (.holoCyan, 0.55, 0.06)
            case .next: (.holoAmber, 1, 0.25)
            case .flash: (.white, 1, 0.6)
            case .passed: (.holoGreen, 0.45, 0.05)
            }
            ring.model?.materials = [MeshFactory.glow(color, opacity: ringOpacity)]
            halo.model?.materials = [MeshFactory.glow(color, opacity: haloOpacity)]
        }
    }

    // MARK: - Backdrop & sky

    /// An opaque inside-out sphere that hides the room once the scene has faded in.
    private func buildBackdrop() {
        let backdrop = ModelEntity(mesh: .generateSphere(radius: 4000),
                                   materials: [MeshFactory.glow(UIColor(red: 0.004, green: 0.004, blue: 0.015, alpha: 1))])
        backdrop.scale = [-1, 1, 1]  // flip winding so the inside faces are rendered
        backdrop.position = pivot
        flight.addChild(backdrop)
    }

    private func buildSky() {
        var rng = SeededRandom(seed: 144)

        // Stars in three tints, with a few bright ones.
        let tints: [UIColor] = [.white, UIColor(red: 0.75, green: 0.85, blue: 1, alpha: 1), UIColor(red: 1, green: 0.88, blue: 0.7, alpha: 1)]
        for tint in tints {
            let centers = (0..<1100).map { _ in rng.unitVector() * 1800 }
            let sizes = centers.indices.map { _ in rng.next() < 0.04 ? 5 + rng.next() * 4 : 1.2 + rng.next() * 2.4 }
            sky.addChild(ModelEntity(mesh: MeshFactory.points(centers, size: { sizes[$0] }), materials: [MeshFactory.glow(tint)]))
        }

        // Nebula: layered translucent clouds.
        let nebula: [(SIMD3<Float>, Float, UIColor)] = [
            ([-0.6, 0.35, -0.7], 520, UIColor(red: 0.45, green: 0.2, blue: 0.75, alpha: 1)),
            ([-0.5, 0.45, -0.75], 380, UIColor(red: 0.2, green: 0.35, blue: 0.9, alpha: 1)),
            ([-0.7, 0.25, -0.6], 300, UIColor(red: 0.9, green: 0.3, blue: 0.55, alpha: 1)),
            ([0.8, -0.3, 0.5], 450, UIColor(red: 0.15, green: 0.45, blue: 0.6, alpha: 1)),
        ]
        for (dir, radius, color) in nebula {
            for layer in 0..<3 {
                let cloud = ModelEntity(mesh: .generateSphere(radius: radius * (1 - Float(layer) * 0.25)),
                                        materials: [MeshFactory.glow(color, opacity: 0.035 + Float(layer) * 0.02)])
                cloud.position = simd_normalize(dir) * 3000 + rng.unitVector() * 60
                sky.addChild(cloud)
            }
        }

        // Sun with a soft corona, and the light it casts.
        let sunDirection = simd_normalize(SIMD3<Float>(0.55, 0.4, 0.3))
        let sun = Entity()
        sun.position = sunDirection * 3200
        sun.addChild(ModelEntity(mesh: .generateSphere(radius: 60), materials: [MeshFactory.glow(UIColor(red: 1, green: 0.97, blue: 0.9, alpha: 1))]))
        for (r, o) in [(110, 0.25), (190, 0.1), (320, 0.04)] as [(Float, Float)] {
            sun.addChild(ModelEntity(mesh: .generateSphere(radius: r), materials: [MeshFactory.glow(UIColor(red: 1, green: 0.85, blue: 0.6, alpha: 1), opacity: o)]))
        }
        sky.addChild(sun)
        let light = DirectionalLight()
        light.light.intensity = 4000
        light.light.color = UIColor(red: 1, green: 0.95, blue: 0.88, alpha: 1)
        light.look(at: .zero, from: sunDirection * 100, relativeTo: nil)
        sky.addChild(light)

        // A gas giant with an atmosphere rim and rings.
        let planet = Entity()
        planet.position = simd_normalize(SIMD3<Float>(-0.35, -0.12, -0.9)) * 2900
        var surface = PhysicallyBasedMaterial()
        surface.baseColor = .init(tint: UIColor(red: 0.75, green: 0.55, blue: 0.4, alpha: 1))
        surface.roughness = 0.9
        planet.addChild(ModelEntity(mesh: .generateSphere(radius: 420), materials: [surface]))
        planet.addChild(ModelEntity(mesh: .generateSphere(radius: 445), materials: [MeshFactory.glow(UIColor(red: 0.5, green: 0.7, blue: 1, alpha: 1), opacity: 0.12)]))
        let rings = ModelEntity(mesh: MeshFactory.annulus(inner: 560, outer: 820, segments: 128),
                                materials: [MeshFactory.glow(UIColor(red: 0.85, green: 0.75, blue: 0.6, alpha: 1), opacity: 0.35)])
        rings.orientation = simd_quatf(angle: 0.35, axis: [1, 0, 0.3])
        planet.addChild(rings)
        sky.addChild(planet)
    }

    // MARK: - Asteroids & dust

    private func buildField() {
        var rng = SeededRandom(seed: 2026)
        let rockTones: [UIColor] = [UIColor(white: 0.42, alpha: 1), UIColor(red: 0.45, green: 0.38, blue: 0.32, alpha: 1), UIColor(red: 0.35, green: 0.36, blue: 0.4, alpha: 1)]
        let meshes = (0..<3).map { _ in MeshResource.generateSphere(radius: 1) }
        for i in 0..<180 {
            var rock = PhysicallyBasedMaterial()
            rock.baseColor = .init(tint: rockTones[i % rockTones.count])
            rock.roughness = 0.95
            let asteroid = ModelEntity(mesh: meshes[i % meshes.count], materials: [rock])
            let radius = 3 + rng.next() * 25
            asteroid.scale = SIMD3(radius, radius * (0.55 + rng.next() * 0.6), radius * (0.7 + rng.next() * 0.5))
            asteroid.orientation = simd_quatf(angle: rng.next() * 6.28, axis: rng.unitVector())
            var p = Vec3(Double(rng.next()) * 2 - 1, Double(rng.next()) * 2 - 1, Double(rng.next()) * 2 - 1) * fieldSize / 2
            if simd_length(p) < 120 { p *= 4 }
            field.addChild(asteroid)
            asteroids.append((asteroid, p))
        }

        // Space dust: tiny specks close to the craft — the strongest cue for speed and turning.
        let speck = MeshResource.generateBox(size: 0.06)
        let dustMaterial = MeshFactory.glow(UIColor(red: 0.8, green: 0.9, blue: 1, alpha: 1), opacity: 0.7)
        for _ in 0..<260 {
            let e = ModelEntity(mesh: speck, materials: [dustMaterial])
            let p = Vec3(Double(rng.next()) * 2 - 1, Double(rng.next()) * 2 - 1, Double(rng.next()) * 2 - 1) * dustSize / 2
            field.addChild(e)
            dust.append((e, p))
        }
    }

    // MARK: - Cockpit

    private func buildCockpit(style: CockpitStyle) {
        let hull = MeshFactory.metal(0.16, roughness: 0.55)
        let trim = MeshFactory.metal(0.42, roughness: 0.3)
        let accent = MeshFactory.emissive(.holoCyan, intensity: 2)
        let warm = MeshFactory.emissive(UIColor(red: 1, green: 0.55, blue: 0.15, alpha: 1), intensity: 2)
        let screen = MeshFactory.emissive(UIColor(red: 0.05, green: 0.25, blue: 0.35, alpha: 1), intensity: 1)

        func box(_ size: SIMD3<Float>, _ position: SIMD3<Float>, _ material: any RealityKit.Material, tilt: Float = 0, radius: Float = 0.01) {
            let e = ModelEntity(mesh: .generateBox(size: size, cornerRadius: radius), materials: [material])
            e.position = position
            e.orientation = simd_quatf(angle: tilt, axis: [1, 0, 0])
            cockpit.addChild(e)
        }

        func strut(from a: SIMD3<Float>, to b: SIMD3<Float>, thickness: Float = 0.045) {
            let e = ModelEntity(mesh: .generateBox(size: [thickness, simd_length(b - a), thickness], cornerRadius: 0.01), materials: [trim])
            e.position = (a + b) / 2
            e.orientation = simd_quatf(from: [0, 1, 0], to: simd_normalize(b - a))
            cockpit.addChild(e)
        }

        // Floor with light strips.
        box([1.8, 0.03, 2.0], [0, 0, -0.2], hull)
        for x: Float in [-0.6, 0.6] { box([0.015, 0.005, 1.6], [x, 0.018, -0.2], accent, radius: 0.002) }

        // Main console, with dark screens and a glowing edge.
        box([1.3, 0.05, 0.4], [0, 0.82, -0.6], hull, tilt: 0.35)
        box([1.3, 0.8, 0.05], [0, 0.42, -0.78], hull)
        box([1.24, 0.008, 0.012], [0, 0.875, -0.415], accent, radius: 0.003)
        for x: Float in [-0.42, 0.42] { box([0.26, 0.006, 0.2], [x, 0.855, -0.6], screen, tilt: 0.35, radius: 0.004) }

        // Side consoles.
        for x: Float in [-0.68, 0.68] {
            box([0.3, 0.06, 0.8], [x, 0.7, -0.15], hull)
            box([0.3, 0.7, 0.05], [x, 0.35, -0.55], hull)
            box([0.008, 0.008, 0.76], [x + (x < 0 ? 0.15 : -0.15), 0.735, -0.15], warm, radius: 0.002)
        }

        // Canopy frame.
        let height: Float = style == .fighterCanopy ? 1.75 : 2.05
        let sill: [SIMD3<Float>] = [[-0.8, 0.9, -0.8], [0.8, 0.9, -0.8], [0.9, 0.9, 0.4], [-0.9, 0.9, 0.4]]
        for i in 0..<sill.count { strut(from: sill[i], to: sill[(i + 1) % sill.count]) }
        strut(from: [-0.8, 0.9, -0.8], to: [-0.45, height, -0.35])
        strut(from: [0.8, 0.9, -0.8], to: [0.45, height, -0.35])
        strut(from: [-0.45, height, -0.35], to: [0.45, height, -0.35])
        strut(from: [-0.45, height, -0.35], to: [-0.6, height, 0.5])
        strut(from: [0.45, height, -0.35], to: [0.6, height, 0.5])
        strut(from: [-0.9, 0.9, 0.4], to: [-0.6, height, 0.5])
        strut(from: [0.9, 0.9, 0.4], to: [0.6, height, 0.5])
        // Thin glowing inlay along the windscreen top.
        box([0.86, 0.006, 0.006], [0, height - 0.03, -0.33], accent, radius: 0.002)

        // Interior fill light so the cockpit is readable in deep space.
        let fill = PointLight()
        fill.light.intensity = 600
        fill.light.attenuationRadius = 3
        fill.light.color = UIColor(red: 0.8, green: 0.9, blue: 1, alpha: 1)
        fill.position = [0, 1.9, 0.2]
        cockpit.addChild(fill)
    }
}
