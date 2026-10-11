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
    /// Earth: scenery that repeats horizontally around the craft at fixed heights.
    private var clutter: [(entity: Entity, base: Vec3)] = []
    private var clouds: [(entity: Entity, base: Vec3)] = []
    private let clutterSize = 4000.0
    private let cloudSize = 12000.0
    private let ground = Entity()
    private let horizon = Entity()
    private let flightPathMarker = Entity()

    private var gates: [(gate: Gate, ring: ModelEntity, halo: ModelEntity)] = []
    private var gateStyles: [Int: GateStyle] = [:]

    private var plates: [FlyFreeCore.Chirality: HandPlate] = [:]
    /// Spinning rotor blades visible from the cockpit (multirotors), with their actuator index.
    private var rotorBlades: [(blades: Entity, actuator: Int)] = []
    private var lastSpinClock: Double?
    private let attitude = AttitudeHologram()
    private let thrusters: ThrusterHologram
    private let score = ScoreHologram()
    private let speedometer: SpeedGauge

    /// The point the universe pivots around — roughly a seated pilot's head.
    private let pivot = SIMD3<Float>(0, 1.2, 0)

    init(craft: CraftDefinition, course: Course) {
        thrusters = ThrusterHologram(craft: craft)
        speedometer = SpeedGauge(scale: craft.speedGauge)
        root.addChild(flight)
        root.addChild(guide)
        flight.components.set(OpacityComponent(opacity: 0))
        for e in [sky, field, world, cockpit] { flight.addChild(e) }

        switch craft.world {
        case .deepSpace:
            buildBackdrop()
            buildSky()
            buildField()
        case .earth:
            buildEarth(airfield: true)
        case .city:
            buildEarth(airfield: false)
            buildCity()
        }
        buildRotors(craft)
        buildCourse(course)
        buildFlightPathMarker()
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
        // Speedometer at the centre of the dashboard, angled up toward the pilot.
        speedometer.entity.position = [0, 0.99, -0.62]
        speedometer.entity.orientation = simd_quatf(angle: -0.45, axis: [1, 0, 0])
        cockpit.addChild(speedometer.entity)
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

        spinRotors(telemetry, clock: model.clock)
        let s = telemetry.state
        speedometer.update(speed: telemetry.air?.airspeed ?? telemetry.speed)
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
        wrap(clutter, around: s.position, size: clutterSize, horizontalOnly: true)
        wrap(clouds, around: s.position, size: cloudSize, horizontalOnly: true)
        ground.position = SIMD3(position.x, 0, position.z)  // an endless plain under the craft
        horizon.position = SIMD3(0, -position.y, 0)

        // Flight-path marker: where the craft is actually going, in cockpit (body) axes.
        let v = telemetry.air?.bodyAirVelocity ?? s.orientation.inverse.act(s.velocity)
        let speed = simd_length(v)
        flightPathMarker.isEnabled = speed > 3 && model.phase != .calibrating(initial: true)
        if speed > 3 {
            let dir = SIMD3<Float>(Float(v.x / speed), Float(v.y / speed), Float(v.z / speed))
            flightPathMarker.position = pivot + dir * 12
            flightPathMarker.look(at: pivot, from: flightPathMarker.position, relativeTo: cockpit)
        }
    }

    private func wrap<E: Entity>(_ items: [(entity: E, base: Vec3)], around position: Vec3, size: Double,
                                 horizontalOnly: Bool = false) {
        for (entity, base) in items {
            var offset = base - position
            var wrapped = offset - size * ((offset + size / 2) / size).rounded(.down)
            if horizontalOnly { wrapped.y = offset.y }
            offset = wrapped
            entity.position = SIMD3(Float(offset.x), Float(offset.y), Float(offset.z))
        }
    }

    // MARK: - Flight-path marker

    private func buildFlightPathMarker() {
        let green = MeshFactory.glow(.holoGreen, opacity: 0.9)
        let ring = ModelEntity(mesh: MeshFactory.annulus(inner: 0.16, outer: 0.2), materials: [green])
        ring.orientation = simd_quatf(angle: .pi / 2, axis: [1, 0, 0])  // face along the line of sight
        flightPathMarker.addChild(ring)
        for x: Float in [-0.32, 0.32] {
            let wing = ModelEntity(mesh: .generateBox(size: [0.2, 0.025, 0.01]), materials: [green])
            wing.position = [x, 0, 0]
            flightPathMarker.addChild(wing)
        }
        let fin = ModelEntity(mesh: .generateBox(size: [0.025, 0.14, 0.01]), materials: [green])
        fin.position = [0, 0.25, 0]
        flightPathMarker.addChild(fin)
        cockpit.addChild(flightPathMarker)
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
        // Unit ring (radius 1), scaled per gate and turned to face along the gate's direction.
        let ringMesh = MeshFactory.torus(major: 1.04, minor: 0.028)
        let haloMesh = MeshFactory.torus(major: 1.04, minor: 0.085, sides: 8)
        let lightMesh = MeshResource.generateSphere(radius: 0.05)
        for gate in course.gates {
            let ring = ModelEntity(mesh: ringMesh)
            let halo = ModelEntity(mesh: haloMesh)
            let c = gate.center
            let d = gate.direction
            let facing = simd_quatf(from: [0, 0, -1], to: SIMD3(Float(d.x), Float(d.y), Float(d.z)))
            for e in [ring, halo] {
                e.position = SIMD3(Float(c.x), Float(c.y), Float(c.z))
                e.orientation = facing
                e.scale = SIMD3(repeating: Float(gate.radius))
                world.addChild(e)
            }
            for k in 0..<4 {
                let a = Float(k) * .pi / 2 + .pi / 4
                let light = ModelEntity(mesh: lightMesh, materials: [MeshFactory.glow(.white)])
                light.position = SIMD3(cos(a), sin(a), 0) * 1.04
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
            let r = Float(gate.radius)
            if style == .next {
                let pulse = 1 + 0.04 * Float(sin(model.clock * 5))
                halo.scale = SIMD3(r * pulse, r * pulse, r)
            } else {
                halo.scale = SIMD3(repeating: r)
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

    // MARK: - Earth

    private func buildEarth(airfield: Bool) {
        var rng = SeededRandom(seed: 1903)

        // Sky dome and horizon haze.
        let skyDome = ModelEntity(mesh: .generateSphere(radius: 14000),
                                  materials: [MeshFactory.glow(UIColor(red: 0.42, green: 0.64, blue: 0.92, alpha: 1))])
        skyDome.scale = [-1, 1, 1]
        sky.addChild(skyDome)

        field.addChild(horizon)
        for (radius, height, opacity) in [(12500, 900, 0.55), (12000, 2200, 0.3), (11500, 4500, 0.15)] as [(Float, Float, Float)] {
            horizon.addChild(ModelEntity(mesh: MeshFactory.band(radius: radius, height: height),
                                         materials: [MeshFactory.glow(UIColor(red: 0.85, green: 0.9, blue: 0.97, alpha: 1), opacity: opacity)]))
        }

        // Sun and its light.
        let sunDirection = simd_normalize(SIMD3<Float>(0.4, 0.55, 0.35))
        let sun = Entity()
        sun.position = sunDirection * 12000
        sun.addChild(ModelEntity(mesh: .generateSphere(radius: 260), materials: [MeshFactory.glow(UIColor(red: 1, green: 0.98, blue: 0.9, alpha: 1))]))
        sun.addChild(ModelEntity(mesh: .generateSphere(radius: 700), materials: [MeshFactory.glow(UIColor(red: 1, green: 0.95, blue: 0.8, alpha: 1), opacity: 0.18)]))
        sky.addChild(sun)
        let light = DirectionalLight()
        light.light.intensity = 6000
        light.look(at: .zero, from: sunDirection * 100, relativeTo: nil)
        sky.addChild(light)

        // Endless plain (follows the craft horizontally).
        var grass = PhysicallyBasedMaterial()
        grass.baseColor = .init(tint: UIColor(red: 0.33, green: 0.45, blue: 0.24, alpha: 1))
        grass.roughness = 1.0
        ground.addChild(ModelEntity(mesh: .generatePlane(width: 26000, depth: 26000), materials: [grass]))
        world.addChild(ground)

        if airfield { buildRunway() }

        // Mountains ringing the valley.
        let rockTones: [UIColor] = [UIColor(red: 0.42, green: 0.4, blue: 0.36, alpha: 1), UIColor(red: 0.36, green: 0.42, blue: 0.32, alpha: 1)]
        let snow = PhysicallyBasedMaterial.make(UIColor(white: 0.95, alpha: 1), roughness: 0.6)
        for i in 0..<46 {
            let a = Float(i) / 46 * 2 * .pi + rng.next() * 0.1
            let distance = 7500 + rng.next() * 4000
            let radius = 700 + rng.next() * 1300
            let height = radius * (0.6 + rng.next() * 0.7)
            let mountain = ModelEntity(mesh: .generateSphere(radius: 1),
                                       materials: [PhysicallyBasedMaterial.make(rockTones[i % 2], roughness: 1)])
            mountain.scale = [radius, height, radius * (0.8 + rng.next() * 0.4)]
            mountain.position = [cos(a) * distance + 3000, 0, sin(a) * distance - 5000]
            if height > 1100 {
                let cap = ModelEntity(mesh: .generateSphere(radius: 1), materials: [snow])
                cap.scale = [0.42, 0.25, 0.42]
                cap.position = [0, 0.86, 0]
                mountain.addChild(cap)
            }
            world.addChild(mountain)
        }

        // Ground clutter — trees, rocks, buildings — repeating around the craft: speed cues low down.
        // (The city has its own streets and buildings instead.)
        let tree = PhysicallyBasedMaterial.make(UIColor(red: 0.16, green: 0.3, blue: 0.14, alpha: 1), roughness: 1)
        let stone = PhysicallyBasedMaterial.make(UIColor(white: 0.5, alpha: 1), roughness: 0.9)
        let roof = PhysicallyBasedMaterial.make(UIColor(red: 0.55, green: 0.3, blue: 0.22, alpha: 1), roughness: 0.8)
        let sphere = MeshResource.generateSphere(radius: 1), cube = MeshResource.generateBox(size: 1)
        for i in 0..<(airfield ? 420 : 0) {
            let e = Entity()
            switch i % 6 {
            case 0, 1, 2, 3:
                let h = 8 + rng.next() * 14
                let crown = ModelEntity(mesh: sphere, materials: [tree])
                crown.scale = [h * 0.3, h * 0.55, h * 0.3]
                crown.position = [0, h * 0.55, 0]
                e.addChild(crown)
            case 4:
                let r = 3 + rng.next() * 6
                let rock = ModelEntity(mesh: sphere, materials: [stone])
                rock.scale = [r, r * 0.6, r * 0.8]
                e.addChild(rock)
            default:
                let w = 10 + rng.next() * 20
                let house = ModelEntity(mesh: cube, materials: [stone])
                house.scale = [w, w * 0.6, w * 0.8]
                house.position = [0, w * 0.3, 0]
                let top = ModelEntity(mesh: cube, materials: [roof])
                top.scale = [w * 1.05, w * 0.15, w * 0.85]
                top.position = [0, w * 0.67, 0]
                e.addChild(house)
                e.addChild(top)
            }
            var p = Vec3(Double(rng.next()) - 0.5, 0, Double(rng.next()) - 0.5) * clutterSize
            if abs(p.x) < 60 { p.x += p.x < 0 ? -60 : 60 }  // keep the runway clear
            field.addChild(e)
            clutter.append((e, p))
        }

        // Clouds: soft clusters at 700–1600 m.
        let cloud = MeshFactory.glow(.white, opacity: 0.82)
        let shade = MeshFactory.glow(UIColor(white: 0.86, alpha: 1), opacity: 0.75)
        for _ in 0..<60 {
            let cluster = Entity()
            for k in 0..<6 {
                let puff = ModelEntity(mesh: sphere, materials: [k < 2 ? shade : cloud])
                let r = 60 + rng.next() * 110
                puff.scale = [r * 1.3, r * 0.55, r]
                puff.position = [rng.next() * 260 - 130, Float(k < 2 ? 0 : 25) + rng.next() * 30, rng.next() * 200 - 100]
                cluster.addChild(puff)
            }
            let p = Vec3((Double(rng.next()) - 0.5) * cloudSize, 700 + Double(rng.next()) * 900, (Double(rng.next()) - 0.5) * cloudSize)
            field.addChild(cluster)
            clouds.append((cluster, p))
        }
    }

    private func buildRunway() {
        let asphalt = PhysicallyBasedMaterial.make(UIColor(white: 0.2, alpha: 1), roughness: 0.9)
        let paint = PhysicallyBasedMaterial.make(UIColor(white: 0.92, alpha: 1), roughness: 0.6)
        let runway = ModelEntity(mesh: .generateBox(size: [45, 0.1, 3000]), materials: [asphalt])
        runway.position = [0, 0.02, -1300]
        world.addChild(runway)
        for i in 0..<48 {
            let dash = ModelEntity(mesh: .generateBox(size: [0.9, 0.12, 30]), materials: [paint])
            dash.position = [0, 0.03, 150 - Float(i) * 60]
            world.addChild(dash)
        }
        for end: Float in [170, -2770] {
            for k in 0..<8 {
                let bar = ModelEntity(mesh: .generateBox(size: [1.8, 0.12, 30]), materials: [paint])
                bar.position = [Float(k - 4) * 4.5 + 2.25, 0.03, end]
                world.addChild(bar)
            }
        }
        let edge = MeshFactory.glow(UIColor(red: 1, green: 0.95, blue: 0.7, alpha: 1))
        for i in 0..<50 {
            for x: Float in [-23.5, 23.5] {
                let lamp = ModelEntity(mesh: .generateSphere(radius: 0.35), materials: [edge])
                lamp.position = [x, 0.4, 200 - Float(i) * 60]
                world.addChild(lamp)
            }
        }
    }

    // MARK: - City

    /// A procedural city on a 100 m street grid: towers in every block (taller downtown), a park,
    /// and a helipad on the intersection at the origin. Gates hang over the streets.
    private func buildCity() {
        var rng = SeededRandom(seed: 2027)
        let span = 4  // blocks −4…3 on each axis → about 800 m across

        let pavement = PhysicallyBasedMaterial.make(UIColor(red: 0.42, green: 0.42, blue: 0.4, alpha: 1), roughness: 0.95)
        let asphalt = PhysicallyBasedMaterial.make(UIColor(red: 0.16, green: 0.16, blue: 0.17, alpha: 1), roughness: 0.9)
        let paint = PhysicallyBasedMaterial.make(UIColor(red: 0.9, green: 0.85, blue: 0.6, alpha: 1), roughness: 0.6)
        let extent = Float(span * 2 * 100 + 40)

        let plaza = ModelEntity(mesh: .generateBox(size: [extent, 0.04, extent]), materials: [pavement])
        plaza.position = [-50, 0.02, -50]
        world.addChild(plaza)

        // Streets every 100 m with centre-line dashes.
        let dash = MeshResource.generateBox(size: [0.4, 0.02, 4])
        for k in -span...span {
            let c = Float(k * 100)
            for vertical in [true, false] {
                let road = ModelEntity(mesh: .generateBox(size: vertical ? [20, 0.05, extent] : [extent, 0.05, 20]), materials: [asphalt])
                road.position = vertical ? [c, 0.05, -50] : [-50, 0.05, c]
                world.addChild(road)
                for d in stride(from: -extent / 2, to: extent / 2, by: 12) {
                    let m = ModelEntity(mesh: dash, materials: [paint])
                    m.position = vertical ? [c, 0.08, d - 50] : [d - 50, 0.08, c]
                    if !vertical { m.orientation = simd_quatf(angle: .pi / 2, axis: [0, 1, 0]) }
                    world.addChild(m)
                }
            }
        }

        // Façade textures: a few window grids shared by every building.
        let facades = (0..<4).compactMap { Self.facadeTexture(rows: 6 + $0 * 8, columns: 8, seed: UInt64(10 + $0)) }
        func facade(_ height: Float, tint: UIColor) -> any RealityKit.Material {
            var m = PhysicallyBasedMaterial()
            let index = min(facades.count - 1, Int(height / 35))
            if facades.indices.contains(index) {
                m.baseColor = .init(tint: tint, texture: .init(facades[index]))
                m.emissiveColor = .init(color: UIColor(red: 1, green: 0.9, blue: 0.7, alpha: 1), texture: .init(facades[index]))
                m.emissiveIntensity = 0.15
            } else {
                m.baseColor = .init(tint: tint)
            }
            m.roughness = 0.4
            m.metallic = 0.3
            return m
        }
        let tints: [UIColor] = [UIColor(red: 0.75, green: 0.8, blue: 0.86, alpha: 1), UIColor(red: 0.86, green: 0.82, blue: 0.74, alpha: 1),
                                UIColor(red: 0.6, green: 0.68, blue: 0.75, alpha: 1), UIColor(red: 0.8, green: 0.76, blue: 0.72, alpha: 1)]
        let roof = PhysicallyBasedMaterial.make(UIColor(white: 0.3, alpha: 1), roughness: 0.9)
        let unitBox = MeshResource.generateBox(size: 1)
        let tree = PhysicallyBasedMaterial.make(UIColor(red: 0.2, green: 0.38, blue: 0.18, alpha: 1), roughness: 1)
        let grass = PhysicallyBasedMaterial.make(UIColor(red: 0.3, green: 0.5, blue: 0.25, alpha: 1), roughness: 1)
        let sphere = MeshResource.generateSphere(radius: 1)

        for bx in -span..<span {
            for bz in -span..<span {
                let x0 = Float(bx * 100) + 10, z0 = Float(bz * 100) + 10  // block interior: x0…x0+80
                if bx == -1, bz == 0 {
                    // Park beside the helipad.
                    let lawn = ModelEntity(mesh: .generateBox(size: [80, 0.1, 80]), materials: [grass])
                    lawn.position = [x0 + 40, 0.08, z0 + 40]
                    world.addChild(lawn)
                    for _ in 0..<24 {
                        let t = ModelEntity(mesh: sphere, materials: [tree])
                        let h = 5 + rng.next() * 6
                        t.scale = [h * 0.45, h * 0.6, h * 0.45]
                        t.position = [x0 + 6 + rng.next() * 68, h * 0.6, z0 + 6 + rng.next() * 68]
                        world.addChild(t)
                    }
                    continue
                }
                // Taller toward downtown (around the middle of the course).
                let centre = simd_distance(SIMD2<Float>(Float(bx * 100 + 50), Float(bz * 100 + 50)), SIMD2<Float>(150, -150))
                let skyline = max(0.15, 1 - centre / 500)
                for lx in 0..<2 {
                    for lz in 0..<2 {
                        let w = 26 + rng.next() * 10, d = 26 + rng.next() * 10
                        let h = 10 + (20 + rng.next() * 110) * skyline
                        let building = ModelEntity(mesh: unitBox, materials: [facade(h, tint: tints[Int(rng.next() * 4) % 4])])
                        building.scale = [w, h, d]
                        building.position = [x0 + 20 + Float(lx) * 40, h / 2, z0 + 20 + Float(lz) * 40]
                        world.addChild(building)
                        let cap = ModelEntity(mesh: unitBox, materials: [roof])
                        cap.scale = [w * 0.6, 2.5, d * 0.6]
                        cap.position = [building.position.x, h + 1.25, building.position.z]
                        world.addChild(cap)
                    }
                }
            }
        }

        // Helipad on the intersection at the origin.
        let pad = ModelEntity(mesh: .generateBox(size: [18, 0.12, 18]), materials: [asphalt])
        pad.position = [0, 0.09, 0]
        world.addChild(pad)
        let marking = MeshFactory.glow(UIColor(red: 1, green: 0.95, blue: 0.85, alpha: 1))
        let ring = ModelEntity(mesh: MeshFactory.annulus(inner: 6.5, outer: 7.1), materials: [marking])
        ring.position = [0, 0.16, 0]
        world.addChild(ring)
        for (size, position) in [([0.9, 0.05, 6], [-1.8, 0.17, 0]), ([0.9, 0.05, 6], [1.8, 0.17, 0]), ([3.6, 0.05, 0.9], [0, 0.17, 0])] as [(SIMD3<Float>, SIMD3<Float>)] {
            let bar = ModelEntity(mesh: .generateBox(size: size), materials: [marking])
            bar.position = position
            world.addChild(bar)
        }
        let padLight = MeshFactory.glow(UIColor(red: 0.3, green: 1, blue: 0.4, alpha: 1))
        for k in 0..<12 {
            let a = Float(k) / 12 * 2 * .pi
            let lamp = ModelEntity(mesh: .generateSphere(radius: 0.18), materials: [padLight])
            lamp.position = [cos(a) * 8.6, 0.25, sin(a) * 8.6]
            world.addChild(lamp)
        }
    }

    /// A window-grid façade texture: dark glass with randomly lit panes and a light frame.
    private static func facadeTexture(rows: Int, columns: Int, seed: UInt64) -> TextureResource? {
        let cell = 16, width = columns * cell, height = rows * cell
        var rng = SeededRandom(seed: seed)
        let renderer = UIGraphicsImageRenderer(size: CGSize(width: width, height: height))
        let image = renderer.image { context in
            UIColor(white: 0.82, alpha: 1).setFill()
            context.fill(CGRect(x: 0, y: 0, width: width, height: height))
            for r in 0..<rows {
                for c in 0..<columns {
                    let lit = rng.next() < 0.18
                    (lit ? UIColor(red: 0.95, green: 0.85, blue: 0.6, alpha: 1) : UIColor(red: 0.18, green: 0.24, blue: 0.3, alpha: 1)).setFill()
                    context.fill(CGRect(x: c * cell + 2, y: r * cell + 3, width: cell - 4, height: cell - 6))
                }
            }
        }
        guard let cgImage = image.cgImage else { return nil }
        return try? TextureResource(image: cgImage, options: .init(semantic: .color))
    }

    // MARK: - Rotors

    /// Rotor arms, ducts and blades around the cockpit for multirotors (body frame = cockpit frame).
    private func buildRotors(_ craft: CraftDefinition) {
        let frame = MeshFactory.metal(0.25, roughness: 0.4)
        let duct = MeshFactory.metal(0.12, roughness: 0.5)
        var bladeMaterial = UnlitMaterial(color: UIColor(red: 0.08, green: 0.08, blue: 0.08, alpha: 1))
        bladeMaterial.blending = .transparent(opacity: 0.85)
        for (i, actuator) in craft.actuators.enumerated() where actuator.kind == .rotor {
            let p = actuator.position
            let hub = pivot + SIMD3<Float>(Float(p.x), Float(p.y), Float(p.z))
            // Arm from the cabin roof out to the hub.
            let root = SIMD3<Float>(hub.x * 0.25, hub.y - 0.15, hub.z * 0.25)
            let arm = ModelEntity(mesh: .generateBox(size: [0.12, simd_length(hub - root), 0.12], cornerRadius: 0.04), materials: [frame])
            arm.position = (hub + root) / 2
            arm.orientation = simd_quatf(from: [0, 1, 0], to: simd_normalize(hub - root))
            cockpit.addChild(arm)
            // Duct ring (torus around Z, laid flat) and motor.
            let ring = ModelEntity(mesh: MeshFactory.torus(major: 1.25, minor: 0.06, segments: 48, sides: 8), materials: [duct])
            ring.position = hub
            ring.orientation = simd_quatf(angle: .pi / 2, axis: [1, 0, 0])
            cockpit.addChild(ring)
            let motor = ModelEntity(mesh: .generateBox(size: [0.26, 0.24, 0.26], cornerRadius: 0.1), materials: [frame])
            motor.position = hub - [0, 0.08, 0]
            cockpit.addChild(motor)
            // Two-blade propeller.
            let blades = Entity()
            blades.position = hub + [0, 0.06, 0]
            for angle: Float in [0, .pi / 2] {
                let blade = ModelEntity(mesh: .generateBox(size: [2.3, 0.02, 0.16], cornerRadius: 0.01), materials: [bladeMaterial])
                blade.orientation = simd_quatf(angle: angle, axis: [0, 1, 0])
                blades.addChild(blade)
            }
            cockpit.addChild(blades)
            rotorBlades.append((blades, i))
        }
    }

    private func spinRotors(_ telemetry: FlightTelemetry, clock: Double) {
        guard !rotorBlades.isEmpty else { return }
        let dt = Float(lastSpinClock.map { clock - $0 } ?? 0)
        lastSpinClock = clock
        for (blades, index) in rotorBlades {
            let command = Float(telemetry.actuatorCommands[index])
            let spin: Float = (index % 2 == 0 ? 1 : -1) * (6 + 40 * command)  // rad/s, neighbours counter-rotate
            blades.orientation = simd_quatf(angle: spin * dt, axis: [0, 1, 0]) * blades.orientation
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
        let height: Float = switch style {
        case .fighterCanopy: 2.0
        case .droneBubble: 2.15
        case .spacecraftCapsule: 2.05
        }
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
