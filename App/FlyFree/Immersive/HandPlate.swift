import FlyFreeCore
import RealityKit
import UIKit

/// The holographic "stick" plate floating on a hand's fitted plane: a glowing rim with a soft
/// halo, a faint fill, compass ticks and a forward chevron. Colour shows the hand's state.
@MainActor
final class HandPlate {
    enum Style: Equatable {
        case waiting, ready, engaged, released, resetting, braking

        var color: UIColor {
            switch self {
            case .waiting: .holoAmber
            case .ready, .engaged: .holoCyan
            case .released: UIColor(white: 0.65, alpha: 1)
            case .resetting: .holoViolet
            case .braking: .holoRed
            }
        }
    }

    let entity = Entity()
    private let rim: ModelEntity
    private let halo: ModelEntity
    private let fill: ModelEntity
    private let marks: [ModelEntity]
    private var style: Style?
    private var smoothed: (position: SIMD3<Float>, orientation: simd_quatf)?

    init() {
        let radius: Float = 0.058
        rim = ModelEntity(mesh: MeshFactory.annulus(inner: radius - 0.004, outer: radius))
        halo = ModelEntity(mesh: MeshFactory.annulus(inner: radius, outer: radius + 0.014))
        fill = ModelEntity(mesh: MeshFactory.disc(radius: radius - 0.004))
        var marks: [ModelEntity] = []
        let chevron = ModelEntity(mesh: MeshFactory.chevron(width: 0.026, length: 0.022))
        chevron.position = [0, 0.001, -radius * 0.45]
        marks.append(chevron)
        for i in 0..<12 {
            let a = Float(i) / 12 * 2 * .pi
            let long = i % 3 == 0
            let tick = ModelEntity(mesh: .generateBox(size: [0.0015, 0.0005, long ? 0.01 : 0.005]))
            tick.position = SIMD3(sin(a), 0, -cos(a)) * (radius - (long ? 0.011 : 0.008))
            tick.orientation = simd_quatf(angle: -a, axis: [0, 1, 0])
            marks.append(tick)
        }
        self.marks = marks
        for e in [rim, halo, fill] + marks { entity.addChild(e) }
        entity.isEnabled = false
    }

    func update(plane: HandPlane?, style: Style, time: Double) {
        guard let plane else {
            entity.isEnabled = false
            smoothed = nil
            return
        }
        entity.isEnabled = true
        let c = plane.centroid + plane.normal * 0.022
        let q = plane.orientation
        let target = (SIMD3<Float>(Float(c.x), Float(c.y), Float(c.z)),
                      simd_quatf(ix: Float(q.imag.x), iy: Float(q.imag.y), iz: Float(q.imag.z), r: Float(q.real)))
        // Light smoothing so the plate glides rather than jitters.
        if let s = smoothed {
            smoothed = (simd_mix(s.position, target.0, SIMD3(repeating: 0.5)), simd_slerp(s.orientation, target.1, 0.5))
        } else {
            smoothed = target
        }
        entity.position = smoothed!.position
        entity.orientation = smoothed!.orientation

        // Gentle breathing pulse on the halo.
        let pulse = 1 + 0.06 * Float(sin(time * 4))
        halo.scale = [pulse, 1, pulse]

        guard self.style != style else { return }
        self.style = style
        let color = style.color
        rim.model?.materials = [MeshFactory.glow(color, opacity: 0.95)]
        halo.model?.materials = [MeshFactory.glow(color, opacity: 0.22)]
        fill.model?.materials = [MeshFactory.glow(color, opacity: 0.08)]
        for m in marks { m.model?.materials = [MeshFactory.glow(color, opacity: 0.8)] }
    }
}
