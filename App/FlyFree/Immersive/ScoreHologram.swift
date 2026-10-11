import FlyFreeCore
import RealityKit
import UIKit

/// Holographic score floating above the dashboard: score, gate progress, streak and a flash
/// of the points earned on each gate pass.
@MainActor
final class ScoreHologram {
    let entity = Entity()
    private let scoreText = ModelEntity()
    private let detailText = ModelEntity()
    private let popupText = ModelEntity()
    private let timeText = ModelEntity()
    private var shownTenths: Int?
    private var shown: (score: Int, passed: Int, streak: Int)?
    private var shownEvent: Double?

    init() {
        let frame = MeshFactory.glow(.holoCyan, opacity: 0.5)
        for y: Float in [0.052, -0.088] {
            let line = ModelEntity(mesh: .generateBox(size: [0.34, 0.0015, 0.0015]), materials: [frame])
            line.position = [0, y, 0]
            entity.addChild(line)
        }
        for e in [scoreText, detailText, popupText, timeText] { entity.addChild(e) }
        timeText.position = [0, -0.07, 0]
        scoreText.position = [0, 0.004, 0]
        detailText.position = [0, -0.038, 0]
        popupText.position = [0, 0.07, 0]
    }

    func update(course: CourseTracker, lastEvent: (event: CourseTracker.Event, time: Double)?, now: Double) {
        // Course clock, refreshed every tenth of a second.
        let tenths = Int(course.elapsed(at: now) * 10)
        if tenths != shownTenths {
            shownTenths = tenths
            set(timeText, ScoreHologram.format(Double(tenths) / 10), size: 0.022,
                color: course.isFinished ? .holoGreen : .white)
        }

        let state = (course.score, course.gatesPassed, course.streak)
        if shown == nil || shown! != state {
            shown = state
            set(scoreText, String(format: "%06d", course.score), size: 0.045, color: .holoCyan)
            let total = course.course.gates.count
            var detail = "GATES \(course.gatesPassed)/\(total)"
            if course.streak > 1 { detail += "   STREAK ×\(course.streak)" }
            if course.isFinished { detail = "COURSE COMPLETE" }
            set(detailText, detail, size: 0.017, color: .holoCyan)
        }

        if let (event, time) = lastEvent {
            if shownEvent != time {
                shownEvent = time
                set(popupText, "+\(event.points)", size: 0.03, color: event.inOrder ? .holoGreen : .holoAmber)
            }
            // Pop, rise and fade over 1.2 s; the score swells briefly.
            let age = Float(now - time)
            popupText.isEnabled = age < 1.2
            popupText.position.y = 0.07 + age * 0.03
            popupText.components.set(OpacityComponent(opacity: max(0, 1 - age / 1.2)))
            let swell = 1 + 0.18 * max(0, 1 - age / 0.4)
            scoreText.scale = SIMD3(repeating: swell)
        } else {
            popupText.isEnabled = false
        }
    }

    static func format(_ seconds: Double) -> String {
        String(format: "%d:%04.1f", Int(seconds) / 60, seconds.truncatingRemainder(dividingBy: 60))
    }

    private func set(_ entity: ModelEntity, _ text: String, size: CGFloat, color: UIColor) {
        let mesh = MeshResource.generateText(text, extrusionDepth: 0.001,
                                             font: .monospacedSystemFont(ofSize: size, weight: .semibold))
        // Centre the text on its anchor.
        let b = mesh.bounds
        entity.children.forEach { $0.removeFromParent() }
        let child = ModelEntity(mesh: mesh, materials: [MeshFactory.glow(color, opacity: 0.92)])
        child.position = [-b.center.x, -b.center.y, 0]
        entity.addChild(child)
    }
}
