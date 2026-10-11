import FlyFreeCore
import RealityKit
import SwiftUI

/// The full immersive space: the pilot sits still in a cockpit while the universe moves
/// around them (visionOS cannot move the camera, so we move the world by the inverse
/// of the craft's pose).
struct CockpitImmersiveView: View {
    @Environment(AppModel.self) private var model
    @State private var subscription: EventSubscription?

    var body: some View {
        RealityView { content, attachments in
            let scene = CockpitScene(craft: model.craft)
            content.add(scene.root)

            if let hud = attachments.entity(for: "hud") {
                hud.position = [-0.42, 1.08, -0.62]
                hud.orientation = simd_quatf(angle: 0.4, axis: [0, 1, 0])
                scene.cockpit.addChild(hud)
            }

            subscription = content.subscribe(to: SceneEvents.Update.self) { event in
                model.tick(dt: event.deltaTime)
                if let telemetry = model.telemetry { scene.apply(telemetry) }
            }
        } attachments: {
            Attachment(id: "hud") {
                HUDView().environment(model)
            }
        }
        .onDisappear { subscription?.cancel() }
    }
}

struct HUDView: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text(model.craft.name).font(.headline)
            if let telemetry = model.telemetry {
                TelemetryReadout(telemetry: telemetry)
                if !telemetry.input.engaged {
                    Text("Make a fist with your \(model.dominantHand.rawValue) hand to take the stick.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            } else {
                Text("Starting systems…")
            }
        }
        .padding(20)
        .frame(width: 380)
        .glassBackgroundEffect()
    }
}
