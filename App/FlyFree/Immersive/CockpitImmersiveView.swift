import FlyFreeCore
import RealityKit
import SwiftUI

/// The immersive flight session. The pilot sits still in a cockpit while the universe moves
/// around them (visionOS cannot move the camera, so the world moves by the inverse of the
/// craft's pose). All session controls live in the scene: the hangar window is closed.
struct CockpitImmersiveView: View {
    @Environment(AppModel.self) private var model
    @Environment(\.openWindow) private var openWindow
    @State private var subscription: EventSubscription?

    var body: some View {
        RealityView { content, attachments in
            let scene = CockpitScene(craft: model.craft)
            content.add(scene.root)

            // Flight instruments: part of the cockpit, so they fade in with it.
            scene.mountInCockpit(attachments.entity(for: Panel.hud),
                                 position: [-0.44, 1.06, -0.6], yaw: 0.45, pitch: -0.2)
            scene.mountInCockpit(attachments.entity(for: Panel.dashboard),
                                 position: [0, 0.9, -0.52], yaw: 0, pitch: -0.75)
            // Session guidance: always visible, even before the cockpit fades in.
            scene.mountInGuide(attachments.entity(for: Panel.calibration),
                               position: [0, 1.4, -0.85], yaw: 0, pitch: 0)
            scene.mountInGuide(attachments.entity(for: Panel.simulator),
                               position: [0.62, 1.05, -0.4], yaw: -0.8, pitch: -0.25)

            subscription = content.subscribe(to: SceneEvents.Update.self) { event in
                model.tick(dt: event.deltaTime)
                scene.apply(model)
            }
        } attachments: {
            Attachment(id: Panel.hud) { HUDView().environment(model) }
            Attachment(id: Panel.dashboard) { DashboardControls().environment(model) }
            Attachment(id: Panel.calibration) { CalibrationPanel().environment(model) }
            Attachment(id: Panel.simulator) { SimulatedHandsPanel().environment(model) }
        }
        .onAppear { model.immersiveSpaceState = .open }
        .onDisappear {
            // However the session ends (Land, Digital Crown, …), bring the hangar back.
            subscription?.cancel()
            model.handTracking.stop()
            model.immersiveSpaceState = .closed
            openWindow(id: AppModel.hangarWindowID)
        }
    }

    private enum Panel {
        static let hud = "hud"
        static let dashboard = "dashboard"
        static let calibration = "calibration"
        static let simulator = "simulator"
    }
}

/// Land / Reset / Recenter, mounted flat on the main console.
struct DashboardControls: View {
    @Environment(AppModel.self) private var model
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace

    var body: some View {
        HStack(spacing: 16) {
            Button("Recenter", systemImage: "hand.raised", action: model.recenter)
            Button("Reset", systemImage: "arrow.counterclockwise", action: model.resetFlight)
            Button("Land", systemImage: "airplane.arrival") {
                Task {
                    model.immersiveSpaceState = .inTransition
                    await dismissImmersiveSpace()
                }
            }
            .tint(.orange)
        }
        .labelStyle(.titleAndIcon)
        .padding(14)
        .glassBackgroundEffect()
        .opacity(model.phase == .calibrating(initial: true) ? 0 : 1)
    }
}

struct HUDView: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text(model.craft.name).font(.headline)
            if let telemetry = model.telemetry {
                TelemetryReadout(telemetry: telemetry)
                Text("Close stick hand: reset orientation · Close throttle hand: stop")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            } else {
                Text("Starting systems…")
            }
        }
        .padding(20)
        .frame(width: 380)
        .glassBackgroundEffect()
    }
}

/// "Hold out both hands, flat" — shown at session start and on Recenter.
struct CalibrationPanel: View {
    @Environment(AppModel.self) private var model
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace

    var body: some View {
        if case .calibrating(let initial) = model.phase {
            VStack(spacing: 16) {
                Image(systemName: "hand.raised.fingers.spread.fill")
                    .font(.system(size: 44))
                    .symbolEffect(.pulse)
                Text(initial ? "Hold out both hands, flat" : "Recenter: hands out flat")
                    .font(.title2.bold())
                Text("Palms down, fingers together and pointing forward. Keep them still in a comfortable position — this becomes center for the stick and the throttle.")
                    .font(.callout)
                    .multilineTextAlignment(.center)
                    .foregroundStyle(.secondary)
                HStack(spacing: 24) {
                    HandStatusChip(label: "Left", readiness: model.calibrationProgress.left)
                    HandStatusChip(label: "Right", readiness: model.calibrationProgress.right)
                }
                ProgressView(value: model.calibrationProgress.fraction)
                    .tint(.green)
                    .frame(width: 280)
                Button(initial ? "Cancel" : "Back to cockpit") {
                    if initial {
                        Task { await dismissImmersiveSpace() }
                    } else {
                        model.cancelRecenter()
                    }
                }
            }
            .padding(28)
            .frame(width: 520)
            .glassBackgroundEffect()
        }
    }
}

private struct HandStatusChip: View {
    var label: String
    var readiness: HandReadiness

    var body: some View {
        Label {
            VStack(alignment: .leading) {
                Text(label).font(.headline)
                Text(text).font(.caption)
            }
        } icon: {
            Image(systemName: readiness == .ready ? "checkmark.circle.fill" : "circle.dashed")
                .foregroundStyle(readiness == .ready ? .green : .orange)
        }
        .frame(width: 170, alignment: .leading)
    }

    private var text: String {
        switch readiness {
        case .notDetected: "Not detected"
        case .notFlat: "Flatten your hand"
        case .notLevel: "Turn palm down"
        case .moving: "Hold still"
        case .ready: "Locked"
        }
    }
}

/// On-device stand-in for the hands, shown only with the Simulated Hands input source.
struct SimulatedHandsPanel: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        @Bindable var model = model
        if model.inputSource == .simulated {
            VStack(alignment: .leading, spacing: 8) {
                Text("Simulated hands").font(.headline)
                Toggle("Hands present", isOn: $model.simulatedHands.present)
                Toggle("Stick hand closed (reset)", isOn: $model.simulatedHands.stickClosed)
                SimSlider(label: "Pitch", value: $model.simulatedHands.pitch, range: -30...30)
                SimSlider(label: "Yaw", value: $model.simulatedHands.yaw, range: -30...30)
                SimSlider(label: "Roll", value: $model.simulatedHands.roll, range: -40...40)
                SimSlider(label: "Pedal", value: $model.simulatedHands.pedal, range: -30...30)
                Toggle("Throttle hand closed (stop)", isOn: $model.simulatedHands.throttleClosed)
                Button("Center") {
                    model.simulatedHands.pitch = 0
                    model.simulatedHands.yaw = 0
                    model.simulatedHands.roll = 0
                    model.simulatedHands.pedal = 0
                }
            }
            .padding(20)
            .frame(width: 360)
            .glassBackgroundEffect()
        }
    }
}

private struct SimSlider: View {
    var label: String
    @Binding var value: Double
    var range: ClosedRange<Double>

    var body: some View {
        HStack {
            Text(label).frame(width: 70, alignment: .leading)
            Slider(value: $value, in: range)
            Text("\(Int(value))°").monospacedDigit().frame(width: 44, alignment: .trailing)
        }
    }
}
