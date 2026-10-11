import FlyFreeCore
import RealityKit
import SwiftUI

/// The immersive flight session. The pilot sits still in a cockpit while the universe moves
/// around them (visionOS cannot move the camera, so the world moves by the inverse of the
/// craft's pose). All session controls live in the scene: the hangar window is closed.
///
/// This view owns the space's lifecycle; the scene itself (`CockpitRealityView`) is rebuilt
/// whenever `sceneID` changes, so one simulation can follow another without leaving the space.
struct CockpitImmersiveView: View {
    @Environment(AppModel.self) private var model
    @Environment(\.openWindow) private var openWindow
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace

    var body: some View {
        CockpitRealityView()
            .id(model.sceneID)
            .onAppear { model.immersiveSpaceState = .open }
            .task {
                // Development aid: `-FlyFreeAutoLandAfter <seconds>` lands automatically.
                let seconds = UserDefaults.standard.double(forKey: "FlyFreeAutoLandAfter")
                guard seconds > 0 else { return }
                try? await Task.sleep(for: .seconds(seconds))
                await Landing.land(model: model, openWindow: openWindow, dismiss: dismissImmersiveSpace)
            }
            .onDisappear {
                model.handTracking.stop()
                model.immersiveSpaceState = .closed
                // Left some other way (e.g. Digital Crown): bring the hangar back.
                if !model.hangarVisible { openWindow(id: AppModel.hangarWindowID) }
            }
    }
}

/// Ending a session. The hangar is opened *before* the space is dismissed (opening a window
/// from a space that is tearing down is fragile), hand tracking stops before the scene goes
/// away, and repeated taps are ignored.
@MainActor
enum Landing {
    static func land(model: AppModel, openWindow: OpenWindowAction, dismiss: DismissImmersiveSpaceAction) async {
        guard model.immersiveSpaceState == .open else { return }
        model.immersiveSpaceState = .inTransition
        model.handTracking.stop()
        openWindow(id: AppModel.hangarWindowID)
        await dismiss()
    }
}

/// The RealityKit scene for one simulation.
private struct CockpitRealityView: View {
    @Environment(AppModel.self) private var model
    @State private var subscription: EventSubscription?

    var body: some View {
        RealityView { content, attachments in
            let scene = CockpitScene(craft: model.craft, course: model.course.course)
            content.add(scene.root)

            // Flight instruments: part of the cockpit, so they fade in with it.
            scene.mountInCockpit(attachments.entity(for: Panel.hud),
                                 position: [-0.44, 1.06, -0.6], yaw: 0.45, pitch: -0.2)
            // Buttons to the right of the centre speedometer.
            scene.mountInCockpit(attachments.entity(for: Panel.dashboard),
                                 position: [0.38, 0.92, -0.5], yaw: -0.35, pitch: -0.75)
            // Session guidance: always visible, even before the cockpit fades in.
            scene.mountInGuide(attachments.entity(for: Panel.calibration),
                               position: [0, 1.4, -0.85], yaw: 0, pitch: 0)
            scene.mountInCockpit(attachments.entity(for: Panel.complete),
                                 position: [0, 1.3, -0.85], yaw: 0, pitch: 0)
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
            Attachment(id: Panel.complete) { CourseCompletePanel().environment(model) }
        }
        .onDisappear { subscription?.cancel() }
    }

    private enum Panel {
        static let hud = "hud"
        static let dashboard = "dashboard"
        static let calibration = "calibration"
        static let simulator = "simulator"
        static let complete = "complete"
    }
}

/// Land / Reset / Recenter, mounted flat on the main console.
struct DashboardControls: View {
    @Environment(AppModel.self) private var model
    @Environment(\.openWindow) private var openWindow
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace

    var body: some View {
        HStack(spacing: 16) {
            Button("Recenter", systemImage: "hand.raised", action: model.recenter)
            Button("Reset", systemImage: "arrow.counterclockwise", action: model.resetFlight)
            Button("Land", systemImage: "airplane.arrival") {
                Task { await Landing.land(model: model, openWindow: openWindow, dismiss: dismissImmersiveSpace) }
            }
            .tint(.orange)
            .disabled(model.immersiveSpaceState != .open)
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
                Text("Score \(model.course.score) · gates \(model.course.gatesPassed)/\(model.course.course.gates.count) · \(ScoreHologram.format(model.course.elapsed(at: model.clock)))")
                    .font(.callout.monospacedDigit().bold())
                    .foregroundStyle(.cyan)
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

/// Shown when the last gate is passed: time, score, best, and Retry / Land.
struct CourseCompletePanel: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        let course = model.course
        if course.isFinished {
            VStack(spacing: 18) {
                Image(systemName: "flag.checkered")
                    .font(.system(size: 40))
                Text("Course complete").font(.largeTitle.bold())
                Text(course.course.name).font(.headline).foregroundStyle(.secondary)
                Grid(horizontalSpacing: 28, verticalSpacing: 8) {
                    GridRow {
                        stat("Time", ScoreHologram.format(course.elapsed(at: model.clock)))
                        stat("Score", "\(course.score)")
                        stat("Best", model.bestTime.map(ScoreHologram.format) ?? "—")
                    }
                }
                if model.isNewBest {
                    Label("New best time!", systemImage: "trophy.fill")
                        .font(.headline)
                        .foregroundStyle(.yellow)
                }
                if let remaining = model.autoAdvanceRemaining {
                    Text("Next: \(model.nextCraft.name) in \(Int(remaining.rounded(.up)))…")
                        .font(.headline.monospacedDigit())
                        .foregroundStyle(.secondary)
                }
                HStack(spacing: 20) {
                    Button("Next: \(model.nextCraft.name)", systemImage: "forward.fill", action: model.startNextSimulation)
                        .tint(.green)
                    Button("Retry", systemImage: "arrow.counterclockwise", action: model.resetFlight)
                }
                .labelStyle(.titleAndIcon)
                .controlSize(.large)
            }
            .padding(32)
            .frame(width: 560)
            .glassBackgroundEffect()
        }
    }

    private func stat(_ label: String, _ value: String) -> some View {
        VStack {
            Text(value).font(.title.monospacedDigit().bold())
            Text(label).font(.caption).foregroundStyle(.secondary)
        }
    }
}

/// "Hold out both hands, flat" — shown at session start and on Recenter.
struct CalibrationPanel: View {
    @Environment(AppModel.self) private var model
    @Environment(\.openWindow) private var openWindow
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace

    var body: some View {
        if case .calibrating(let initial) = model.phase {
            VStack(spacing: 16) {
                Image(systemName: "hand.raised.fingers.spread.fill")
                    .font(.system(size: 44))
                    .symbolEffect(.pulse)
                Text(initial ? "\(model.craft.name): get into position" : "Recenter: back into position")
                    .font(.title2.bold())
                Text("Match each hand to the outline floating on it, fingers straight, and hold still. This pose becomes center for the stick and the throttle.")
                    .font(.callout)
                    .multilineTextAlignment(.center)
                    .foregroundStyle(.secondary)
                HStack(alignment: .top, spacing: 24) {
                    HandStatusChip(label: "Left", role: model.dominantHand == .left ? "Stick" : "Throttle",
                                   instruction: model.targetPoses[.left]?.instruction ?? "",
                                   readiness: model.calibrationProgress.left)
                    HandStatusChip(label: "Right", role: model.dominantHand == .right ? "Stick" : "Throttle",
                                   instruction: model.targetPoses[.right]?.instruction ?? "",
                                   readiness: model.calibrationProgress.right)
                }
                ProgressView(value: model.calibrationProgress.fraction)
                    .tint(.green)
                    .frame(width: 280)
                Button(initial ? "Cancel" : "Back to cockpit") {
                    if initial {
                        Task { await Landing.land(model: model, openWindow: openWindow, dismiss: dismissImmersiveSpace) }
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
    var role: String
    var instruction: String
    var readiness: HandReadiness

    var body: some View {
        Label {
            VStack(alignment: .leading, spacing: 2) {
                Text("\(label) · \(role)").font(.headline)
                Text(instruction).font(.caption).foregroundStyle(.secondary).fixedSize(horizontal: false, vertical: true)
                Text(text).font(.caption.bold())
            }
        } icon: {
            Image(systemName: readiness == .ready ? "checkmark.circle.fill" : "circle.dashed")
                .foregroundStyle(readiness == .ready ? .green : .orange)
        }
        .frame(width: 210, alignment: .leading)
    }

    private var text: String {
        switch readiness {
        case .notDetected: "Not detected"
        case .notFlat: "Flatten your hand"
        case .wrongPose: "Match the outline"
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
                if model.craft.throttleHand == .position {
                    SimSlider(label: "Hand up", value: $model.simulatedHands.handUp, range: -15...15, unit: "cm")
                    SimSlider(label: "Hand fwd", value: $model.simulatedHands.handForward, range: -15...15, unit: "cm")
                } else {
                    SimSlider(label: "Pedal", value: $model.simulatedHands.pedal, range: -30...30)
                }
                Toggle("Throttle hand closed (stop)", isOn: $model.simulatedHands.throttleClosed)
                Button("Center") {
                    model.simulatedHands.pitch = 0
                    model.simulatedHands.yaw = 0
                    model.simulatedHands.roll = 0
                    model.simulatedHands.pedal = 0
                    model.simulatedHands.handUp = 0
                    model.simulatedHands.handForward = 0
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
    var unit = "°"

    var body: some View {
        HStack {
            Text(label).frame(width: 70, alignment: .leading)
            Slider(value: $value, in: range)
            Text("\(Int(value))\(unit)").monospacedDigit().frame(width: 52, alignment: .trailing)
        }
    }
}
