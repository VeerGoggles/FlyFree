import FlyFreeCore
import SwiftUI

/// The launch window: pick a craft, set handedness and control options, then fly.
struct HangarView: View {
    @Environment(AppModel.self) private var model
    @Environment(\.openImmersiveSpace) private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace

    var body: some View {
        @Bindable var model = model
        NavigationStack {
            Form {
                Section("Craft") {
                    ForEach(CraftCatalog.all) { craft in
                        Button {
                            model.craft = craft
                        } label: {
                            HStack {
                                VStack(alignment: .leading) {
                                    Text(craft.name).font(.headline)
                                    Text(craft.summary).font(.caption).foregroundStyle(.secondary)
                                }
                                Spacer()
                                if craft.id == model.craft.id { Image(systemName: "checkmark") }
                            }
                        }
                    }
                    ForEach(["Atmospheric Jet", "Lunar Lander", "Quadcopter"], id: \.self) { name in
                        Label("\(name) — coming soon", systemImage: "lock").foregroundStyle(.tertiary)
                    }
                }

                Section {
                    Picker("Dominant hand (stick)", selection: $model.dominantHand) {
                        Text("Right").tag(FlyFreeCore.Chirality.right)
                        Text("Left").tag(FlyFreeCore.Chirality.left)
                    }
                    Picker("Stick style", selection: $model.attitudeMode) {
                        Text("Fist (whole hand)").tag(AttitudeInputMode.palm)
                        Text("Finger pointer").tag(AttitudeInputMode.pointer)
                    }
                    Picker("Flight assist", selection: $model.flightAssist) {
                        Text("Velocity hold").tag(FlightAssistMode.velocityHold)
                        Text("Off (Newtonian)").tag(FlightAssistMode.off)
                    }
                    Toggle("Invert pitch", isOn: $model.invertPitch)
                    Picker("Input", selection: $model.inputSource) {
                        ForEach(AppModel.InputSource.allCases) { Text($0.rawValue).tag($0) }
                    }
                } header: {
                    Text("Controls")
                } footer: {
                    Text("Grip with your dominant hand to take the stick — that pose becomes center. Tilt to pitch, turn to yaw, rotate to roll. Open your hand to let go and hold attitude. Curl the fingers of your other hand to throttle up.")
                }

                if model.inputSource == .simulated {
                    SimulatedHandsControls()
                }

                if let telemetry = model.telemetry {
                    Section("Telemetry") { TelemetryReadout(telemetry: telemetry) }
                }
            }
            .navigationTitle("FlyFree")
            .toolbar {
                ToolbarItem(placement: .bottomOrnament) { launchButton.labelStyle(.titleAndIcon) }
            }
        }
    }

    @ViewBuilder private var launchButton: some View {
        switch model.immersiveSpaceState {
        case .closed:
            Button("Launch", systemImage: "airplane.departure") {
                Task {
                    model.immersiveSpaceState = .inTransition
                    model.resetFlight()
                    if model.inputSource == .handTracking { await model.handTracking.start() }
                    if case .opened = await openImmersiveSpace(id: AppModel.cockpitSpaceID) {
                        return
                    }
                    model.immersiveSpaceState = .closed
                }
            }
        case .inTransition:
            ProgressView()
        case .open:
            HStack {
                Button("Reset", systemImage: "arrow.counterclockwise") { model.resetFlight() }
                Button("Land", systemImage: "airplane.arrival") {
                    Task {
                        model.immersiveSpaceState = .inTransition
                        await dismissImmersiveSpace()
                        model.handTracking.stop()
                    }
                }
            }
        }
    }
}

private struct SimulatedHandsControls: View {
    @Environment(AppModel.self) private var model

    var body: some View {
        @Bindable var model = model
        Section {
            Toggle("Grip stick", isOn: $model.simulatedHands.grip)
            LabeledSlider(label: "Pitch", value: $model.simulatedHands.pitch, range: -45...45)
            LabeledSlider(label: "Yaw", value: $model.simulatedHands.yaw, range: -45...45)
            LabeledSlider(label: "Roll", value: $model.simulatedHands.roll, range: -45...45)
            LabeledSlider(label: "Throttle curl", value: $model.simulatedHands.throttleCurl, range: 0...1)
            Button("Center stick") {
                model.simulatedHands.pitch = 0
                model.simulatedHands.yaw = 0
                model.simulatedHands.roll = 0
            }
        } header: {
            Text("Simulated hands")
        } footer: {
            Text("Synthetic hand skeletons fed through the same pipeline as real hand tracking. Grip first, then tilt.")
        }
    }
}

private struct LabeledSlider: View {
    var label: String
    @Binding var value: Double
    var range: ClosedRange<Double>

    var body: some View {
        HStack {
            Text(label).frame(width: 130, alignment: .leading)
            Slider(value: $value, in: range)
            Text(value, format: .number.precision(.fractionLength(range.upperBound > 1 ? 0 : 2)))
                .monospacedDigit()
                .frame(width: 50, alignment: .trailing)
        }
    }
}
