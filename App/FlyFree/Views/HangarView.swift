import FlyFreeCore
import SwiftUI

/// The launch window: pick a craft and control options, then fly. It closes itself once the
/// cockpit opens and comes back when you land.
struct HangarView: View {
    @Environment(AppModel.self) private var model
    @Environment(\.openImmersiveSpace) private var openImmersiveSpace
    @Environment(\.dismissWindow) private var dismissWindow

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
                    Picker("Stick hand", selection: $model.dominantHand) {
                        Text("Right").tag(FlyFreeCore.Chirality.right)
                        Text("Left").tag(FlyFreeCore.Chirality.left)
                    }
                    Picker("Stick frame", selection: $model.controlFrame) {
                        Text("Global (matches hologram)").tag(ControlFrame.global)
                        Text("Cockpit (ship axes)").tag(ControlFrame.body)
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
                    Text("""
                    Hold both hands out flat, palms down, until they lock in — the cockpit fades in around you.
                    Stick hand: tip fingertips up/down to pitch, turn the wrist left/right to yaw, rotate the forearm to roll. Close it to return to the starting orientation.
                    Throttle hand is a pedal: push fingertips down to accelerate, level to hold speed, tip up to brake. Close it to brake to a stop.
                    """)
                }
            }
            .navigationTitle("FlyFree")
            .task {
                // Development aid: `-FlyFreeAutoLaunch` skips the hangar (e.g. for simulator runs).
                if ProcessInfo.processInfo.arguments.contains("-FlyFreeAutoLaunch"),
                   model.immersiveSpaceState == .closed {
                    launch()
                }
            }
            .toolbar {
                ToolbarItem(placement: .bottomOrnament) {
                    Button("Launch", systemImage: "airplane.departure", action: launch)
                        .labelStyle(.titleAndIcon)
                        .disabled(model.immersiveSpaceState != .closed)
                }
            }
        }
    }

    private func launch() {
        Task {
            model.immersiveSpaceState = .inTransition
            model.beginSession()
            if model.inputSource == .handTracking { await model.handTracking.start() }
            if case .opened = await openImmersiveSpace(id: AppModel.cockpitSpaceID) {
                dismissWindow(id: AppModel.hangarWindowID)
            } else {
                model.immersiveSpaceState = .closed
            }
        }
    }
}
