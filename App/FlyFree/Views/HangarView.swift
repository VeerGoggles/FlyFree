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
                    ForEach(["Lunar Lander"], id: \.self) { name in
                        Label("\(name) — coming soon", systemImage: "lock").foregroundStyle(.tertiary)
                    }
                }

                Section {
                    Picker("Stick hand", selection: $model.dominantHand) {
                        Text("Right").tag(FlyFreeCore.Chirality.right)
                        Text("Left").tag(FlyFreeCore.Chirality.left)
                    }
                    if model.craft.world == .deepSpace {
                        Picker("Flight assist", selection: $model.flightAssist) {
                            Text("Velocity hold").tag(FlightAssistMode.velocityHold)
                            Text("Off (Newtonian)").tag(FlightAssistMode.off)
                        }
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
                    Drone: tilt your hand to fly that way, twist to turn, pedal to climb or descend; let go to hover.
                    Jet: the pedal sets the autothrottle's airspeed. Let go of the stick to hold your climb and bank; close it for wings-level, level flight. Rotate at about 90 m/s to take off.
                    """)
                }
            }
            .navigationTitle("FlyFree")
            .onAppear { model.hangarVisible = true }
            .onDisappear { model.hangarVisible = false }
            .task {
                // Development aid: `-FlyFreeAutoLaunch` skips the hangar (e.g. for simulator runs).
                // `-FlyFreeCraft <id>` preselects a craft.
                if let id = UserDefaults.standard.string(forKey: "FlyFreeCraft"),
                   let craft = CraftCatalog.all.first(where: { $0.id == id }) {
                    model.craft = craft
                }
                if ProcessInfo.processInfo.arguments.contains("-FlyFreeAutoLaunch"),
                   !model.autoLaunched, model.immersiveSpaceState == .closed {
                    model.autoLaunched = true
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
