import SwiftUI

@main
struct FlyFreeApp: App {
    @State private var model = AppModel()

    var body: some Scene {
        WindowGroup(id: AppModel.hangarWindowID) {
            HangarView()
                .environment(model)
        }
        .defaultSize(width: 760, height: 820)

        ImmersiveSpace(id: AppModel.cockpitSpaceID) {
            CockpitImmersiveView()
                .environment(model)
                .onAppear { model.immersiveSpaceState = .open }
                .onDisappear { model.immersiveSpaceState = .closed }
        }
        .immersionStyle(selection: .constant(.full), in: .full)
        .upperLimbVisibility(.visible)
    }
}
