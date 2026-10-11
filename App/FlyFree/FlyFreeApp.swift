import SwiftUI

@main
struct FlyFreeApp: App {
    @State private var model = AppModel()

    init() {
        if UserDefaults.standard.object(forKey: "FlyFreeHD") == nil || UserDefaults.standard.bool(forKey: "FlyFreeHD") {
            HDScenery.setup()
        }
    }

    var body: some Scene {
        WindowGroup(id: AppModel.hangarWindowID) {
            HangarView()
                .environment(model)
        }
        .defaultSize(width: 680, height: 640)

        // Mixed immersion: the session starts over passthrough so you can see your hands while
        // calibrating, then the cockpit and an opaque universe fade in around you.
        ImmersiveSpace(id: AppModel.cockpitSpaceID) {
            CockpitImmersiveView()
                .environment(model)
        }
        .immersionStyle(selection: .constant(.mixed), in: .mixed)
        .upperLimbVisibility(.visible)
    }
}
