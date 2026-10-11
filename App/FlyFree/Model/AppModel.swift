import FlyFreeCore
import Observation
import SwiftUI

/// App-wide state: settings chosen in the hangar, the active flight pipeline, and telemetry.
@MainActor
@Observable
final class AppModel {
    static let hangarWindowID = "hangar"
    static let cockpitSpaceID = "cockpit"

    enum ImmersiveSpaceState { case closed, inTransition, open }

    enum InputSource: String, CaseIterable, Identifiable {
        case handTracking = "Hand Tracking"
        case simulated = "Simulated Hands"
        var id: String { rawValue }
    }

    // MARK: Settings

    var craft: CraftDefinition = CraftCatalog.genericSpacecraft
    var dominantHand: FlyFreeCore.Chirality = .right
    var attitudeMode: AttitudeInputMode = .palm
    var flightAssist: FlightAssistMode = .velocityHold
    var invertPitch = false
    #if targetEnvironment(simulator)
    var inputSource: InputSource = .simulated
    #else
    var inputSource: InputSource = .handTracking
    #endif

    // MARK: Runtime

    var immersiveSpaceState = ImmersiveSpaceState.closed
    var simulatedHands = SimulatedHands()
    let handTracking = HandTrackingService()
    private(set) var pipeline: FlightPipeline
    private(set) var telemetry: FlightTelemetry?

    init() {
        pipeline = FlightPipeline(craft: CraftCatalog.genericSpacecraft)
    }

    /// Rebuilds the pipeline from current settings and puts the craft back at the origin.
    func resetFlight() {
        var config = InterpreterConfiguration()
        config.dominantHand = dominantHand
        config.attitudeMode = attitudeMode
        var craft = craft
        craft.profile.invertPitch = invertPitch
        pipeline = FlightPipeline(craft: craft, interpreter: config, assist: flightAssist)
        telemetry = nil
    }

    /// Called once per rendered frame from the immersive scene.
    func tick(dt: Double) {
        let hands: (left: HandFrame?, right: HandFrame?)
        switch inputSource {
        case .handTracking:
            hands = handTracking.latestHands()
        case .simulated:
            hands = simulatedHands.frames(dominant: dominantHand, attitudeMode: attitudeMode)
        }
        telemetry = pipeline.step(left: hands.left, right: hands.right, dt: dt)
    }
}
