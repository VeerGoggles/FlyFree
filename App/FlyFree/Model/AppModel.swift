import FlyFreeCore
import Observation
import SwiftUI

/// App-wide state: settings chosen in the hangar, the session phase, the active flight
/// pipeline, and the latest telemetry.
@MainActor
@Observable
final class AppModel {
    static let hangarWindowID = "hangar"
    static let cockpitSpaceID = "cockpit"
    static let fadeInDuration = 2.5

    enum ImmersiveSpaceState { case closed, inTransition, open }

    enum InputSource: String, CaseIterable, Identifiable {
        case handTracking = "Hand Tracking"
        case simulated = "Simulated Hands"
        var id: String { rawValue }
    }

    /// A flight session: hold hands out flat → cockpit fades in → fly. Recenter returns to
    /// calibration without fading the cockpit out.
    enum Phase: Equatable {
        case calibrating(initial: Bool)
        case fadingIn(start: Double)
        case flying
    }

    // MARK: Settings

    var craft: CraftDefinition = CraftCatalog.genericSpacecraft
    var dominantHand: FlyFreeCore.Chirality = .right
    var flightAssist: FlightAssistMode = .velocityHold
    var controlFrame: ControlFrame = .global
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
    private(set) var phase = Phase.calibrating(initial: true)
    private(set) var calibrationProgress = CalibrationProgress(left: .notDetected, right: .notDetected, fraction: 0)
    private(set) var handPlanes: [FlyFreeCore.Chirality: HandPlane] = [:]
    /// Opacity of the cockpit + universe (0 during the first calibration).
    private(set) var sceneOpacity: Float = 0
    private(set) var pipeline: FlightPipeline
    private(set) var telemetry: FlightTelemetry?
    private(set) var course = CourseTracker(course: CourseCatalog.asteroidSlalom)
    /// Most recent gate pass and when it happened (session clock), for score effects.
    private(set) var lastGateEvent: (event: CourseTracker.Event, time: Double)?
    /// Seconds since the session started.
    private(set) var clock = 0.0

    private var calibrationSession = CalibrationSession()
    private var calibration: HandCalibration?

    init() {
        pipeline = FlightPipeline(craft: CraftCatalog.genericSpacecraft)
    }

    /// Starts a new session at the hands-out calibration step.
    func beginSession() {
        calibration = nil
        calibrationSession.reset()
        simulatedHands = SimulatedHands()
        phase = .calibrating(initial: true)
        sceneOpacity = 0
        resetFlight()
    }

    /// Puts the craft back at the start, keeping the current hand calibration.
    func resetFlight() {
        var config = InterpreterConfiguration()
        config.dominantHand = dominantHand
        var craft = craft
        craft.profile.invertPitch = invertPitch
        pipeline = FlightPipeline(craft: craft, interpreter: config, calibration: calibration,
                                  assist: flightAssist, controlFrame: controlFrame)
        telemetry = nil
        course = CourseTracker(course: CourseCatalog.asteroidSlalom)
        lastGateEvent = nil
    }

    /// Re-captures the neutral flat-hand pose. The craft holds attitude meanwhile.
    func recenter() {
        calibrationSession.reset()
        phase = .calibrating(initial: false)
    }

    /// Leaves a recenter without changing the existing calibration.
    func cancelRecenter() {
        guard case .calibrating(initial: false) = phase else { return }
        phase = .flying
    }

    /// Called once per rendered frame from the immersive scene.
    func tick(dt: Double) {
        clock += dt
        let hands: (left: HandFrame?, right: HandFrame?)
        switch inputSource {
        case .handTracking: hands = handTracking.latestHands()
        case .simulated: hands = simulatedHands.frames(dominant: dominantHand)
        }
        var planes: [FlyFreeCore.Chirality: HandPlane] = [:]
        planes[.left] = hands.left.flatMap(HandMetrics.plane)
        planes[.right] = hands.right.flatMap(HandMetrics.plane)
        handPlanes = planes

        switch phase {
        case .calibrating(let initial):
            calibrationProgress = calibrationSession.update(left: hands.left, right: hands.right, time: clock)
            if let result = calibrationProgress.result {
                calibration = result
                pipeline.calibrate(result)
                phase = initial ? .fadingIn(start: clock) : .flying
            }
        case .fadingIn(let start):
            sceneOpacity = Float(min(1, (clock - start) / Self.fadeInDuration))
            if sceneOpacity >= 1 { phase = .flying }
            fly(hands, dt: dt)
        case .flying:
            sceneOpacity = 1
            // Development aid: `-FlyFreeSimPedal <degrees>` holds the simulated pedal after calibration.
            if inputSource == .simulated, simulatedHands.pedal == 0,
               let pedal = UserDefaults.standard.object(forKey: "FlyFreeSimPedal") as? Double ?? Double(UserDefaults.standard.string(forKey: "FlyFreeSimPedal") ?? "") {
                simulatedHands.pedal = pedal
            }
            fly(hands, dt: dt)
        }
    }

    private func fly(_ hands: (left: HandFrame?, right: HandFrame?), dt: Double) {
        let telemetry = pipeline.step(left: hands.left, right: hands.right, dt: dt)
        self.telemetry = telemetry
        for event in course.update(position: telemetry.state.position, time: clock) {
            lastGateEvent = (event, clock)
        }
    }
}
