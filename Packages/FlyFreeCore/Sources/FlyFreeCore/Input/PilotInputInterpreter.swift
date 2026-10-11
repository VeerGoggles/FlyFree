import Foundation

/// How the non-dominant hand sets throttle.
public enum ThrottleInputMode: String, Sendable, CaseIterable, Identifiable {
    /// Flat hand as a lever: tilt fingertips up for forward, down for reverse, level for zero.
    /// Curl the hand to lock the current setting (cruise); flatten to take it back.
    case lever
    /// Finger curl: open = 0, fist = 1.
    case fingerCurl
    public var id: String { rawValue }
}

/// Per-hand open/closed flexion range used to normalize finger curl to 0...1.
public struct CurlCalibration: Sendable, Equatable {
    public var open: Double
    public var closed: Double

    public init(open: Double = 0.35, closed: Double = 3.6) {
        self.open = open
        self.closed = closed
    }

    public func normalize(_ flexion: Double) -> Double {
        guard closed > open else { return 0 }
        return ((flexion - open) / (closed - open)).clamped(to: 0...1)
    }
}

public struct InterpreterConfiguration: Sendable, Equatable {
    public var dominantHand: Chirality = .right
    public var throttleMode: ThrottleInputMode = .lever
    /// Hand-plane rotation (radians) for full deflection: pitch (fingertips up/down),
    /// yaw (wrist turned left/right), roll (forearm rotation).
    public var fullDeflection = Vec3(0.45, 0.5, 0.7)
    /// Lever-throttle pitch (radians) for full throttle.
    public var throttleFullDeflection = 0.45
    /// Curl at or below which a hand counts as flat; at or above which it counts as closed.
    public var flatCurl = 0.3
    public var closedCurl = 0.5
    /// Seconds to fade authority in after the stick engages (avoids a jolt).
    public var engageRamp = 0.25
    /// Seconds to hold the last throttle after the throttle hand loses tracking.
    public var trackingLossGrace = 0.5
    public var curlCalibration = CurlCalibration()

    public init() {}
}

/// Turns raw hand frames into a `PilotInput`.
///
/// Both hands are modelled as planes in space. Each plane's rotation relative to its
/// calibrated neutral is the command: for the dominant hand, the rotation vector maps
/// directly to pitch / yaw / roll; for the throttle hand, plane pitch is a lever.
/// Closing the dominant hand releases the stick (attitude hold); closing the throttle hand
/// locks the throttle.
public struct PilotInputInterpreter: Sendable {
    public var configuration: InterpreterConfiguration
    public var calibration: HandCalibration?

    private var engagedAt: Double?
    private var rotationFilters = [OneEuroFilter](repeating: OneEuroFilter(minCutoff: 1.2, beta: 0.4), count: 3)
    private var throttleFilter = OneEuroFilter(minCutoff: 1.0, beta: 0.3)
    private var throttle = 0.0
    private var throttleLocked = false
    private var lastThrottleSeen: Double?

    public init(configuration: InterpreterConfiguration = .init(), calibration: HandCalibration? = nil) {
        self.configuration = configuration
        self.calibration = calibration
    }

    public mutating func update(left: HandFrame?, right: HandFrame?, time: Double) -> PilotInput {
        let c = configuration
        let dominant = c.dominantHand == .left ? left : right
        let offHand = c.dominantHand == .left ? right : left
        var input = PilotInput()

        // Dominant hand plane → attitude.
        if let hand = dominant, let plane = HandMetrics.plane(of: hand) {
            input.dominantHandTracked = true
            let curl = HandMetrics.curl(hand, calibration: c.curlCalibration)
            if engagedAt == nil, curl <= c.flatCurl, calibration != nil {
                engagedAt = time
                for i in rotationFilters.indices { rotationFilters[i].reset() }
            } else if engagedAt != nil, curl >= c.closedCurl {
                engagedAt = nil
            }
            if let start = engagedAt, let neutral = calibration?[c.dominantHand] {
                let rv = HandMetrics.rotationVector(current: plane.orientation, neutral: neutral)
                let authority = c.engageRamp > 0 ? ((time - start) / c.engageRamp).clamped(to: 0...1) : 1
                var rotation = Vec3.zero
                for axis in 0..<3 {
                    let normalized = (rv[axis] / c.fullDeflection[axis]).clamped(to: -1...1)
                    rotation[axis] = rotationFilters[axis].filter(normalized, at: time) * authority
                }
                input.rotation = rotation
            }
        } else {
            engagedAt = nil
        }
        input.engaged = engagedAt != nil

        // Throttle hand.
        if let hand = offHand, let plane = HandMetrics.plane(of: hand) {
            input.throttleHandTracked = true
            lastThrottleSeen = time
            let curl = HandMetrics.curl(hand, calibration: c.curlCalibration)
            switch c.throttleMode {
            case .fingerCurl:
                throttleLocked = false
                throttle = throttleFilter.filter(curl, at: time).clamped(to: 0...1)
            case .lever:
                if curl >= c.closedCurl { throttleLocked = true }
                else if curl <= c.flatCurl, throttleLocked {
                    throttleLocked = false
                    throttleFilter.reset()
                }
                if !throttleLocked, let neutral = calibration?[c.dominantHand.opposite] {
                    let pitch = HandMetrics.relativeAngles(current: plane.orientation, neutral: neutral).x
                    throttle = throttleFilter.filter((pitch / c.throttleFullDeflection).clamped(to: -1...1), at: time)
                }
            }
        } else if let seen = lastThrottleSeen, time - seen > c.trackingLossGrace {
            throttle = 0
            throttleLocked = false
            throttleFilter.reset()
        }
        input.throttle = throttle
        input.throttleLocked = throttleLocked
        return input
    }
}
