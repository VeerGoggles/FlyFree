import Foundation

/// How the dominant hand's pose becomes an attitude command.
public enum AttitudeInputMode: String, Sendable, CaseIterable, Identifiable {
    /// Make a fist around a virtual stick; whole-hand pitch/yaw/roll drives the craft.
    case palm
    /// "Finger gun": grip with middle/ring/little, the extended index finger points
    /// pitch/yaw, and palm roll drives roll.
    case pointer
    public var id: String { rawValue }
}

/// Where the zero-deflection ("stick centered") hand orientation comes from.
public enum NeutralPolicy: Sendable, Equatable {
    /// Re-center every time the pilot grips. No calibration required, never drifts.
    case captureOnEngage
    /// A fixed neutral orientation in world space (e.g. from a calibration step).
    case fixed(Quat)
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
    public var attitudeMode: AttitudeInputMode = .palm
    public var neutralPolicy: NeutralPolicy = .captureOnEngage
    /// Hand rotation (radians) that maps to full deflection on each axis (pitch, yaw, roll).
    public var fullDeflection = Vec3(0.6, 0.6, 0.7)
    /// Grip curl (0...1) above which the stick engages, and below which it releases.
    public var engageThreshold = 0.55
    public var releaseThreshold = 0.35
    /// Fingers whose curl drives throttle on the non-dominant hand.
    public var throttleFingers: [Finger] = [.index, .middle, .ring, .little]
    /// Seconds to hold the last throttle after the throttle hand loses tracking.
    public var trackingLossGrace = 0.5
    public var dominantCalibration = CurlCalibration()
    public var throttleCalibration = CurlCalibration()

    public init() {}
}

/// Turns raw hand frames into a `PilotInput`. Stateful: handles engage hysteresis,
/// neutral capture, filtering and tracking-loss behaviour.
public struct PilotInputInterpreter: Sendable {
    public var configuration: InterpreterConfiguration

    private var engaged = false
    private var capturedNeutral: Quat?
    private var rotationFilters = [OneEuroFilter](repeating: OneEuroFilter(minCutoff: 1.2, beta: 0.4), count: 3)
    private var throttleFilter = OneEuroFilter(minCutoff: 0.8, beta: 0.2)
    private var lastThrottle = 0.0
    private var lastThrottleSeen: Double?

    public init(configuration: InterpreterConfiguration = .init()) {
        self.configuration = configuration
    }

    public mutating func update(left: HandFrame?, right: HandFrame?, time: Double) -> PilotInput {
        let dominant = configuration.dominantHand == .left ? left : right
        let offHand = configuration.dominantHand == .left ? right : left
        var input = PilotInput()

        // Dominant hand → attitude.
        if let hand = dominant, let orientation = HandMetrics.orientation(
            of: hand, pointer: configuration.attitudeMode == .pointer
        ) {
            input.dominantHandTracked = true
            updateEngagement(gripCurl: gripCurl(hand), orientation: orientation)
            if engaged, let neutral = neutralOrientation {
                let angles = HandMetrics.relativeAngles(current: orientation, neutral: neutral)
                let normalized = (angles / configuration.fullDeflection)
                var rotation = Vec3.zero
                for axis in 0..<3 {
                    rotation[axis] = rotationFilters[axis].filter(normalized[axis].clamped(to: -1...1), at: time)
                }
                input.rotation = rotation
            }
        } else {
            release()
        }
        input.engaged = engaged

        // Non-dominant hand → throttle.
        if let hand = offHand {
            input.throttleHandTracked = true
            let curls = configuration.throttleFingers.map {
                configuration.throttleCalibration.normalize(HandMetrics.flexion(hand, $0))
            }
            let raw = curls.isEmpty ? 0 : curls.reduce(0, +) / Double(curls.count)
            lastThrottle = throttleFilter.filter(raw, at: time).clamped(to: 0...1)
            lastThrottleSeen = time
        } else if let seen = lastThrottleSeen, time - seen > configuration.trackingLossGrace {
            lastThrottle = 0
            throttleFilter.reset()
        }
        input.throttle = lastThrottle
        return input
    }

    private var neutralOrientation: Quat? {
        switch configuration.neutralPolicy {
        case .captureOnEngage: capturedNeutral
        case .fixed(let q): q
        }
    }

    private func gripCurl(_ hand: HandFrame) -> Double {
        let fingers: [Finger] = [.middle, .ring, .little]
        let total = fingers.reduce(0) {
            $0 + configuration.dominantCalibration.normalize(HandMetrics.flexion(hand, $1))
        }
        return total / Double(fingers.count)
    }

    private mutating func updateEngagement(gripCurl: Double, orientation: Quat) {
        if !engaged, gripCurl >= configuration.engageThreshold {
            engaged = true
            capturedNeutral = orientation
            for i in rotationFilters.indices { rotationFilters[i].reset() }
        } else if engaged, gripCurl <= configuration.releaseThreshold {
            release()
        }
    }

    private mutating func release() {
        engaged = false
        capturedNeutral = nil
        for i in rotationFilters.indices { rotationFilters[i].reset() }
    }
}
