import Foundation
import simd

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
    /// Hand-plane rotation (radians) for full deflection: pitch (fingertips up/down),
    /// yaw (wrist turned left/right), roll (forearm rotation). ±90° on every axis; the
    /// profile's exponential curve gives fine control near center.
    public var fullDeflection = Vec3(repeating: .pi / 2)
    /// Throttle-hand fingertip tilt (radians) for full pedal.
    public var throttleFullDeflection = 0.45
    /// Curl at or below which a hand counts as flat; at or above which it counts as closed.
    public var flatCurl = 0.3
    public var closedCurl = 0.5
    /// Seconds to fade authority in after the stick engages (avoids a jolt).
    public var engageRamp = 0.0625
    /// Smoothing of the stick and throttle signals (1€ filter); every cutoff is 4× the original
    /// tuning, so transitions settle ~4× faster. The resting time constant is
    /// 1 / (2π · minCutoff): ~33 ms for the stick, ~40 ms for the throttle.
    public var rotationSmoothing = OneEuroFilter(minCutoff: 4.8, beta: 1.6, derivativeCutoff: 4)
    public var throttleSmoothing = OneEuroFilter(minCutoff: 4.0, beta: 1.2, derivativeCutoff: 4)
    /// Seconds to hold the last throttle after the throttle hand loses tracking.
    public var trackingLossGrace = 0.5
    public var curlCalibration = CurlCalibration()

    public init() {}
}

/// Turns raw hand frames into a `PilotInput`.
///
/// Both hands are modelled as planes in space, measured relative to their calibrated neutral.
/// - **Stick hand, flat:** the plane's rotation vector → pitch / yaw / roll.
/// - **Stick hand, closed:** reset orientation.
/// - **Throttle hand, flat:** a pedal — fingertips down accelerates, up brakes / reverses.
/// - **Throttle hand, closed:** brake to a stop.
public struct PilotInputInterpreter: Sendable {
    public var configuration: InterpreterConfiguration
    public var calibration: HandCalibration?

    private var engagedAt: Double?
    private var stickClosed = false
    private var rotationFilters: [OneEuroFilter]
    private var throttleFilter: OneEuroFilter
    private var throttle = 0.0
    private var throttleClosed = false
    private var lastThrottleSeen: Double?

    public init(configuration: InterpreterConfiguration = .init(), calibration: HandCalibration? = nil) {
        self.configuration = configuration
        self.calibration = calibration
        self.rotationFilters = Array(repeating: configuration.rotationSmoothing, count: 3)
        self.throttleFilter = configuration.throttleSmoothing
    }

    public mutating func update(left: HandFrame?, right: HandFrame?, time: Double) -> PilotInput {
        let c = configuration
        let dominant = c.dominantHand == .left ? left : right
        let offHand = c.dominantHand == .left ? right : left
        var input = PilotInput()

        // Stick hand.
        if let hand = dominant, let plane = HandMetrics.plane(of: hand), let neutral = calibration?[c.dominantHand] {
            input.dominantHandTracked = true
            let curl = HandMetrics.curl(hand, calibration: c.curlCalibration)
            if curl >= c.closedCurl {
                engagedAt = nil
                stickClosed = true
            } else if curl <= c.flatCurl {
                stickClosed = false
                if engagedAt == nil {
                    engagedAt = time
                    for i in rotationFilters.indices { rotationFilters[i].reset() }
                }
            }
            if let start = engagedAt {
                let rv = HandMetrics.pilotRotationVector(current: plane.orientation, neutral: neutral)
                let authority = c.engageRamp > 0 ? ((time - start) / c.engageRamp).clamped(to: 0...1) : 1
                var rotation = Vec3.zero
                for axis in 0..<3 {
                    let normalized = (rv[axis] / c.fullDeflection[axis]).clamped(to: -1...1)
                    rotation[axis] = rotationFilters[axis].filter(normalized, at: time) * authority
                }
                input.rotation = rotation
            }
        } else {
            input.dominantHandTracked = dominant != nil
            engagedAt = nil
            stickClosed = false
        }
        input.engaged = engagedAt != nil
        input.stickClosed = stickClosed

        // Throttle hand.
        if let hand = offHand, let plane = HandMetrics.plane(of: hand), let neutral = calibration?[c.dominantHand.opposite] {
            input.throttleHandTracked = true
            lastThrottleSeen = time
            let curl = HandMetrics.curl(hand, calibration: c.curlCalibration)
            if curl >= c.closedCurl { throttleClosed = true }
            else if curl <= c.flatCurl { throttleClosed = false }

            if throttleClosed {
                throttle = 0
                throttleFilter.reset()
            } else {
                // Fingertips down (negative pitch) = accelerate, like pressing a pedal.
                // Fingertips down/up in the pilot's frame, whatever pose the hand started in.
                let inPilotFrame: Quat = (plane.orientation * neutral.inverse).normalized
                let pitch = HandMetrics.twistAngle(current: inPilotFrame, neutral: Quat(ix: 0, iy: 0, iz: 0, r: 1),
                                                   about: BodyAxis.right)
                throttle = throttleFilter.filter((-pitch / c.throttleFullDeflection).clamped(to: -1...1), at: time)
            }
        } else {
            input.throttleHandTracked = offHand != nil
            if let seen = lastThrottleSeen, time - seen > c.trackingLossGrace {
                throttle = 0
                throttleClosed = false
                throttleFilter.reset()
            }
        }
        input.throttle = throttle
        input.throttleClosed = throttleClosed
        return input
    }
}
