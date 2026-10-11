/// Everything that happened in one simulation frame — for HUDs, visuals and debugging.
public struct FlightTelemetry: Sendable {
    public var time: Double
    public var input: PilotInput
    public var desired: DesiredCraftState
    public var demand: Wrench
    public var achieved: Wrench
    /// Actuator commands (0...1), index-aligned with `craft.actuators`.
    public var actuatorCommands: [Double]
    public var state: RigidBodyState

    /// Speed relative to the starting frame (m/s).
    public var speed: Double { (state.velocity * state.velocity).sum().squareRoot() }
}

/// The full hands → craft pipeline:
///
///     HandFrame ─► HandPlane (least-squares fit through the finger joints)
///               ─► PilotInputInterpreter ─► PilotInput (relative to calibrated neutral)
///               ─► IntentMapper          ─► DesiredCraftState
///               ─► FlightController      ─► Wrench demand
///               ─► ControlAllocator      ─► actuator commands (+ achieved wrench)
///               ─► RigidBodyIntegrator   ─► RigidBodyState
///
/// Input and intent run once per rendered frame; control, allocation and physics run at a
/// fixed sub-step so behaviour does not depend on frame rate.
public struct FlightPipeline: Sendable {
    public let craft: CraftDefinition
    public var interpreter: PilotInputInterpreter
    public var mapper: IntentMapper
    public var controller: FlightController
    public var allocator: ControlAllocator
    public var environment: any FlightEnvironment
    public var state: RigidBodyState
    public var physicsStep = 1.0 / 180.0

    private var time = 0.0
    private var accumulator = 0.0

    public init(
        craft: CraftDefinition,
        interpreter: InterpreterConfiguration = .init(),
        calibration: HandCalibration? = nil,
        assist: FlightAssistMode = .velocityHold,
        environment: any FlightEnvironment = DeepSpace(),
        initialState: RigidBodyState = .init()
    ) {
        self.craft = craft
        self.interpreter = PilotInputInterpreter(configuration: interpreter, calibration: calibration)
        self.mapper = IntentMapper(profile: craft.profile, assist: assist)
        self.controller = FlightController(gains: craft.gains, mass: craft.mass,
                                           maxForwardThrust: craft.maxForwardThrust,
                                           maxHoldRate: craft.profile.maxAngularRate * 0.25)
        self.allocator = ControlAllocator(actuators: craft.actuators, weights: craft.allocationWeights)
        self.environment = environment
        self.state = initialState
    }

    /// Sets the neutral hand frames that all stick and lever input is measured against.
    public mutating func calibrate(_ calibration: HandCalibration) {
        interpreter.calibration = calibration
    }

    /// Advances the simulation by a rendered frame of length `dt` seconds.
    public mutating func step(left: HandFrame?, right: HandFrame?, dt: Double) -> FlightTelemetry {
        let frameDt = dt.clamped(to: 0...0.1)
        let input = interpreter.update(left: left, right: right, time: time + frameDt)
        return step(input: input, dt: frameDt)
    }

    /// Advances the simulation from an already-interpreted `PilotInput` (useful for
    /// alternative input devices, replays and tests).
    public mutating func step(input: PilotInput, dt: Double) -> FlightTelemetry {
        let frameDt = dt.clamped(to: 0...0.1)
        time += frameDt
        accumulator += frameDt
        let desired = mapper.map(input, state: state)

        var demand = Wrench.zero
        var achieved = allocator.achievedWrench
        while accumulator >= physicsStep {
            demand = controller.demand(desired: desired, state: state)
            achieved = allocator.allocate(demand)
            let loads = environment.loads(on: state, mass: craft.mass)
            RigidBodyIntegrator.step(&state, mass: craft.mass, bodyWrench: achieved, external: loads, dt: physicsStep)
            accumulator -= physicsStep
        }

        return FlightTelemetry(
            time: time, input: input, desired: desired, demand: demand, achieved: achieved,
            actuatorCommands: allocator.currentCommands, state: state
        )
    }
}
