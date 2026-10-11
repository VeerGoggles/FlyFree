import FlyFreeCore
import SwiftUI

struct TelemetryReadout: View {
    var telemetry: FlightTelemetry

    var body: some View {
        Grid(alignment: .leading, horizontalSpacing: 16, verticalSpacing: 6) {
            GridRow {
                Text("Stick")
                Text(telemetry.input.stickClosed ? "RESETTING" : telemetry.input.engaged ? "ENGAGED" : "released · holding")
                    .foregroundStyle(telemetry.input.stickClosed ? .purple : telemetry.input.engaged ? .green : .secondary)
                    .bold()
            }
            GridRow { Text("Command P/Y/R"); Text(triple(telemetry.input.rotation, "%+.2f")) }
            GridRow { Text("Rate °/s"); Text(triple(telemetry.state.angularVelocity * (180 / .pi), "%+.0f")) }
            GridRow {
                Text("Throttle")
                HStack {
                    if telemetry.input.throttleClosed {
                        Text("BRAKING").bold().foregroundStyle(.red)
                    } else {
                        Text(String(format: "%+.0f%%", telemetry.input.throttle * 100))
                    }
                }
            }
            GridRow {
                Text("Speed")
                Text(telemetry.speed < 1000
                     ? String(format: "%.1f m/s", telemetry.speed)
                     : String(format: "%.2f km/s", telemetry.speed / 1000))
            }
            if let air = telemetry.air {
                GridRow { Text("Airspeed"); Text(String(format: "%.0f m/s  (%.0f kt)", air.airspeed, air.airspeed * 1.944)) }
                GridRow { Text("Altitude"); Text(String(format: "%.0f m", air.altitude)) }
                GridRow {
                    Text("AoA / G")
                    HStack {
                        Text(String(format: "%.1f°  %.1f g", air.alpha * 180 / .pi, telemetry.loadFactor))
                        if air.alpha > 0.2 { Text("AoA LIMIT").bold().foregroundStyle(.orange) }
                        if telemetry.onGround { Text("ON GROUND").bold().foregroundStyle(.green) }
                    }
                }
            }
            GridRow {
                Text("Hands")
                Text("stick \(telemetry.input.dominantHandTracked ? "✓" : "✗")   throttle \(telemetry.input.throttleHandTracked ? "✓" : "✗")")
            }
        }
        .font(.callout.monospacedDigit())
    }

    private func triple(_ v: Vec3, _ format: String) -> String {
        [v.x, v.y, v.z].map { String(format: format, $0) }.joined(separator: "  ")
    }
}
