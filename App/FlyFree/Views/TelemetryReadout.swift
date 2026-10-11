import FlyFreeCore
import SwiftUI

struct TelemetryReadout: View {
    var telemetry: FlightTelemetry

    var body: some View {
        Grid(alignment: .leading, horizontalSpacing: 16, verticalSpacing: 6) {
            GridRow {
                Text("Stick")
                Text(telemetry.input.engaged ? "ENGAGED" : "released · holding")
                    .foregroundStyle(telemetry.input.engaged ? .green : .secondary)
                    .bold()
            }
            GridRow { Text("Command P/Y/R"); Text(triple(telemetry.input.rotation, "%+.2f")) }
            GridRow { Text("Rate °/s"); Text(triple(telemetry.state.angularVelocity * (180 / .pi), "%+.0f")) }
            GridRow {
                Text("Throttle")
                HStack {
                    Text(String(format: "%+.0f%%", telemetry.input.throttle * 100))
                    if telemetry.input.throttleLocked {
                        Image(systemName: "lock.fill").foregroundStyle(.cyan)
                    }
                }
            }
            GridRow {
                Text("Speed")
                Text(telemetry.speed < 1000
                     ? String(format: "%.1f m/s", telemetry.speed)
                     : String(format: "%.2f km/s", telemetry.speed / 1000))
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
