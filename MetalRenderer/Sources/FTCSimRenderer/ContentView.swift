import SwiftUI

struct ContentView: View {

    @StateObject private var client = SimClient()

    // Robot config state
    @State private var width     = "18.0"
    @State private var length    = "18.0"
    @State private var chassisMotor = "GOBILDA_5203_19_2"
    @State private var launcherMotor = "GOBILDA_5203_30_1"
    @State private var hasVision  = false
    @State private var intakeType = "NONE"
    @State private var scoringType = "NONE"
    @State private var programText = "MOVE_TO(24, 24)\nINTAKE\nMOVE_TO(-48, -12)\nLAUNCH"
    @State private var isRunning  = false
    @State private var brightness: Float = 1.0

    private let motorOptions = ["GOBILDA_5203_19_2", "GOBILDA_5203_30_1", "REV_HD_HEX_20_1"]
    private let intakeOptions = ["NONE", "SERVO_CLAW", "INTAKE_ROLLER"]
    private let scoringOptions = ["NONE", "OBELISK_ONLY", "CLASSIFIER_ONLY"]

    var body: some View {
        HSplitView {
            // Left sidebar — controls
            ScrollView {
                VStack(alignment: .leading, spacing: 10) {
                    Text("FTC Auto Simulator")
                        .font(.title2.bold())
                        .padding(.bottom, 4)

                    Text("DECODE 2025-2026   [Metal]")
                        .font(.caption)
                        .foregroundColor(.secondary)

                    Divider()

                    // Dimensions
                    GroupBox("Robot Dimensions (inches)") {
                        HStack {
                            Text("Width:").frame(width: 60, alignment: .leading)
                            TextField("18.0", text: $width).frame(width: 80)
                        }
                        HStack {
                            Text("Length:").frame(width: 60, alignment: .leading)
                            TextField("18.0", text: $length).frame(width: 80)
                        }
                    }

                    // Motors
                    GroupBox("Motor Hardware") {
                        HStack {
                            Text("Chassis:").frame(width: 70, alignment: .leading)
                            Picker("", selection: $chassisMotor) {
                                ForEach(motorOptions, id: \.self) { Text($0) }
                            }.labelsHidden()
                        }
                        HStack {
                            Text("Launcher:").frame(width: 70, alignment: .leading)
                            Picker("", selection: $launcherMotor) {
                                ForEach(motorOptions, id: \.self) { Text($0) }
                            }.labelsHidden()
                        }
                    }

                    // Features
                    GroupBox("Hardware Features") {
                        Toggle("Vision Sensor", isOn: $hasVision)
                        HStack {
                            Text("Intake:").frame(width: 60, alignment: .leading)
                            Picker("", selection: $intakeType) {
                                ForEach(intakeOptions, id: \.self) { Text($0) }
                            }.labelsHidden()
                        }
                        HStack {
                            Text("Scoring:").frame(width: 60, alignment: .leading)
                            Picker("", selection: $scoringType) {
                                ForEach(scoringOptions, id: \.self) { Text($0) }
                            }.labelsHidden()
                        }
                    }

                    // Program
                    GroupBox("Auto Program") {
                        TextEditor(text: $programText)
                            .font(.system(.caption, design: .monospaced))
                            .frame(minHeight: 100)
                            .border(Color.gray.opacity(0.3))
                    }

                    // Buttons
                    VStack(spacing: 6) {
                        Button(action: startSim) {
                            Label("Start Auto Simulation", systemImage: "play.fill")
                                .frame(maxWidth: .infinity)
                        }
                        .buttonStyle(.borderedProminent)
                        .tint(.green)
                        .disabled(isRunning)

                        Button(action: resetSim) {
                            Label("Reset", systemImage: "arrow.counterclockwise")
                                .frame(maxWidth: .infinity)
                        }

                        Button(action: uploadOpMode) {
                            Label("Upload .java OpMode", systemImage: "doc.badge.plus")
                                .frame(maxWidth: .infinity)
                        }
                    }
                    .padding(.top, 4)

                    // Display controls
                    GroupBox("Display") {
                        HStack {
                            Text("Brightness:").frame(width: 70, alignment: .leading)
                            Slider(value: $brightness, in: 0.3...2.5)
                            Text(String(format: "%.1f", brightness))
                                .frame(width: 30)
                                .font(.caption)
                        }
                    }

                    // Status
                    Text(client.statusText)
                        .font(.caption)
                        .foregroundColor(.secondary)
                        .lineLimit(3)
                        .padding(.top, 4)

                    // Connection
                    Text(client.connectionState)
                        .font(.caption2)
                        .foregroundColor(client.connectionState == "Connected"
                                         ? .green : .orange)

                    Spacer()
                }
                .padding()
                .frame(minWidth: 260, idealWidth: 280)
            }

            // Right — Metal 3D viewport
            MetalView(client: client, brightness: $brightness)
                .frame(minWidth: 400)
        }
        .frame(minWidth: 800, minHeight: 600)
        .onAppear {
            client.connect()
        }
        .onDisappear {
            client.disconnect()
        }
    }

    // MARK: - actions

    private func startSim() {
        guard let w = Double(width), let l = Double(length) else { return }
        var config = RobotConfig()
        config.width = w
        config.length = l
        config.chassisMotor = chassisMotor
        config.launcherMotor = launcherMotor
        config.hasVisionSensor = hasVision
        config.intakeType = intakeType
        config.scoringCapability = scoringType

        client.sendConfigure(profile: config)
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.2) {
            client.sendProgram(programText)
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
                client.sendStart()
            }
        }
        isRunning = true
        DispatchQueue.main.asyncAfter(deadline: .now() + 32) {
            isRunning = false
        }
    }

    private func resetSim() {
        client.sendReset()
        isRunning = false
    }

    private func uploadOpMode() {
        let panel = NSOpenPanel()
        panel.allowedFileTypes = ["java"]
        panel.allowsMultipleSelection = false
        panel.begin { response in
            if response == .OK, let url = panel.url {
                client.sendUpload(path: url.path)
                if let content = try? String(contentsOf: url, encoding: .utf8) {
                    programText = content
                }
            }
        }
    }
}
