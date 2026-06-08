import Foundation
import Network

/// Manages TCP connection to the headless Java simulation server.
@MainActor
final class SimClient: ObservableObject {

    @Published var connectionState = "Disconnected"
    @Published var statusText = ""
    @Published var fieldElements: [FieldElement] = []
    @Published var frame: FrameData?
    @Published var finished = false

    private var connection: NWConnection?
    private var recvBuffer = ""

    func connect(host: String = "localhost", port: UInt16 = 9876) {
        let nwPort = NWEndpoint.Port(rawValue: port)!
        connection = NWConnection(host: NWEndpoint.Host(host), port: nwPort, using: .tcp)
        connection?.stateUpdateHandler = { [weak self] state in
            Task { @MainActor in
                switch state {
                case .ready:
                    self?.connectionState = "Connected"
                    self?.receive()
                case .failed(let err):
                    self?.connectionState = "Error: \(err.localizedDescription)"
                case .cancelled:
                    self?.connectionState = "Disconnected"
                default: break
                }
            }
        }
        connection?.start(queue: .global())
    }

    func disconnect() {
        connection?.cancel()
        connection = nil
    }

    // MARK: - send commands

    func sendConfigure(profile: RobotConfig) {
        let json = """
        {"cmd":"cfg","w":\(profile.width),"l":\(profile.length),"h":\(profile.height),\
        "cm":"\(profile.chassisMotor)","wr":\(profile.wheelRadius),\
        "lm":"\(profile.launcherMotor)","la":\(profile.launchAngle),"lw":\(profile.launcherWheelRadius),\
        "vs":\(profile.hasVisionSensor),"it":"\(profile.intakeType)","sc":"\(profile.scoringCapability)"}
        """
        send(json)
    }

    func sendProgram(_ text: String) {
        let escaped = text
            .replacingOccurrences(of: "\\", with: "\\\\")
            .replacingOccurrences(of: "\"", with: "\\\"")
            .replacingOccurrences(of: "\n", with: "\\n")
            .replacingOccurrences(of: "\r", with: "")
        send("{\"cmd\":\"prog\",\"txt\":\"\(escaped)\"}")
    }

    func sendStart()    { send("{\"cmd\":\"start\"}") }
    func sendReset()    { send("{\"cmd\":\"reset\"}") }

    func sendUpload(path: String) {
        let escaped = path.replacingOccurrences(of: "\"", with: "\\\"")
        send("{\"cmd\":\"upload\",\"path\":\"\(escaped)\"}")
    }

    // MARK: - internal

    private func send(_ json: String) {
        guard let conn = connection else { return }
        var data = json.data(using: .utf8)!
        data.append(10) // newline
        conn.send(content: data, completion: .contentProcessed({ _ in }))
    }

    private func receive() {
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 65536) {
            [weak self] data, _, _, error in
            guard let self else { return }
            if let data, let str = String(data: data, encoding: .utf8) {
                Task { @MainActor in self.processChunk(str) }
            }
            if error == nil {
                self.receive()
            }
        }
    }

    private func processChunk(_ chunk: String) {
        recvBuffer += chunk
        while let nl = recvBuffer.firstIndex(of: "\n") {
            let line = String(recvBuffer[..<nl])
            recvBuffer = String(recvBuffer[recvBuffer.index(after: nl)...])
            processLine(line.trimmingCharacters(in: .whitespaces))
        }
    }

    private func processLine(_ line: String) {
        guard !line.isEmpty, let data = line.data(using: .utf8) else { return }

        // Try to decode as a type-tagged message
        if let msg = try? JSONDecoder().decode(ServerMessage.self, from: data) {
            switch msg.t {
            case "ready":
                statusText = "[READY] Connected to simulation server."
            case "status":
                if let txt = msg.txt { statusText = txt }
            case "fld":
                if let els = msg.els { fieldElements = els }
            case "end":
                finished = true
                if let txt = msg.txt { statusText = txt }
            case "err":
                if let txt = msg.txt { statusText = "[ERROR] \(txt)" }
            default:
                break
            }
        }

        // Try to decode as frame data
        if let frm = try? JSONDecoder().decode(FrameData.self, from: data) {
            frame = frm
            statusText = frm.st
            finished = frm.fn
        }
    }
}

// MARK: - robot configuration model

struct RobotConfig {
    var width: Double  = 18.0
    var length: Double = 18.0
    var height: Double = 8.0
    var chassisMotor     = "GOBILDA_5203_19_2"
    var wheelRadius: Double = 2.0
    var launcherMotor    = "GOBILDA_5203_19_2"
    var launchAngle: Double = 45.0
    var launcherWheelRadius: Double = 1.5
    var hasVisionSensor  = false
    var intakeType       = "NONE"
    var scoringCapability = "NONE"
}
