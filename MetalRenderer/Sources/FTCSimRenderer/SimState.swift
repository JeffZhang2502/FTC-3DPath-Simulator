import Foundation

/// Decoded from the JSON "fld" message — static field-element data.
struct FieldElement: Codable {
    let tp: String    // ZoneType name
    let x: Double     // centre X
    let y: Double     // centre Y (world Y → Metal Z)
    let w: Double     // width
    let d: Double     // depth
    let h: Double     // height
    let c: Int        // packed RGBA
    let lb: String    // label

    var rgba: SIMD4<Float> {
        let a = Float((c >> 24) & 0xFF) / 255.0
        let r = Float((c >> 16) & 0xFF) / 255.0
        let g = Float((c >>  8) & 0xFF) / 255.0
        let b = Float( c        & 0xFF) / 255.0
        return SIMD4<Float>(r, g, b, a)
    }

    var isObelisk: Bool { tp == "OBELISK" }
    var isWall:    Bool { tp == "PERIMETER_WALL" || tp == "OBSTACLE" }
}

/// Decoded from the JSON "frm" message — per-frame dynamic state.
struct FrameData: Codable {
    let e: Double       // elapsed
    let fn: Bool        // finished
    let st: String      // statusText
    let r: RobotPose    // robot
    let trl: [Float]?   // trail [x,y,h, ...]
    let pth: [Float]?   // path waypoints [x,y,h, ...]
    let bal: BallData?  // ball

    struct RobotPose: Codable {
        let x: Double; let y: Double; let h: Double
    }

    struct BallData: Codable {
        let a: Bool        // active
        let x: Double; let y: Double; let z: Double
        let tr: [Float]?   // trajectory [x,y,z, ...]
    }
}

/// General-purpose message from the Java server.
struct ServerMessage: Codable {
    let t: String       // type: "ready", "status", "fld", "frm", "end", "err"
    let txt: String?
    let els: [FieldElement]?
    // frame data fields (decoded separately)
}
