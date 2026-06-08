import Metal
import simd

/// Procedural mesh generators matching the geometry used in the original JavaFX code.
enum MeshGenerator {

    struct Mesh {
        let vertexBuffer: MTLBuffer
        let indexBuffer:  MTLBuffer
        let indexCount:   Int
    }

    /// A single vertex: position + normal.
    struct Vertex {
        var position: SIMD3<Float>
        var normal:   SIMD3<Float>
    }

    // MARK: - public generators

    static func makeBox(device: MTLDevice, w: Float, h: Float, d: Float) -> Mesh {
        let hw = w / 2, hh = h / 2, hd = d / 2
        // 6 faces × 4 vertices = 24 unique vertices (with per-face normals)
        let verts: [Vertex] = [
            // +X face
            Vertex(position: SIMD3(hw,  hh,  hd), normal: SIMD3( 1, 0, 0)),
            Vertex(position: SIMD3(hw, -hh,  hd), normal: SIMD3( 1, 0, 0)),
            Vertex(position: SIMD3(hw, -hh, -hd), normal: SIMD3( 1, 0, 0)),
            Vertex(position: SIMD3(hw,  hh, -hd), normal: SIMD3( 1, 0, 0)),
            // -X face
            Vertex(position: SIMD3(-hw,  hh, -hd), normal: SIMD3(-1, 0, 0)),
            Vertex(position: SIMD3(-hw, -hh, -hd), normal: SIMD3(-1, 0, 0)),
            Vertex(position: SIMD3(-hw, -hh,  hd), normal: SIMD3(-1, 0, 0)),
            Vertex(position: SIMD3(-hw,  hh,  hd), normal: SIMD3(-1, 0, 0)),
            // +Y face
            Vertex(position: SIMD3(-hw, hh, -hd), normal: SIMD3(0,  1, 0)),
            Vertex(position: SIMD3(-hw, hh,  hd), normal: SIMD3(0,  1, 0)),
            Vertex(position: SIMD3( hw, hh,  hd), normal: SIMD3(0,  1, 0)),
            Vertex(position: SIMD3( hw, hh, -hd), normal: SIMD3(0,  1, 0)),
            // -Y face
            Vertex(position: SIMD3(-hw, -hh,  hd), normal: SIMD3(0, -1, 0)),
            Vertex(position: SIMD3(-hw, -hh, -hd), normal: SIMD3(0, -1, 0)),
            Vertex(position: SIMD3( hw, -hh, -hd), normal: SIMD3(0, -1, 0)),
            Vertex(position: SIMD3( hw, -hh,  hd), normal: SIMD3(0, -1, 0)),
            // +Z face
            Vertex(position: SIMD3(-hw,  hh, hd), normal: SIMD3(0, 0,  1)),
            Vertex(position: SIMD3(-hw, -hh, hd), normal: SIMD3(0, 0,  1)),
            Vertex(position: SIMD3( hw, -hh, hd), normal: SIMD3(0, 0,  1)),
            Vertex(position: SIMD3( hw,  hh, hd), normal: SIMD3(0, 0,  1)),
            // -Z face
            Vertex(position: SIMD3( hw,  hh, -hd), normal: SIMD3(0, 0, -1)),
            Vertex(position: SIMD3( hw, -hh, -hd), normal: SIMD3(0, 0, -1)),
            Vertex(position: SIMD3(-hw, -hh, -hd), normal: SIMD3(0, 0, -1)),
            Vertex(position: SIMD3(-hw,  hh, -hd), normal: SIMD3(0, 0, -1)),
        ]
        let idx: [UInt16] = [
            0,1,2, 0,2,3,      4,5,6, 4,6,7,
            8,9,10, 8,10,11,   12,13,14, 12,14,15,
            16,17,18, 16,18,19, 20,21,22, 20,22,23,
        ]
        return build(device: device, verts: verts, indices: idx)
    }

    static func makeSphere(device: MTLDevice, radius: Float, stacks: Int = 16, slices: Int = 24) -> Mesh {
        var verts: [Vertex] = []
        var idx: [UInt16] = []

        for i in 0...stacks {
            let phi = Float(i) / Float(stacks) * .pi
            let y = cos(phi) * radius
            let r = sin(phi) * radius
            for j in 0...slices {
                let theta = Float(j) / Float(slices) * 2 * .pi
                let x = r * cos(theta)
                let z = r * sin(theta)
                let n = normalize(SIMD3<Float>(x, y, z))
                verts.append(Vertex(position: n * radius, normal: n))
            }
        }
        for i in 0..<stacks {
            for j in 0..<slices {
                let a = UInt16(i * (slices + 1) + j)
                let b = UInt16(a + UInt16(slices) + 1)
                idx.append(contentsOf: [a, b, a + 1, b, b + 1, a + 1])
            }
        }
        return build(device: device, verts: verts, indices: idx)
    }

    static func makeCylinder(device: MTLDevice, radius: Float, height: Float, slices: Int = 24) -> Mesh {
        let hh = height / 2
        var verts: [Vertex] = []
        var idx: [UInt16] = []

        // top cap centre, bottom cap centre, side vertices (2 per slice: top + bottom)
        verts.append(Vertex(position: SIMD3(0, hh, 0), normal: SIMD3(0, 1, 0)))   // index 0
        verts.append(Vertex(position: SIMD3(0, -hh, 0), normal: SIMD3(0, -1, 0))) // index 1

        for j in 0...slices {
            let theta = Float(j) / Float(slices) * 2 * .pi
            let x = cos(theta) * radius
            let z = sin(theta) * radius
            let n = SIMD3<Float>(x, 0, z) / radius
            // top ring vertex
            verts.append(Vertex(position: SIMD3(x, hh, z), normal: n))
            // bottom ring vertex
            verts.append(Vertex(position: SIMD3(x, -hh, z), normal: n))
        }

        // top cap fans
        for j in 0..<slices {
            idx.append(contentsOf: [0, UInt16(2 + j * 2 + 2), UInt16(2 + j * 2)])
        }
        // bottom cap fans
        for j in 0..<slices {
            idx.append(contentsOf: [1, UInt16(3 + j * 2), UInt16(3 + j * 2 + 2)])
        }
        // side quads
        for j in 0..<slices {
            let a = UInt16(2 + j * 2)
            let b = UInt16(2 + j * 2 + 1)
            let c = UInt16(2 + (j + 1) % slices * 2)
            let d = UInt16(2 + (j + 1) % slices * 2 + 1)
            // side normals should face outward; use average
            idx.append(contentsOf: [a, b, c, b, d, c])
        }
        return build(device: device, verts: verts, indices: idx)
    }

    /// Equilateral triangular prism, matching the JavaFX obelisk.
    /// Face width s, height h, centred at origin, base on Y=0.
    static func makeTriPrism(device: MTLDevice, faceWidth s: Float, height h: Float) -> Mesh {
        let R = s / sqrt(3.0)
        let halfH = h / 2
        let rt32 = s * 0.5  // R * sqrt(3)/2, approximates 0.8660254 * R

        // bottom triangle (Y = -halfH) facing down
        // top triangle (Y = +halfH) facing up
        let b0 = SIMD3<Float>( R,     -halfH,  0)
        let b1 = SIMD3<Float>(-R/2,   -halfH,  rt32)
        let b2 = SIMD3<Float>(-R/2,   -halfH, -rt32)
        let t0 = SIMD3<Float>( R,      halfH,  0)
        let t1 = SIMD3<Float>(-R/2,    halfH,  rt32)
        let t2 = SIMD3<Float>(-R/2,    halfH, -rt32)

        let down  = SIMD3<Float>(0, -1, 0)
        let up    = SIMD3<Float>(0,  1, 0)

        // side normals (pointing outward from centre of each face)
        let n01 = normalize(cross(t1 - b0, b1 - b0))
        let n12 = normalize(cross(t2 - b1, b2 - b1))
        let n20 = normalize(cross(t0 - b2, b0 - b2))

        let verts: [Vertex] = [
            // bottom face
            Vertex(position: b0, normal: down), Vertex(position: b1, normal: down), Vertex(position: b2, normal: down),
            // top face
            Vertex(position: t0, normal: up), Vertex(position: t1, normal: up), Vertex(position: t2, normal: up),
            // side 0-1
            Vertex(position: b0, normal: n01), Vertex(position: b1, normal: n01),
            Vertex(position: t0, normal: n01), Vertex(position: t1, normal: n01),
            // side 1-2
            Vertex(position: b1, normal: n12), Vertex(position: b2, normal: n12),
            Vertex(position: t1, normal: n12), Vertex(position: t2, normal: n12),
            // side 2-0
            Vertex(position: b2, normal: n20), Vertex(position: b0, normal: n20),
            Vertex(position: t2, normal: n20), Vertex(position: t0, normal: n20),
        ]

        let idx: [UInt16] = [
            0,2,1,   3,4,5,             // bottom, top
            6,7,8, 8,7,9,              // side 0-1
            10,11,12, 12,11,13,        // side 1-2
            14,15,16, 16,15,17,        // side 2-0
        ]

        return build(device: device, verts: verts, indices: idx)
    }

    // MARK: - buffer construction

    private static func build(device: MTLDevice, verts: [Vertex], indices: [UInt16]) -> Mesh {
        guard let vb = device.makeBuffer(bytes: verts, length: verts.count * MemoryLayout<Vertex>.stride),
              let ib = device.makeBuffer(bytes: indices, length: indices.count * MemoryLayout<UInt16>.stride)
        else { fatalError("Failed to create mesh buffers") }
        return Mesh(vertexBuffer: vb, indexBuffer: ib, indexCount: indices.count)
    }
}
