import simd

/// Orbit camera for viewing the 144×144 inch FTC field.
struct Camera {
    var azimuth: Float   = 45    // degrees around Y axis
    var elevation: Float = 40    // degrees above XZ plane
    var distance: Float  = 140   // inches from origin

    var fov: Float       = 45
    var nearZ: Float     = 1
    var farZ: Float      = 800

    /// Combined view-projection matrix.
    func viewProjectionMatrix(aspect: Float) -> float4x4 {
        let proj = projectionMatrix(aspect: aspect)
        let view = viewMatrix()
        return proj * view
    }

    func viewMatrix() -> float4x4 {
        let azRad  = azimuth  * .pi / 180
        let elRad  = elevation * .pi / 180

        let eyex = distance * cos(elRad) * sin(azRad)
        let eyey = distance * sin(elRad)
        let eyez = distance * cos(elRad) * cos(azRad)

        let target = SIMD3<Float>(0, 0, 0)
        let up     = SIMD3<Float>(0, 1, 0)

        return float4x4(lookAt: SIMD3<Float>(eyex, eyey, eyez),
                        target: target, up: up)
    }

    func projectionMatrix(aspect: Float) -> float4x4 {
        let fovRad = fov * .pi / 180
        return float4x4(perspectiveFov: fovRad, aspect: aspect,
                        nearZ: nearZ, farZ: farZ)
    }
}

// MARK: - float4x4 helpers

extension float4x4 {
    init(lookAt eye: SIMD3<Float>, target: SIMD3<Float>, up: SIMD3<Float>) {
        let f = normalize(target - eye)
        let s = normalize(cross(f, up))
        let u = cross(s, f)
        self.init(columns: (
            SIMD4<Float>(s.x, u.x, -f.x, 0),
            SIMD4<Float>(s.y, u.y, -f.y, 0),
            SIMD4<Float>(s.z, u.z, -f.z, 0),
            SIMD4<Float>(-dot(s, eye), -dot(u, eye), dot(f, eye), 1)
        ))
    }

    init(perspectiveFov fov: Float, aspect: Float, nearZ: Float, farZ: Float) {
        let y = 1 / tan(fov * 0.5)
        let x = y / aspect
        let zs = farZ / (nearZ - farZ)
        self.init(columns: (
            SIMD4<Float>(x, 0, 0, 0),
            SIMD4<Float>(0, y, 0, 0),
            SIMD4<Float>(0, 0, zs, -1),
            SIMD4<Float>(0, 0, zs * nearZ, 0)
        ))
    }
}
