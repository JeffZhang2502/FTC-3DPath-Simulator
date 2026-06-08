import MetalKit
import simd

final class Renderer: NSObject, MTKViewDelegate {

    // MARK: - Metal objects
    private let device: MTLDevice
    private let queue: MTLCommandQueue
    private let pipeline: MTLRenderPipelineState
    private let depthState: MTLDepthStencilState

    // MARK: - meshes (shared geometry)
    private let boxMesh:       MeshGenerator.Mesh
    private let sphereMesh:    MeshGenerator.Mesh
    private let cylinderMesh:  MeshGenerator.Mesh
    private let triPrismMesh:  MeshGenerator.Mesh

    // MARK: - camera
    var camera = Camera()
    private var aspectRatio: Float = 1.0

    // MARK: - lighting (adjustable from UI)
    var brightness: Float = 1.0

    // MARK: - scene data (set from outside)
    var fieldElements: [FieldElement] = []
    var frame: FrameData?

    // MARK: - colour palette (deep saturated)
    private let matFloor   = SIMD3<Float>(0.60, 0.60, 0.78)
    private let matTileA   = SIMD3<Float>(0.72, 0.72, 0.90)
    private let matTileB   = SIMD3<Float>(0.63, 0.63, 0.78)
    private let matRobot   = SIMD3<Float>(0.24, 1.00, 0.24)
    private let matDir     = SIMD3<Float>(1.0,   1.0,   0.0)
    private let matTrail   = SIMD3<Float>(1.0,   0.90, 0.20)
    private let matPath    = SIMD3<Float>(0.10, 1.0,   0.20)
    private let matBall    = SIMD3<Float>(1.0,   0.65, 0.0)
    private let matBallTr  = SIMD3<Float>(1.0,   0.55, 0.0)
    private let matColCyl  = SIMD3<Float>(0.60, 0.60, 0.72)

    // MARK: - init

    init(device: MTLDevice, view: MTKView) {
        self.device = device
        print("[Renderer] device: \(device.name)")
        self.queue  = device.makeCommandQueue()!
        print("[Renderer] command queue created")

        let lib: MTLLibrary
        do {
            lib = try device.makeLibrary(source: shaderSource, options: nil)
            print("[Renderer] shader library compiled")
        } catch {
            fatalError("[Renderer] Shader compilation failed: \(error)")
        }
        let desc = MTLRenderPipelineDescriptor()
        desc.vertexFunction   = lib.makeFunction(name: "vertex_main")
        desc.fragmentFunction = lib.makeFunction(name: "fragment_main")
        desc.colorAttachments[0].pixelFormat = view.colorPixelFormat
        desc.depthAttachmentPixelFormat = view.depthStencilPixelFormat

        // Vertex descriptor: interleaved position(float3) + normal(float3)
        let vd = MTLVertexDescriptor()
        vd.attributes[0].format = .float3
        vd.attributes[0].offset = 0
        vd.attributes[0].bufferIndex = 0
        vd.attributes[1].format = .float3
        vd.attributes[1].offset = MemoryLayout<SIMD3<Float>>.stride
        vd.attributes[1].bufferIndex = 0
        vd.layouts[0].stride = MemoryLayout<MeshGenerator.Vertex>.stride
        vd.layouts[0].stepFunction = .perVertex
        desc.vertexDescriptor = vd

        // Enable alpha blending
        let ca = desc.colorAttachments[0]!
        ca.isBlendingEnabled = true
        ca.rgbBlendOperation = .add
        ca.alphaBlendOperation = .add
        ca.sourceRGBBlendFactor = .sourceAlpha
        ca.destinationRGBBlendFactor = .oneMinusSourceAlpha
        ca.sourceAlphaBlendFactor = .sourceAlpha
        ca.destinationAlphaBlendFactor = .oneMinusSourceAlpha

        do {
            pipeline = try device.makeRenderPipelineState(descriptor: desc)
            print("[Renderer] pipeline state created")
        } catch {
            fatalError("[Renderer] Pipeline creation failed: \(error)")
        }

        let dsDesc = MTLDepthStencilDescriptor()
        dsDesc.depthCompareFunction = .less
        dsDesc.isDepthWriteEnabled = true
        guard let ds = device.makeDepthStencilState(descriptor: dsDesc) else {
            fatalError("[Renderer] Depth-stencil state creation failed")
        }
        depthState = ds

        boxMesh      = MeshGenerator.makeBox(device: device, w: 1, h: 1, d: 1)
        sphereMesh   = MeshGenerator.makeSphere(device: device, radius: 1)
        cylinderMesh = MeshGenerator.makeCylinder(device: device, radius: 1, height: 1)
        triPrismMesh = MeshGenerator.makeTriPrism(device: device,
                                                   faceWidth: Float(FieldMap.obeliskFaceWidth),
                                                   height:    Float(FieldMap.obeliskHeight))

        super.init()
        view.delegate = self
        view.depthStencilPixelFormat = .depth32Float
        view.clearColor = MTLClearColor(red: 0.55, green: 0.60, blue: 0.68, alpha: 1)
        view.sampleCount = 4  // MSAA
    }

    // MARK: - MTKViewDelegate

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        aspectRatio = Float(size.width / max(size.height, 1))
    }

    func draw(in view: MTKView) {
        guard let drawable = view.currentDrawable,
              let desc = view.currentRenderPassDescriptor,
              let cmdBuf = queue.makeCommandBuffer(),
              let encoder = cmdBuf.makeRenderCommandEncoder(descriptor: desc)
        else { return }

        encoder.setRenderPipelineState(pipeline)
        encoder.setDepthStencilState(depthState)
        encoder.setFrontFacing(.counterClockwise)
        encoder.setCullMode(.back)

        let vp = camera.viewProjectionMatrix(aspect: aspectRatio)

        drawField(encoder: encoder, vp: vp)
        drawRobot(encoder: encoder, vp: vp)
        drawBall(encoder: encoder, vp: vp)

        encoder.endEncoding()
        cmdBuf.present(drawable)
        cmdBuf.commit()
    }

    // MARK: - drawing routines

    private func drawField(encoder: MTLRenderCommandEncoder, vp: float4x4) {
        // Floor
        drawBox(encoder: encoder, vp: vp, w: 144, h: 0.1, d: 144,
                color: matFloor, alpha: 1, tx: 0, ty: 0.05, tz: 0)

        // 6×6 tile grid
        let halfField: Float = 72
        for i in 0..<6 {
            for j in 0..<6 {
                let col = (i + j) % 2 == 0 ? matTileA : matTileB
                let x = -halfField + 12 + Float(i) * 24
                let z = -halfField + 12 + Float(j) * 24
                drawBox(encoder: encoder, vp: vp, w: 24, h: 0.02, d: 24,
                        color: col, alpha: 1, tx: x, ty: 0.11, tz: z)
            }
        }

        // Field elements from server
        for el in fieldElements {
            let rgba = el.rgba
            let color = SIMD3<Float>(rgba.x, rgba.y, rgba.z)
            let alpha = rgba.w
            let x = Float(el.x)
            let z = Float(el.y)     // world Y → Metal Z
            let w = Float(el.w)
            let d = Float(el.d)
            let h = Float(el.h > 0.05 ? el.h : 0.15)

            if el.isObelisk {
                // Triangular prism
                let cx = Float(FieldMap.obeliskCenterX)
                let cz = Float(FieldMap.obeliskCenterY)
                drawTriPrism(encoder: encoder, vp: vp, color: color, alpha: alpha,
                             tx: cx, ty: h / 2, tz: cz)
                // Semi-transparent collision cylinder
                drawCylinder(encoder: encoder, vp: vp,
                             radius: Float(FieldMap.obeliskCircumRadius),
                             height: h, color: matColCyl, alpha: 0.3,
                             tx: cx, ty: h / 2, tz: cz)
            } else {
                drawBox(encoder: encoder, vp: vp, w: w, h: h, d: d,
                        color: color, alpha: alpha,
                        tx: x, ty: h / 2, tz: z)
            }
        }
    }

    private func drawRobot(encoder: MTLRenderCommandEncoder, vp: float4x4) {
        guard let f = frame else { return }

        let rx = Float(f.r.x)
        let rz = Float(f.r.y)
        let rh = Float(f.r.h)
        let rw: Float = 18       // robot width (hardcoded for now)
        let rl: Float = 18
        let robotH: Float = 8

        // Robot body
        drawBox(encoder: encoder, vp: vp, w: rw, h: robotH, d: rl,
                color: matRobot, alpha: 1,
                tx: rx, ty: robotH / 2, tz: rz, rotY: -rad2deg(rh))

        // Direction indicator dot
        drawSphere(encoder: encoder, vp: vp, radius: 1.5,
                   color: matDir, alpha: 1,
                   tx: rx + rl / 2 * cos(rh), ty: robotH - 1,
                   tz: rz + rl / 2 * sin(rh))

        // Trail
        if let trail = f.trl {
            for i in stride(from: 0, to: trail.count, by: 6) {  // every 2nd point (skip for perf)
                guard i + 2 < trail.count else { break }
                drawSphere(encoder: encoder, vp: vp, radius: 0.8,
                           color: matTrail, alpha: 0.6,
                           tx: trail[i], ty: 0.5, tz: trail[i + 1])
            }
        }

        // Path waypoints
        if let path = f.pth {
            for i in stride(from: 0, to: path.count, by: 3) {
                guard i + 2 < path.count else { break }
                drawBox(encoder: encoder, vp: vp, w: 1.5, h: 0.2, d: 1.5,
                        color: matPath, alpha: 0.7,
                        tx: path[i], ty: 0.6, tz: path[i + 1])
            }
        }
    }

    private func drawBall(encoder: MTLRenderCommandEncoder, vp: float4x4) {
        guard let f = frame, let bal = f.bal, bal.a else { return }

        drawSphere(encoder: encoder, vp: vp, radius: 1.5,
                   color: matBall, alpha: 1,
                   tx: Float(bal.x), ty: Float(bal.z), tz: Float(bal.y))

        // Trajectory trail
        if let traj = bal.tr {
            for i in stride(from: 0, to: traj.count, by: 9) {  // every 3rd point
                guard i + 2 < traj.count else { break }
                drawSphere(encoder: encoder, vp: vp, radius: 0.6,
                           color: matBallTr, alpha: 0.7,
                           tx: traj[i], ty: traj[i + 2], tz: traj[i + 1])
            }
        }
    }

    // MARK: - primitive draw helpers (push constant style)

    private func setUniforms(encoder: MTLRenderCommandEncoder, vp: float4x4,
                             model: float4x4, color: SIMD3<Float>, alpha: Float) {
        var u = Uniforms(
            modelMatrix: model,
            viewProjection: vp,
            normalMatrix: float3x3(
                SIMD3(model.columns.0.x, model.columns.0.y, model.columns.0.z),
                SIMD3(model.columns.1.x, model.columns.1.y, model.columns.1.z),
                SIMD3(model.columns.2.x, model.columns.2.y, model.columns.2.z)
            ),
            materialColor: color,
            alpha: alpha,
            brightness: brightness
        )
        encoder.setVertexBytes(&u, length: MemoryLayout<Uniforms>.stride, index: 1)
        encoder.setFragmentBytes(&u, length: MemoryLayout<Uniforms>.stride, index: 1)
    }

    private func drawMesh(encoder: MTLRenderCommandEncoder, mesh: MeshGenerator.Mesh) {
        encoder.setVertexBuffer(mesh.vertexBuffer, offset: 0, index: 0)
        encoder.drawIndexedPrimitives(type: .triangle, indexCount: mesh.indexCount,
                                      indexType: .uint16, indexBuffer: mesh.indexBuffer,
                                      indexBufferOffset: 0)
    }

    private func drawBox(encoder: MTLRenderCommandEncoder, vp: float4x4,
                         w: Float, h: Float, d: Float,
                         color: SIMD3<Float>, alpha: Float,
                         tx: Float, ty: Float, tz: Float,
                         rotY: Float = 0) {
        var model = float4x4(translation: SIMD3(tx, ty, tz))
        if rotY != 0 {
            model = model * float4x4(rotationY: rotY * .pi / 180)
        }
        model = model * float4x4(scale: SIMD3(w, h, d))
        setUniforms(encoder: encoder, vp: vp, model: model, color: color, alpha: alpha)
        drawMesh(encoder: encoder, mesh: boxMesh)
    }

    private func drawSphere(encoder: MTLRenderCommandEncoder, vp: float4x4,
                            radius: Float,
                            color: SIMD3<Float>, alpha: Float,
                            tx: Float, ty: Float, tz: Float) {
        let model = float4x4(translation: SIMD3(tx, ty, tz))
                  * float4x4(scale: SIMD3(repeating: radius))
        setUniforms(encoder: encoder, vp: vp, model: model, color: color, alpha: alpha)
        drawMesh(encoder: encoder, mesh: sphereMesh)
    }

    private func drawCylinder(encoder: MTLRenderCommandEncoder, vp: float4x4,
                              radius: Float, height: Float,
                              color: SIMD3<Float>, alpha: Float,
                              tx: Float, ty: Float, tz: Float) {
        let model = float4x4(translation: SIMD3(tx, ty, tz))
                  * float4x4(scale: SIMD3(radius, height, radius))
        setUniforms(encoder: encoder, vp: vp, model: model, color: color, alpha: alpha)
        drawMesh(encoder: encoder, mesh: cylinderMesh)
    }

    private func drawTriPrism(encoder: MTLRenderCommandEncoder, vp: float4x4,
                              color: SIMD3<Float>, alpha: Float,
                              tx: Float, ty: Float, tz: Float) {
        let model = float4x4(translation: SIMD3(tx, ty, tz))
        setUniforms(encoder: encoder, vp: vp, model: model, color: color, alpha: alpha)
        drawMesh(encoder: encoder, mesh: triPrismMesh)
    }
}

// MARK: - shader uniform struct (must match Shaders.metal)

private struct Uniforms {
    var modelMatrix: float4x4
    var viewProjection: float4x4
    var normalMatrix: float3x3
    var materialColor: SIMD3<Float>
    var alpha: Float
    var brightness: Float
}

// MARK: - float4x4 helpers (translation, rotation, scale)

extension float4x4 {
    init(translation t: SIMD3<Float>) {
        self.init(columns: (
            SIMD4(1, 0, 0, 0),
            SIMD4(0, 1, 0, 0),
            SIMD4(0, 0, 1, 0),
            SIMD4(t.x, t.y, t.z, 1)
        ))
    }

    init(rotationY radians: Float) {
        let c = cos(radians), s = sin(radians)
        self.init(columns: (
            SIMD4( c, 0, s, 0),
            SIMD4( 0, 1, 0, 0),
            SIMD4(-s, 0, c, 0),
            SIMD4( 0, 0, 0, 1)
        ))
    }

    init(scale s: SIMD3<Float>) {
        self.init(columns: (
            SIMD4(s.x, 0, 0, 0),
            SIMD4(0, s.y, 0, 0),
            SIMD4(0, 0, s.z, 0),
            SIMD4(0, 0, 0, 1)
        ))
    }
}

private func rad2deg(_ r: Float) -> Float { r * 180 / .pi }

// MARK: - constants matching FieldMap.java

private enum FieldMap {
    static let obeliskFaceWidth: Double  = 11.0
    static let obeliskHeight: Double     = 23.0
    static let obeliskInradius: Double   = 11.0 * sqrt(3.0) / 6.0
    static let obeliskCircumRadius: Double = 11.0 * sqrt(3.0) / 3.0
    static let obeliskCenterX: Double    = 0.0
    static let obeliskCenterY: Double    = -72.0 - obeliskInradius
}

// MARK: - embedded Metal shader source (compiled at runtime)

private let shaderSource = """
#include <metal_stdlib>
using namespace metal;

struct VertexIn {
    float3 position [[attribute(0)]];
    float3 normal   [[attribute(1)]];
};

struct VertexOut {
    float4 position [[position]];
    float3 worldPos;
    float3 normal;
};

struct Uniforms {
    float4x4 modelMatrix;
    float4x4 viewProjection;
    float3x3 normalMatrix;
    float3   materialColor;
    float    alpha;
    float    brightness;
};

constant float3 dirLightDir   = float3(0.43193, 0.86387, 0.25916);
constant float3 dirLightCol   = float3(2.0, 2.0, 2.1);
constant float3 ambientCol    = float3(0.30, 0.30, 0.32);

vertex VertexOut vertex_main(VertexIn in [[stage_in]],
                             constant Uniforms &u [[buffer(1)]]) {
    VertexOut out;
    float4 world = u.modelMatrix * float4(in.position, 1.0);
    out.worldPos = world.xyz;
    out.position = u.viewProjection * world;
    out.normal   = normalize(u.normalMatrix * in.normal);
    return out;
}

fragment float4 fragment_main(VertexOut in [[stage_in]],
                              constant Uniforms &u [[buffer(1)]]) {
    float3 N = normalize(in.normal);
    float3 viewDir = normalize(-in.worldPos);

    float  NdotL = max(dot(N, dirLightDir), 0.0);
    float3 ambient   = u.materialColor * ambientCol;
    float3 diffuse   = u.materialColor * dirLightCol * NdotL;
    float3 halfDir   = normalize(dirLightDir + viewDir);
    float3 specular  = dirLightCol * pow(max(dot(N, halfDir), 0.0), 32.0) * 0.4;

    float3 col = (ambient + diffuse + specular) * u.brightness;
    return float4(col, u.alpha);
}
"""
