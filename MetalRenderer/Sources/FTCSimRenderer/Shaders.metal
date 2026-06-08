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
};

constant float3 lightDir   = normalize(float3(0.5, 1.0, 0.3));
constant float3 ambientCol = float3(0.2, 0.2, 0.25);
constant float3 lightCol   = float3(0.85, 0.85, 0.9);

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
    float  NdotL = max(dot(N, lightDir), 0.0);

    float3 ambient  = u.materialColor * ambientCol;
    float3 diffuse  = u.materialColor * lightCol * NdotL;

    // Blinn-Phong specular
    float3 viewDir = normalize(-in.worldPos);
    float3 halfVec = normalize(lightDir + viewDir);
    float  spec    = pow(max(dot(N, halfVec), 0.0), 32.0);
    float3 specular = lightCol * spec * 0.4;

    float3 col = ambient + diffuse + specular;
    return float4(col, u.alpha);
}
