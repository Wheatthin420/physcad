// Lambert two-light rig over a flat clear color; input colors are display
// values because the render target is Rgba8UnormSrgb.

struct CameraU {
    view_proj: mat4x4<f32>,
};

struct ModelU {
    model: mat4x4<f32>,
    color: vec4<f32>,
};

@group(0) @binding(0) var<uniform> camera: CameraU;
@group(1) @binding(0) var<uniform> model: ModelU;

struct VsIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct VsOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
};

@vertex
fn vs(v: VsIn) -> VsOut {
    var out: VsOut;
    let world = model.model * vec4<f32>(v.position, 1.0);
    out.clip = camera.view_proj * world;
    out.world_normal = normalize((model.model * vec4<f32>(v.normal, 0.0)).xyz);
    return out;
}

@fragment
fn fs(in: VsOut) -> @location(0) vec4<f32> {
    let n = normalize(in.world_normal);
    let key = normalize(vec3<f32>(0.4, 0.85, 0.35));
    let fill = normalize(vec3<f32>(-0.5, 0.2, -0.6));
    let diff = max(dot(n, key), 0.0) * 0.75 + max(dot(n, fill), 0.0) * 0.20;
    let rgb = model.color.rgb * (0.30 + diff);
    return vec4<f32>(rgb, 1.0);
}
