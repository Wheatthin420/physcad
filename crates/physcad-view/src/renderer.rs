//! Headless wgpu renderer: draws the sample scene offscreen and writes a
//! PNG. Runs with no display server; the adapter selection prefers a CPU
//! (software) device so tests never contend with the shared GPU.

use std::path::Path;

use bytemuck::{Pod, Zeroable};
use physcad::camera::OrbitCamera;

use crate::scene::{SceneItem, sample_scene};

const SHADER: &str = include_str!("../assets/shaders/mesh.wgsl");

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct CameraUniform {
    view_proj: [[f32; 4]; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct ModelUniform {
    model: [[f32; 4]; 4],
    color: [f32; 4],
}

/// What the capture actually used and what it produced.
#[derive(Clone, Debug)]
pub struct RenderReport {
    pub adapter_name: String,
    pub adapter_device_type: String,
    pub adapter_backend: String,
    pub width: u32,
    pub height: u32,
    pub png_path: String,
    pub distinct_colors: usize,
}

pub fn capture_png(path: &Path, width: u32, height: u32) -> Result<RenderReport, String> {
    render_scene_png(&sample_scene(), path, width, height)
}

pub fn render_scene_png(
    scene: &[SceneItem],
    path: &Path,
    width: u32,
    height: u32,
) -> Result<RenderReport, String> {
    if width == 0 || height == 0 {
        return Err("width/height must be non-zero".into());
    }
    // bytes_per_row for the texture->buffer copy must be a multiple of 256.
    if !(width * 4).is_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT) {
        return Err(format!(
            "width {width} invalid: width*4 must be a multiple of {}",
            wgpu::COPY_BYTES_PER_ROW_ALIGNMENT
        ));
    }

    let mut instance_desc = wgpu::InstanceDescriptor::new_without_display_handle();
    instance_desc.backends = wgpu::Backends::VULKAN;
    let instance = wgpu::Instance::new(instance_desc);
    let adapters = pollster::block_on(instance.enumerate_adapters(wgpu::Backends::VULKAN));
    if adapters.is_empty() {
        return Err("wgpu found no Vulkan adapters (no ICD installed?)".into());
    }
    let enumerated: Vec<String> = adapters
        .iter()
        .map(|a| {
            let i = a.get_info();
            format!("{} [{:?}]", i.name, i.device_type)
        })
        .collect();

    // Prefer a software (Cpu) adapter: deterministic and it keeps the shared
    // GPU free for the inference workload on this box.
    let adapter = adapters
        .iter()
        .find(|a| a.get_info().device_type == wgpu::DeviceType::Cpu)
        .or_else(|| {
            adapters.iter().find(|a| {
                matches!(
                    a.get_info().device_type,
                    wgpu::DeviceType::DiscreteGpu | wgpu::DeviceType::IntegratedGpu
                )
            })
        })
        .ok_or_else(|| format!("no usable adapter among: {enumerated:?}"))?;

    let info = adapter.get_info();
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("physcad-capture"),
        ..Default::default()
    }))
    .map_err(|e| format!("request_device failed (adapters: {enumerated:?}): {e}"))?;

    let color_format = wgpu::TextureFormat::Rgba8UnormSrgb;
    let color_tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("capture-color"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: color_format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let depth_tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("capture-depth"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth24Plus,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });

    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("mesh-shader"),
        source: wgpu::ShaderSource::Wgsl(SHADER.into()),
    });

    let vertex_layout = wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &[
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x3,
                offset: 0,
                shader_location: 0,
            },
            wgpu::VertexAttribute {
                format: wgpu::VertexFormat::Float32x3,
                offset: 12,
                shader_location: 1,
            },
        ],
    };

    let camera_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("camera-bgl"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
    });
    let model_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("model-bgl"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            // The vertex stage reads model.model, the fragment stage reads
            // model.color — the whole uniform is visible to both.
            visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
    });
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("mesh-layout"),
        immediate_size: 0,
        bind_group_layouts: &[Some(&camera_bgl), Some(&model_bgl)],
    });

    let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("mesh-pipeline"),
        layout: Some(&pipeline_layout),
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs"),
            compilation_options: Default::default(),
            buffers: &[Some(vertex_layout)],
        },
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: Some(wgpu::Face::Front),
            unclipped_depth: false,
            polygon_mode: wgpu::PolygonMode::Fill,
            conservative: false,
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth24Plus,
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: &shader,
            entry_point: Some("fs"),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: color_format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    });

    let camera = OrbitCamera {
        target: glam::Vec3::new(0.0, 1.1, 0.0),
        ..OrbitCamera::default()
    };
    let aspect = width as f32 / height as f32;
    let camera_uniform = CameraUniform {
        view_proj: camera.view_projection(aspect).to_cols_array_2d(),
    };
    let camera_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("camera-uniform"),
        size: std::mem::size_of::<CameraUniform>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    queue.write_buffer(&camera_buf, 0, bytemuck::bytes_of(&camera_uniform));
    let camera_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("camera-bg"),
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: camera_buf.as_entire_binding(),
        }],
    });

    let mut pass_encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("capture-pass"),
    });
    {
        let mut rpass = pass_encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("capture-pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &color_tex.create_view(&wgpu::TextureViewDescriptor::default()),
                resolve_target: None,
                depth_slice: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: 0.94,
                        g: 0.95,
                        b: 0.97,
                        a: 1.0,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &depth_tex.create_view(&wgpu::TextureViewDescriptor::default()),
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        rpass.set_pipeline(&pipeline);
        rpass.set_bind_group(0, &camera_bg, &[]);
        for item in scene {
            let vertices: Vec<Vertex> = item
                .mesh
                .positions
                .iter()
                .zip(item.mesh.normals.iter())
                .map(|(p, n)| Vertex {
                    position: *p,
                    normal: *n,
                })
                .collect();
            let vbo = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(item.name),
                size: (vertices.len() * std::mem::size_of::<Vertex>()) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            queue.write_buffer(&vbo, 0, bytemuck::cast_slice(&vertices));
            let ibo = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("indices"),
                size: (item.mesh.indices.len() * std::mem::size_of::<u32>()) as u64,
                usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            queue.write_buffer(&ibo, 0, bytemuck::cast_slice(&item.mesh.indices));

            let model_uniform = ModelUniform {
                model: item.model_matrix().to_cols_array_2d(),
                color: [item.color[0], item.color[1], item.color[2], 1.0],
            };
            let model_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("model-uniform"),
                size: std::mem::size_of::<ModelUniform>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            queue.write_buffer(&model_buf, 0, bytemuck::bytes_of(&model_uniform));
            let model_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("model-bg"),
                layout: &pipeline.get_bind_group_layout(1),
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: model_buf.as_entire_binding(),
                }],
            });
            rpass.set_bind_group(1, &model_bg, &[]);
            rpass.set_vertex_buffer(0, vbo.slice(..));
            rpass.set_index_buffer(ibo.slice(..), wgpu::IndexFormat::Uint32);
            rpass.draw_indexed(0..item.mesh.indices.len() as u32, 0, 0..1);
        }
    }
    queue.submit(std::iter::once(pass_encoder.finish()));

    // ---- readback ----
    let bytes_per_row = width * 4;
    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("staging"),
        size: u64::from(bytes_per_row) * u64::from(height),
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut copy_encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("readback"),
    });
    copy_encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: &color_tex,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &staging,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(bytes_per_row),
                rows_per_image: Some(height),
            },
        },
        wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
    );
    queue.submit(std::iter::once(copy_encoder.finish()));

    let (tx, rx) = std::sync::mpsc::channel();
    staging
        .slice(..)
        .map_async(wgpu::MapMode::Read, move |res| {
            let _ = tx.send(res);
        });
    let _ = device.poll(wgpu::PollType::wait_indefinitely());
    let map_result: Result<Result<(), wgpu::BufferAsyncError>, _> = rx.recv();
    map_result
        .map_err(|e| format!("map_async channel closed: {e}"))?
        .map_err(|e| format!("staging map failed: {e}"))?;

    let staging_slice = staging.slice(..);
    let view = staging_slice
        .get_mapped_range()
        .map_err(|e| format!("get_mapped_range failed: {e}"))?;
    let mut img = image::RgbaImage::new(width, height);
    for (y, row) in view.chunks_exact(bytes_per_row as usize).enumerate() {
        for (x, px) in row.as_chunks::<4>().0.iter().enumerate() {
            img.put_pixel(
                x as u32,
                y as u32,
                image::Rgba([px[0], px[1], px[2], px[3]]),
            );
        }
    }
    drop(view);
    staging.unmap();

    let mut distinct = std::collections::HashSet::new();
    for px in img.pixels() {
        distinct.insert([px[0], px[1], px[2], px[3]]);
    }

    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent).map_err(|e| format!("mkdir {}: {e}", parent.display()))?;
    }
    img.save(path)
        .map_err(|e| format!("save {}: {e}", path.display()))?;

    Ok(RenderReport {
        adapter_name: info.name.clone(),
        adapter_device_type: format!("{:?}", info.device_type),
        adapter_backend: format!("{:?}", info.backend),
        width,
        height,
        png_path: path.display().to_string(),
        distinct_colors: distinct.len(),
    })
}
