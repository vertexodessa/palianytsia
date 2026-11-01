//! Video as YUV planes in GL textures, converted to RGB in a fragment
//! shader (spec 5.7 "NV12 planes to a texture"): the decode thread does no
//! colour conversion, the UI uploads Y + UV (NV12) or Y + U + V (I420) as
//! R8/RG8 textures and draws one quad through an egui paint callback.
use std::sync::{Arc, Mutex};

use decode::Planes;
use eframe::egui_glow;
use eframe::glow::{self, HasContext};

/// The newest decoded frame, shared between the decode thread and the UI.
pub struct YuvFrame {
    pub p: Planes,
    pub seq: u64,
    /// lat_us() when it came out of the decoder.
    pub t_dec: u64,
}

struct Gl {
    prog: glow::Program,
    vao: glow::VertexArray,
    tex: [glow::Texture; 3],
    /// (seq, w, h, nv12) of what the textures hold.
    loaded: (u64, usize, usize, bool),
    u_nv12: Option<glow::UniformLocation>,
    u_m: Option<glow::UniformLocation>,
    u_off: Option<glow::UniformLocation>,
}

/// GL state, created on the first paint inside the egui_glow context.
#[derive(Default, Clone)]
pub struct GlVideo(Arc<Mutex<Option<Gl>>>);

const VS: &str = r#"
out vec2 v_uv;
void main() {
    vec2 p = vec2(float(gl_VertexID & 1), float(gl_VertexID >> 1));
    v_uv = p;
    gl_Position = vec4(p.x * 2.0 - 1.0, 1.0 - p.y * 2.0, 0.0, 1.0);
}
"#;

const FS: &str = r#"
precision mediump float;
uniform sampler2D t_y;
uniform sampler2D t_u;
uniform sampler2D t_v;
uniform int u_nv12;
uniform mat3 u_m;
uniform vec3 u_off;
in vec2 v_uv;
out vec4 o_col;
void main() {
    float y = texture(t_y, v_uv).r;
    vec2 c = u_nv12 == 1 ? texture(t_u, v_uv).rg : vec2(texture(t_u, v_uv).r, texture(t_v, v_uv).r);
    vec3 rgb = u_m * (vec3(y, c) - u_off);
    o_col = vec4(clamp(rgb, 0.0, 1.0), 1.0);
}
"#;

/// YUV -> RGB, column-major (columns: Y, U, V coefficients), and the offset
/// subtracted first.
fn matrix(bt709: bool, full: bool) -> ([f32; 9], [f32; 3]) {
    let (kr, kb) = if bt709 {
        (0.2126, 0.0722)
    } else {
        (0.299, 0.114)
    };
    let kg = 1.0 - kr - kb;
    let (ys, cs, yoff) = if full {
        (1.0, 1.0, 0.0)
    } else {
        (255.0 / 219.0, 255.0 / 224.0, 16.0 / 255.0)
    };
    let rv = 2.0 * (1.0 - kr) * cs;
    let bu = 2.0 * (1.0 - kb) * cs;
    let gu = -bu * kb / kg;
    let gv = -rv * kr / kg;
    ([ys, ys, ys, 0.0, gu, bu, rv, gv, 0.0], [yoff, 0.5, 0.5])
}

unsafe fn build(gl: &glow::Context, version: &str) -> Result<Gl, String> {
    let prog = gl.create_program()?;
    let mut shaders = Vec::new();
    for (ty, src) in [(glow::VERTEX_SHADER, VS), (glow::FRAGMENT_SHADER, FS)] {
        let sh = gl.create_shader(ty)?;
        gl.shader_source(sh, &format!("{version}\n{src}"));
        gl.compile_shader(sh);
        if !gl.get_shader_compile_status(sh) {
            return Err(gl.get_shader_info_log(sh));
        }
        gl.attach_shader(prog, sh);
        shaders.push(sh);
    }
    gl.link_program(prog);
    if !gl.get_program_link_status(prog) {
        return Err(gl.get_program_info_log(prog));
    }
    for sh in shaders {
        gl.detach_shader(prog, sh);
        gl.delete_shader(sh);
    }
    let vao = gl.create_vertex_array()?;
    let mut tex = Vec::new();
    for _ in 0..3 {
        let t = gl.create_texture()?;
        gl.bind_texture(glow::TEXTURE_2D, Some(t));
        for (k, v) in [
            (glow::TEXTURE_MIN_FILTER, glow::LINEAR),
            (glow::TEXTURE_MAG_FILTER, glow::LINEAR),
            (glow::TEXTURE_WRAP_S, glow::CLAMP_TO_EDGE),
            (glow::TEXTURE_WRAP_T, glow::CLAMP_TO_EDGE),
        ] {
            gl.tex_parameter_i32(glow::TEXTURE_2D, k, v as i32);
        }
        tex.push(t);
    }
    gl.use_program(Some(prog));
    for (i, n) in ["t_y", "t_u", "t_v"].iter().enumerate() {
        let loc = gl.get_uniform_location(prog, n);
        gl.uniform_1_i32(loc.as_ref(), i as i32);
    }
    Ok(Gl {
        u_nv12: gl.get_uniform_location(prog, "u_nv12"),
        u_m: gl.get_uniform_location(prog, "u_m"),
        u_off: gl.get_uniform_location(prog, "u_off"),
        prog,
        vao,
        tex: [tex[0], tex[1], tex[2]],
        loaded: (0, 0, 0, false),
    })
}

unsafe fn upload(gl: &glow::Context, t: glow::Texture, w: usize, h: usize, fmt: u32, data: &[u8]) {
    gl.bind_texture(glow::TEXTURE_2D, Some(t));
    let internal = if fmt == glow::RG { glow::RG8 } else { glow::R8 };
    gl.tex_image_2d(
        glow::TEXTURE_2D,
        0,
        internal as i32,
        w as i32,
        h as i32,
        0,
        fmt,
        glow::UNSIGNED_BYTE,
        glow::PixelUnpackData::Slice(Some(data)),
    );
}

impl GlVideo {
    /// The paint callback drawing `frame` (the newest one) into `rect`.
    /// Uploads only when the frame's seq changed since the last paint.
    pub fn callback(
        &self,
        rect: egui::Rect,
        frame: Arc<Mutex<Option<YuvFrame>>>,
        on_upload: impl Fn(u64) + Send + Sync + 'static,
    ) -> egui::PaintCallback {
        let me = self.clone();
        egui::PaintCallback {
            rect,
            callback: Arc::new(egui_glow::CallbackFn::new(move |info, painter| {
                let gl = painter.gl();
                let mut st = me.0.lock().unwrap_or_else(|p| p.into_inner());
                if st.is_none() {
                    let v = egui_glow::ShaderVersion::get(gl);
                    match unsafe { build(gl, v.version_declaration()) } {
                        Ok(g) => *st = Some(g),
                        Err(e) => {
                            eprintln!("pal-desktop: video shader: {e}");
                            return;
                        }
                    }
                }
                let g = st.as_mut().expect("gl state");
                let fr = frame.lock().unwrap_or_else(|p| p.into_inner());
                let Some(f) = fr.as_ref() else { return };
                let p = &f.p;
                unsafe {
                    gl.pixel_store_i32(glow::UNPACK_ALIGNMENT, 1);
                    if g.loaded.0 != f.seq {
                        let (cw, ch) = (p.w.div_ceil(2), p.h.div_ceil(2));
                        upload(gl, g.tex[0], p.w, p.h, glow::RED, &p.y);
                        if p.nv12 {
                            upload(gl, g.tex[1], cw, ch, glow::RG, &p.u);
                        } else {
                            upload(gl, g.tex[1], cw, ch, glow::RED, &p.u);
                            upload(gl, g.tex[2], cw, ch, glow::RED, &p.v);
                        }
                        g.loaded = (f.seq, p.w, p.h, p.nv12);
                        on_upload(f.t_dec);
                    }
                    let vp = info.viewport_in_pixels();
                    gl.viewport(vp.left_px, vp.from_bottom_px, vp.width_px, vp.height_px);
                    gl.disable(glow::BLEND);
                    gl.use_program(Some(g.prog));
                    let (m, off) = matrix(p.bt709, p.full_range);
                    gl.uniform_matrix_3_f32_slice(g.u_m.as_ref(), false, &m);
                    gl.uniform_3_f32_slice(g.u_off.as_ref(), &off);
                    gl.uniform_1_i32(g.u_nv12.as_ref(), i32::from(p.nv12));
                    for (i, t) in g.tex.iter().enumerate() {
                        gl.active_texture(glow::TEXTURE0 + i as u32);
                        gl.bind_texture(glow::TEXTURE_2D, Some(*t));
                    }
                    gl.bind_vertex_array(Some(g.vao));
                    gl.draw_arrays(glow::TRIANGLE_STRIP, 0, 4);
                    gl.bind_vertex_array(None);
                    gl.active_texture(glow::TEXTURE0);
                }
            })),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::matrix;

    fn rgb(m: &[f32; 9], off: &[f32; 3], yuv: [f32; 3]) -> [f32; 3] {
        let v = [yuv[0] - off[0], yuv[1] - off[1], yuv[2] - off[2]];
        let mut o = [0.0; 3];
        for (r, out) in o.iter_mut().enumerate() {
            *out = m[r] * v[0] + m[3 + r] * v[1] + m[6 + r] * v[2];
        }
        o
    }

    #[test]
    fn limited_range_black_white_and_bt601_red() {
        for bt709 in [false, true] {
            let (m, off) = matrix(bt709, false);
            let b = rgb(&m, &off, [16.0 / 255.0, 0.5, 0.5]);
            let w = rgb(&m, &off, [235.0 / 255.0, 0.5, 0.5]);
            for c in 0..3 {
                assert!(b[c].abs() < 1e-4 && (w[c] - 1.0).abs() < 1e-4);
            }
        }
        // BT.601 limited red: Y 81, Cb 90, Cr 240 (ITU-R BT.601 tables).
        let (m, off) = matrix(false, false);
        let r = rgb(&m, &off, [81.0 / 255.0, 90.0 / 255.0, 240.0 / 255.0]);
        assert!((r[0] - 1.0).abs() < 0.01 && r[1].abs() < 0.01 && r[2].abs() < 0.01);
    }
}
