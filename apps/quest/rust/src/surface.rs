//! An egui surface the compositor shows as a quad: a GLES swapchain, the
//! framebuffer that targets its images, and an egui context with its own
//! painter. It knows nothing of what is drawn: the caller hands `draw` the
//! pointer events and a closure. Between draws the compositor keeps showing
//! the last released image, so a surface is redrawn only when its content
//! changes.
use std::num::NonZeroU32;
use std::sync::Arc;

use glow::HasContext;
use openxr as xr;

pub struct Surface {
    gl: Arc<glow::Context>,
    // One painter per surface: every egui context numbers its font atlas
    // TextureId 0, and a painter keys its textures by that id.
    painter: egui_glow::Painter,
    ctx: egui::Context,
    fbo: glow::Framebuffer,
    pub sc: xr::Swapchain<xr::OpenGlEs>,
    images: Vec<u32>,
    /// A released image exists: the layer can be submitted.
    pub ready: bool,
    /// Size in pixels.
    pub w: u32,
    pub h: u32,
    /// Pixels per egui point.
    ppp: f32,
}

/// What a draw tells the caller.
pub struct Drawn {
    /// egui is using the pointer: it is over a widget or dragging one.
    #[allow(dead_code)] // read from the menu commit on
    pub used_pointer: bool,
}

impl Surface {
    pub fn new(
        session: &xr::Session<xr::OpenGlEs>,
        gl: Arc<glow::Context>,
        w: u32,
        h: u32,
        ppp: f32,
    ) -> Result<Self, String> {
        let sc = session
            .create_swapchain(&xr::SwapchainCreateInfo {
                create_flags: xr::SwapchainCreateFlags::EMPTY,
                usage_flags: xr::SwapchainUsageFlags::COLOR_ATTACHMENT
                    | xr::SwapchainUsageFlags::SAMPLED,
                format: glow::RGBA8,
                sample_count: 1,
                width: w,
                height: h,
                face_count: 1,
                array_size: 1,
                mip_count: 1,
            })
            .map_err(|e| format!("surface swapchain {w}x{h}: {e}"))?;
        let images = sc.enumerate_images().map_err(|e| e.to_string())?;
        let painter = egui_glow::Painter::new(gl.clone(), "", None, false)
            .map_err(|e| format!("surface painter: {e}"))?;
        // SAFETY: the XR session's GLES context is current on this thread.
        let fbo = unsafe { gl.create_framebuffer() }?;
        let ctx = egui::Context::default();
        // A held trigger is still a click: a ray press is slower than a
        // tap (egui's 0.8 s is made for touch long-press).
        ctx.options_mut(|o| o.input_options.max_click_duration = 3.0);
        Ok(Self {
            gl,
            painter,
            ctx,
            fbo,
            sc,
            images,
            ready: false,
            w,
            h,
            ppp,
        })
    }

    /// The surface's size in egui points.
    pub fn points(&self) -> egui::Vec2 {
        egui::vec2(self.w as f32 / self.ppp, self.h as f32 / self.ppp)
    }

    /// One egui pass into the next swapchain image.
    pub fn draw(
        &mut self,
        events: Vec<egui::Event>,
        run: impl FnMut(&egui::Context),
    ) -> Result<Drawn, String> {
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, self.points())),
            events,
            ..Default::default()
        };
        self.ctx.set_pixels_per_point(self.ppp);
        let out = self.ctx.run(input, run);
        let used_pointer = self.ctx.is_using_pointer() || self.ctx.wants_pointer_input();
        let prims = self.ctx.tessellate(out.shapes, out.pixels_per_point);
        let idx = self.sc.acquire_image().map_err(|e| e.to_string())?;
        self.sc
            .wait_image(xr::Duration::INFINITE)
            .map_err(|e| e.to_string())?;
        let tex = NonZeroU32::new(self.images[idx as usize]).map(glow::NativeTexture);
        // SAFETY: the session's GLES context is current on this thread and
        // `tex` is a live swapchain image.
        unsafe {
            self.gl.bind_framebuffer(glow::FRAMEBUFFER, Some(self.fbo));
            self.gl.framebuffer_texture_2d(
                glow::FRAMEBUFFER,
                glow::COLOR_ATTACHMENT0,
                glow::TEXTURE_2D,
                tex,
                0,
            );
        }
        let size = [self.w, self.h];
        self.painter.clear(size, [0.0, 0.0, 0.0, 0.0]);
        self.painter.paint_and_update_textures(
            size,
            out.pixels_per_point,
            &prims,
            &out.textures_delta,
        );
        // SAFETY: as above.
        unsafe {
            self.gl.bind_framebuffer(glow::FRAMEBUFFER, None);
        }
        self.sc.release_image().map_err(|e| e.to_string())?;
        self.ready = true;
        Ok(Drawn { used_pointer })
    }
}

impl Drop for Surface {
    fn drop(&mut self) {
        self.painter.destroy();
        // SAFETY: as in draw().
        unsafe { self.gl.delete_framebuffer(self.fbo) };
    }
}
