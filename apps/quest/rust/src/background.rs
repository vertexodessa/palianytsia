//! The room behind the window (XR_FB_passthrough). Exists only while the
//! window mode is on: created running and dropped on the way out, so goggle
//! mode pays nothing for the cameras. Created running because openxr 0.19's
//! `Passthrough::start` calls xrPassthroughPauseFB and its handle is private.
use openxr as xr;
use openxr::sys;

pub struct Background {
    // Declared first: the layer is destroyed before its feature.
    layer: xr::PassthroughLayer,
    _feature: xr::Passthrough,
}

impl Background {
    pub fn new(session: &xr::Session<xr::OpenGlEs>) -> Result<Self, String> {
        let running = xr::PassthroughFlagsFB::IS_RUNNING_AT_CREATION;
        let feature = session
            .create_passthrough(running)
            .map_err(|e| format!("xrCreatePassthroughFB: {e}"))?;
        let layer = session
            .create_passthrough_layer(
                &feature,
                running,
                xr::PassthroughLayerPurposeFB::RECONSTRUCTION,
            )
            .map_err(|e| format!("xrCreatePassthroughLayerFB: {e}"))?;
        Ok(Self {
            layer,
            _feature: feature,
        })
    }

    /// The composition layer: submit it below the quads.
    pub fn layer(&self) -> sys::CompositionLayerPassthroughFB {
        sys::CompositionLayerPassthroughFB {
            ty: sys::CompositionLayerPassthroughFB::TYPE,
            next: std::ptr::null(),
            flags: sys::CompositionLayerFlags::BLEND_TEXTURE_SOURCE_ALPHA,
            space: sys::Space::NULL,
            layer_handle: *self.layer.inner(),
        }
    }
}
