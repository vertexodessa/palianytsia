//! XR_META_performance_metrics: the runtime's own counters (compositor and
//! app frame times, dropped frames, GPU and CPU load), the part of the
//! glass-to-glass chain after our decoder that nothing else of ours sees.
//! Logged next to the frame rate; the paths are whatever this runtime
//! offers, enumerated, not a list fixed here.
use std::ptr;

use openxr as xr;
use openxr::sys;

pub struct PerfMetrics {
    raw: xr::raw::PerformanceMetricsMETA,
    session: sys::Session,
    counters: Vec<(String, sys::Path)>,
}

impl PerfMetrics {
    /// None when the extension was not enabled or the runtime refuses.
    pub fn new<G: xr::Graphics>(inst: &xr::Instance, session: &xr::Session<G>) -> Option<Self> {
        let raw = *inst.exts().meta_performance_metrics.as_ref()?;
        let mut n = 0u32;
        // SAFETY: the two-call idiom on a live instance.
        let paths = unsafe {
            let ok = (raw.enumerate_performance_metrics_counter_paths)(
                inst.as_raw(),
                0,
                &mut n,
                ptr::null_mut(),
            );
            if ok != sys::Result::SUCCESS {
                log::warn!("perf metrics: enumerate {ok:?}");
                return None;
            }
            let mut v = vec![sys::Path::NULL; n as usize];
            let ok = (raw.enumerate_performance_metrics_counter_paths)(
                inst.as_raw(),
                n,
                &mut n,
                v.as_mut_ptr(),
            );
            if ok != sys::Result::SUCCESS {
                log::warn!("perf metrics: enumerate {ok:?}");
                return None;
            }
            v.truncate(n as usize);
            v
        };
        let state = sys::PerformanceMetricsStateMETA {
            ty: sys::PerformanceMetricsStateMETA::TYPE,
            next: ptr::null(),
            enabled: sys::TRUE,
        };
        // SAFETY: a live session and a filled state struct.
        let ok = unsafe { (raw.set_performance_metrics_state)(session.as_raw(), &state) };
        if ok != sys::Result::SUCCESS {
            log::warn!("perf metrics: enable {ok:?}");
            return None;
        }
        let counters: Vec<_> = paths
            .into_iter()
            .map(|p| {
                let name = inst.path_to_string(p).unwrap_or_else(|_| format!("{p:?}"));
                // The shared prefix only lengthens every log line.
                let short = name
                    .strip_prefix("/perfmetrics_meta/")
                    .unwrap_or(&name)
                    .to_string();
                (short, p)
            })
            .collect();
        log::info!(
            "perf metrics: {}",
            counters
                .iter()
                .map(|(n, _)| n.as_str())
                .collect::<Vec<_>>()
                .join(" ")
        );
        Some(Self {
            raw,
            session: session.as_raw(),
            counters,
        })
    }

    /// `name=value[unit]` for every counter with a valid value.
    pub fn sample(&self) -> String {
        let mut out = Vec::with_capacity(self.counters.len());
        for (name, path) in &self.counters {
            let mut c = sys::PerformanceMetricsCounterMETA {
                ty: sys::PerformanceMetricsCounterMETA::TYPE,
                next: ptr::null(),
                counter_flags: sys::PerformanceMetricsCounterFlagsMETA::EMPTY,
                counter_unit: sys::PerformanceMetricsCounterUnitMETA::GENERIC,
                uint_value: 0,
                float_value: 0.0,
            };
            // SAFETY: the session outlives this struct (both live in run).
            let ok = unsafe {
                (self.raw.query_performance_metrics_counter)(self.session, *path, &mut c)
            };
            if ok != sys::Result::SUCCESS {
                continue;
            }
            let unit = match c.counter_unit {
                sys::PerformanceMetricsCounterUnitMETA::PERCENTAGE => "%",
                sys::PerformanceMetricsCounterUnitMETA::MILLISECONDS => "ms",
                sys::PerformanceMetricsCounterUnitMETA::BYTES => "B",
                sys::PerformanceMetricsCounterUnitMETA::HERTZ => "Hz",
                _ => "",
            };
            let f = c.counter_flags;
            if f.contains(sys::PerformanceMetricsCounterFlagsMETA::FLOAT_VALUE_VALID) {
                out.push(format!("{name}={:.2}{unit}", c.float_value));
            } else if f.contains(sys::PerformanceMetricsCounterFlagsMETA::UINT_VALUE_VALID) {
                out.push(format!("{name}={}{unit}", c.uint_value));
            }
        }
        out.join(" ")
    }
}
