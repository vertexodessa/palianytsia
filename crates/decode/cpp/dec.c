// libavcodec HEVC decoder for pal, latency settings from spec 5.7:
// AV_CODEC_FLAG_LOW_DELAY, slice threads only (frame threads add a frame of
// delay each), one AU per packet, keep only the newest output frame. Never
// flushed on loss: GDR streams have no IDR to resync on.
//
// Optional hardware decode through libavcodec's hwaccel (vaapi, vdpau,
// cuda, ...: av_hwdevice by name). Frames stay on the GPU; only the frame
// that gets shown is transferred down (NV12 for 8-bit 4:2:0), on rgba().
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct pal_dec {
  AVCodecContext* ctx;
  AVPacket* pkt;
  AVFrame* frame;   // newest decoded frame
  AVFrame* tmp;
  int have;
  struct SwsContext* sws;
  int frames, errors;
  AVBufferRef* hw_dev;
  enum AVPixelFormat hw_fmt;   // AV_PIX_FMT_NONE without hwaccel
  AVFrame* sw;                 // transfer target for a hw frame
  AVFrame* i420;               // NV12 deinterleaved for swscale's fast path
  struct SwsContext* sws_nv;
  int hw_frames;               // frames that came out as hw surfaces
} pal_dec;

static enum AVPixelFormat pick_hw(AVCodecContext* ctx, const enum AVPixelFormat* fmts) {
  const pal_dec* d = ctx->opaque;
  for (const enum AVPixelFormat* p = fmts; *p != AV_PIX_FMT_NONE; p++)
    if (*p == d->hw_fmt) return *p;
  // The hwaccel refused this stream: libavcodec's software fallback.
  return avcodec_default_get_format(ctx, fmts);
}

static void set_err(char* err, size_t cap, const char* msg) {
  if (err && cap) snprintf(err, cap, "%s", msg);
}

// hw: an av_hwdevice type name ("vaapi", "vdpau", "cuda", ...), or NULL /
// "" for software. Fails (NULL + err) when that device cannot be opened, so
// the caller decides about falling back.
pal_dec* pal_dec_new_hw(int threads, const char* hw, char* err, size_t cap) {
  const AVCodec* c = avcodec_find_decoder(AV_CODEC_ID_HEVC);
  if (!c) { set_err(err, cap, "no HEVC decoder in libavcodec"); return NULL; }
  pal_dec* d = calloc(1, sizeof *d);
  if (!d) { set_err(err, cap, "oom"); return NULL; }
  d->ctx = avcodec_alloc_context3(c);
  d->pkt = av_packet_alloc();
  d->frame = av_frame_alloc();
  d->tmp = av_frame_alloc();
  if (!d->ctx || !d->pkt || !d->frame || !d->tmp) { set_err(err, cap, "oom"); goto fail; }
  d->ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
  d->ctx->flags2 |= AV_CODEC_FLAG2_FAST;
  d->ctx->thread_type = FF_THREAD_SLICE;
  d->ctx->thread_count = threads > 0 ? threads : 0;
  d->hw_fmt = AV_PIX_FMT_NONE;
  if (hw && *hw) {
    const enum AVHWDeviceType t = av_hwdevice_find_type_by_name(hw);
    if (t == AV_HWDEVICE_TYPE_NONE) { set_err(err, cap, "unknown hwaccel name"); goto fail; }
    for (int i = 0;; i++) {
      const AVCodecHWConfig* hc = avcodec_get_hw_config(c, i);
      if (!hc) break;
      if ((hc->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) && hc->device_type == t) {
        d->hw_fmt = hc->pix_fmt;
        break;
      }
    }
    if (d->hw_fmt == AV_PIX_FMT_NONE) { set_err(err, cap, "HEVC decoder has no such hwaccel"); goto fail; }
    int r = av_hwdevice_ctx_create(&d->hw_dev, t, NULL, NULL, 0);
    if (r < 0) {
      char e[AV_ERROR_MAX_STRING_SIZE];
      av_strerror(r, e, sizeof e);
      if (err && cap) snprintf(err, cap, "%s device: %s", hw, e);
      goto fail;
    }
    d->ctx->hw_device_ctx = av_buffer_ref(d->hw_dev);
    d->ctx->opaque = d;
    d->ctx->get_format = pick_hw;
    d->sw = av_frame_alloc();
    d->i420 = av_frame_alloc();
    if (!d->ctx->hw_device_ctx || !d->sw || !d->i420) { set_err(err, cap, "oom"); goto fail; }
  }
  if (avcodec_open2(d->ctx, c, NULL) < 0) { set_err(err, cap, "avcodec_open2 failed"); goto fail; }
  return d;
fail:
  avcodec_free_context(&d->ctx);
  av_buffer_unref(&d->hw_dev);
  av_frame_free(&d->sw);
  av_frame_free(&d->i420);
  av_packet_free(&d->pkt);
  av_frame_free(&d->frame);
  av_frame_free(&d->tmp);
  free(d);
  return NULL;
}

void pal_dec_free(pal_dec* d) {
  if (!d) return;
  sws_freeContext(d->sws);
  sws_freeContext(d->sws_nv);
  avcodec_free_context(&d->ctx);
  av_buffer_unref(&d->hw_dev);
  av_frame_free(&d->sw);
  av_frame_free(&d->i420);
  av_packet_free(&d->pkt);
  av_frame_free(&d->frame);
  av_frame_free(&d->tmp);
  free(d);
}

// One Annex-B AU in; drains every ready frame, keeping the newest.
// Returns frames produced by this call, or a negative AVERROR on a send error
// (counted; the decoder keeps going on the next AU).
int pal_dec_send(pal_dec* d, const uint8_t* au, size_t n) {
  if (av_new_packet(d->pkt, (int)n) < 0) return AVERROR(ENOMEM);
  memcpy(d->pkt->data, au, n);
  int r = avcodec_send_packet(d->ctx, d->pkt);
  av_packet_unref(d->pkt);
  if (r < 0 && r != AVERROR(EAGAIN)) { d->errors++; return r; }
  int got = 0;
  for (;;) {
    r = avcodec_receive_frame(d->ctx, d->tmp);
    if (r == AVERROR(EAGAIN) || r == AVERROR_EOF) break;
    if (r < 0) { d->errors++; break; }
    av_frame_unref(d->frame);
    av_frame_move_ref(d->frame, d->tmp);
    d->have = 1;
    d->frames++;
    if (d->frame->format == d->hw_fmt) d->hw_frames++;
    got++;
  }
  return got;
}

// Size of the newest frame; 0 when none yet.
int pal_dec_size(const pal_dec* d, int* w, int* h) {
  if (!d->have) return 0;
  *w = d->frame->width;
  *h = d->frame->height;
  return 1;
}

// Newest frame as tightly packed RGBA (w*h*4 bytes); 0 ok, -1 none or short buffer.
int pal_dec_rgba(pal_dec* d, uint8_t* out, size_t cap) {
  if (!d->have) return -1;
  const AVFrame* f = d->frame;
  if (f->format == d->hw_fmt) {
    av_frame_unref(d->sw);
    if (av_hwframe_transfer_data(d->sw, f, 0) < 0) return -1;
    f = d->sw;
    // swscale has no fast NV12->RGBA path (6.6 ms per 1080p frame here,
    // generic scaler); NV12->I420 is a chroma deinterleave, and I420->RGBA
    // takes the SIMD yuv2rgb path (1.2 ms).
    if (f->format == AV_PIX_FMT_NV12) {
      const AVFrame* nv = f;
      if (d->i420->width != nv->width || d->i420->height != nv->height) {
        av_frame_unref(d->i420);
        d->i420->format = AV_PIX_FMT_YUV420P;
        d->i420->width = nv->width;
        d->i420->height = nv->height;
        if (av_frame_get_buffer(d->i420, 0) < 0) return -1;
      }
      d->sws_nv = sws_getCachedContext(d->sws_nv, nv->width, nv->height, AV_PIX_FMT_NV12,
                                       nv->width, nv->height, AV_PIX_FMT_YUV420P, SWS_POINT,
                                       NULL, NULL, NULL);
      if (!d->sws_nv) return -1;
      sws_scale(d->sws_nv, (const uint8_t* const*)nv->data, nv->linesize, 0, nv->height,
                d->i420->data, d->i420->linesize);
      f = d->i420;
    }
  }
  const int w = f->width, h = f->height;
  if (cap < (size_t)w * (size_t)h * 4) return -1;
  d->sws = sws_getCachedContext(d->sws, w, h, (enum AVPixelFormat)f->format, w, h,
                                AV_PIX_FMT_RGBA, SWS_FAST_BILINEAR, NULL, NULL, NULL);
  if (!d->sws) return -1;
  uint8_t* dst[4] = {out, NULL, NULL, NULL};
  int stride[4] = {w * 4, 0, 0, 0};
  sws_scale(d->sws, (const uint8_t* const*)f->data, f->linesize, 0, h, dst, stride);
  return 0;
}

void pal_dec_counters(const pal_dec* d, int* frames, int* errors) {
  *frames = d->frames;
  *errors = d->errors;
}

int pal_dec_hw_frames(const pal_dec* d) { return d->hw_frames; }
