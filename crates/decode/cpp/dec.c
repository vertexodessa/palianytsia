// libavcodec HEVC decoder for pal, latency settings from spec 5.7:
// AV_CODEC_FLAG_LOW_DELAY, slice threads only (frame threads add a frame of
// delay each), one AU per packet, keep only the newest output frame. Never
// flushed on loss: GDR streams have no IDR to resync on.
#include <libavcodec/avcodec.h>
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
} pal_dec;

static void set_err(char* err, size_t cap, const char* msg) {
  if (err && cap) snprintf(err, cap, "%s", msg);
}

pal_dec* pal_dec_new(int threads, char* err, size_t cap) {
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
  if (avcodec_open2(d->ctx, c, NULL) < 0) { set_err(err, cap, "avcodec_open2 failed"); goto fail; }
  return d;
fail:
  avcodec_free_context(&d->ctx);
  av_packet_free(&d->pkt);
  av_frame_free(&d->frame);
  av_frame_free(&d->tmp);
  free(d);
  return NULL;
}

void pal_dec_free(pal_dec* d) {
  if (!d) return;
  sws_freeContext(d->sws);
  avcodec_free_context(&d->ctx);
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
  const int w = d->frame->width, h = d->frame->height;
  if (cap < (size_t)w * (size_t)h * 4) return -1;
  d->sws = sws_getCachedContext(d->sws, w, h, (enum AVPixelFormat)d->frame->format, w, h,
                                AV_PIX_FMT_RGBA, SWS_FAST_BILINEAR, NULL, NULL, NULL);
  if (!d->sws) return -1;
  uint8_t* dst[4] = {out, NULL, NULL, NULL};
  int stride[4] = {w * 4, 0, 0, 0};
  sws_scale(d->sws, (const uint8_t* const*)d->frame->data, d->frame->linesize, 0, h, dst, stride);
  return 0;
}

void pal_dec_counters(const pal_dec* d, int* frames, int* errors) {
  *frames = d->frames;
  *errors = d->errors;
}
