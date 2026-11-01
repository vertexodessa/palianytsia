// PalCard: the card devourer-sys brought up, lent to mabur's GS core as a
// maburgs::LinkCard, so WebGs runs its ChannelCore (search, boot pick,
// relocation, in-flight hop) on real hardware.
//
// Member by member it is PixelPilot's MaburCard, itself mabur's
// RadioFrontend minus what the host owns: opening the device, InitWrite,
// the RX loop, teardown and the TX frame builder (devourer-sys). The host feeds every received
// frame's fill_rx_body verdict to on_rx() from its RX thread (pal-wire's
// dot11::parse is the same function) and forwards the bodies itself, with
// the rx_channel stamp on_rx() returns. Threads: the RX thread (on_rx),
// the core thread (send_control), the channel core's scout thread (retune,
// set_width, the energy and NHM reads). Device calls other than RX are
// serialised by ctl_: devourer's control calls are single-control-thread,
// and a FastRetune must never overlap a send. on_rx takes no lock, so the
// RX callback can never wait on a thread that is inside a USB transfer.
#include <atomic>
#include <cstdio>
#include <mutex>

#include "AdapterCaps.h"
#include "IRtlRadio.h"
#include "dot11.h"   // RxVerdict only
#include "link_card.h"
#include "mabur/ht40.h"
#include "mabur/node.h"
#include "own_air.h"
#include "pal_ref.h"

namespace {

maburgs::ScoutEnergy to_scout(const RxEnergy& e) {
  maburgs::ScoutEnergy out;
  out.fa_valid = e.valid_fa;
  out.cca_ofdm = e.cca_ofdm;
  out.fa_ofdm = e.fa_ofdm;
  out.igi_valid = e.valid_igi;
  out.igi = e.igi;
  out.nhm_valid = e.valid_nhm;
  out.floor_valid = e.valid_noise_floor;
  out.floor_dbm = e.abs_noise_floor_dbm;
  return out;
}

class PalCard final : public maburgs::LinkCard {
 public:
  PalCard(IRtlRadio& dev, uint8_t ch, uint8_t width, void* user, pal_card_send_cb send)
      : dev_(dev), user_(user), send_(send), channel_(ch), width_(width), rx_channel_(ch) {
    // RadioFrontend::start_ after InitWrite.
    std::lock_guard<std::mutex> g(ctl_);
    const devourer::AdapterCaps ac = dev_.GetAdapterCaps();
    const RxEnergy e = dev_.GetRxEnergy(/*with_nhm=*/true);
    caps_.valid = ac.supported;
    caps_.chip = (ac.chip_name && *ac.chip_name) ? ac.chip_name : "?";
    const char* gen = devourer::generation_name(ac.generation);
    caps_.gen = (gen && *gen) ? gen : "?";
    caps_.tx_chains = ac.tx_chains;
    caps_.rx_chains = ac.rx_chains;
    caps_.bw_mask = ac.bw_mask;
    caps_.tune5g_lo = ac.tune_5g.valid ? ac.tune_5g.min_mhz : 0;
    caps_.tune5g_hi = ac.tune_5g.valid ? ac.tune_5g.max_mhz : 0;
    caps_.fast_retune = ac.fastretune_ok;
    caps_.fa_ok = e.valid_fa;
    caps_.igi_ok = e.valid_igi;
    caps_.nhm_ok = e.valid_nhm;
    caps_.floor_ok = e.valid_noise_floor;
    ready_.store(true, std::memory_order_release);
  }

  void mark_down() { ready_.store(false, std::memory_order_release); }

  // RadioFrontend::on_packet's counters. Returns -1 for a frame that is not
  // a body, else its rx_channel (0 while a retune blinds the stamp).
  int on_rx(maburgs::RxVerdict v, size_t len, const pal_card_rx& a) {
    rx_frames_.fetch_add(1, std::memory_order_relaxed);
    if (v == maburgs::RxVerdict::Short) return -1;
    if (v == maburgs::RxVerdict::Foreign) {
      foreign_.fetch_add(1, std::memory_order_relaxed);
      return -1;
    }
    if (!a.crc_err) {
      own_.fetch_add(1, std::memory_order_relaxed);
      const uint16_t r = a.data_rate;
      const uint8_t mcs = (r >= 0x0C && r <= 0x13)   ? static_cast<uint8_t>(r - 0x0C)
                          : (r >= 0x80 && r <= 0x87) ? static_cast<uint8_t>(r - 0x80)
                                                     : 255;
      own_air_.on_frame(len, mcs, a.physt, a.bw40 ? 40 : 20, a.stbc != 0, a.sgi != 0);
      own_air_us_.store(own_air_.total_us(), std::memory_order_relaxed);
    }
    return rx_channel_.load(std::memory_order_acquire);
  }

  // maburgs::LinkCard
  bool open_and_start() override { return ready(); }
  void stop() override {}
  bool ready() const override { return ready_.load(std::memory_order_acquire); }
  bool alive() const override { return ready(); }
  maburgs::CardCaps caps() const override { return caps_; }
  uint64_t rx_frames() const override { return rx_frames_.load(std::memory_order_relaxed); }
  uint64_t foreign() const override { return foreign_.load(std::memory_order_relaxed); }
  bool send_control(const std::vector<uint8_t>& body) override {
    if (!ready()) {
      tx_fail_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    bool ok = false;
    {
      std::lock_guard<std::mutex> g(ctl_);
      ok = send_(user_, body.data(), body.size()) != 0;
    }
    (ok ? tx_frames_ : tx_fail_).fetch_add(1, std::memory_order_relaxed);
    return ok;
  }
  uint64_t tx_frames() const override { return tx_frames_.load(std::memory_order_relaxed); }
  uint64_t tx_fail() const override { return tx_fail_.load(std::memory_order_relaxed); }
  bool set_width(uint8_t ch, uint8_t w) override {
    if (w == 40 && mabur::ht40_offset(ch) == 0) return false;
    const uint8_t was = width();
    width_.store(w, std::memory_order_release);
    if (!ready()) return false;
    std::lock_guard<std::mutex> g(ctl_);
    rx_channel_.store(0, std::memory_order_release);
    dev_.SetMonitorChannel(w == 40 ? SelectedChannel{ch, mabur::ht40_offset(ch), CHANNEL_WIDTH_40}
                                   : SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
    channel_.store(ch, std::memory_order_release);
    rx_channel_.store(ch, std::memory_order_release);
    std::fprintf(stderr, "maburgs radio: card 0 width %u -> %u MHz on ch %u\n",
                 static_cast<unsigned>(was), static_cast<unsigned>(w), static_cast<unsigned>(ch));
    return true;
  }
  uint8_t channel() const override { return channel_.load(std::memory_order_acquire); }
  uint8_t width() const override { return width_.load(std::memory_order_acquire); }
  int tuned_central() override {
    if (!ready()) return -1;
    std::lock_guard<std::mutex> g(ctl_);
    return dev_.ReadTunedCentral();
  }
  bool can_scout() const override { return true; }

  // maburgs::ScoutRadio
  bool retune(uint8_t ch) override {
    if (!ready()) return false;
    std::lock_guard<std::mutex> g(ctl_);
    // Blind the body stamp for the move (RadioFrontend::retune): a frame
    // picked up during it reads 0 = unknown, not the new channel.
    rx_channel_.store(0, std::memory_order_release);
    dev_.FastRetune(ch, /*cache_rf=*/true);
    channel_.store(ch, std::memory_order_release);
    rx_channel_.store(ch, std::memory_order_release);
    return true;
  }
  bool retune_width(uint8_t ch, uint8_t w) override { return set_width(ch, w); }
  maburgs::ScoutEnergy read_energy(bool with_nhm) override {
    if (!ready()) return {};
    std::lock_guard<std::mutex> g(ctl_);
    return to_scout(dev_.GetRxEnergy(with_nhm));
  }
  maburgs::ScoutEnergy read_energy_scout() override {
    if (!ready()) return {};
    std::lock_guard<std::mutex> g(ctl_);
    return to_scout(dev_.GetRxEnergyScout());
  }
  maburgs::ScoutFrames frames() const override {
    return maburgs::ScoutFrames{own_.load(std::memory_order_relaxed),
                                foreign_.load(std::memory_order_relaxed),
                                own_air_us_.load(std::memory_order_relaxed)};
  }
  bool arm_nhm_busy(uint16_t period_4us) override {
    if (!ready()) return false;
    std::lock_guard<std::mutex> g(ctl_);
    return dev_.ArmNhmBusy(period_4us);
  }
  maburgs::NhmBusyRead read_nhm_busy() override {
    maburgs::NhmBusyRead out;
    if (!ready()) return out;
    std::lock_guard<std::mutex> g(ctl_);
    const NhmBusy b = dev_.ReadNhmBusy();
    out.valid = b.valid;
    for (int i = 0; i < 12; ++i) out.buckets[i] = b.buckets[i];
    out.duration = b.duration;
    out.period = b.period;
    return out;
  }

 private:
  IRtlRadio& dev_;
  void* user_;
  pal_card_send_cb send_;
  maburgs::CardCaps caps_;
  std::mutex ctl_;
  std::atomic<bool> ready_{false};
  std::atomic<uint64_t> rx_frames_{0}, foreign_{0}, own_{0}, own_air_us_{0};
  std::atomic<uint64_t> tx_frames_{0}, tx_fail_{0};
  maburgs::OwnAirAcc own_air_;   // RX thread only
  std::atomic<uint8_t> channel_, width_, rx_channel_;
};

}  // namespace

struct pal_card {
  PalCard card;
  pal_card(IRtlRadio& d, uint8_t ch, uint8_t w, void* u, pal_card_send_cb s)
      : card(d, ch, w, u, s) {}
};

maburgs::LinkCard* pal_card_link(pal_card* c) { return c ? &c->card : nullptr; }

extern "C" {

pal_card* pal_card_new(void* radio, uint8_t channel, uint8_t width_mhz, void* user,
                       pal_card_send_cb send, char* err, size_t err_cap) {
  auto set = [&](const char* s) {
    if (err && err_cap) std::snprintf(err, err_cap, "%s", s);
  };
  if (!radio || !send) {
    set("radio and send are required");
    return nullptr;
  }
  try {
    return new pal_card(*static_cast<IRtlRadio*>(radio), channel, width_mhz == 40 ? 40 : 20,
                        user, send);
  } catch (const std::exception& e) {
    set(e.what());
  } catch (...) {
    set("unknown C++ exception");
  }
  return nullptr;
}

void pal_card_free(pal_card* c) {
  if (c) c->card.mark_down();
  delete c;
}

int pal_card_on_rx(pal_card* c, int verdict, size_t len, const pal_card_rx* a) {
  if (!c || !a || verdict < 0 || verdict > 2) return -1;
  const maburgs::RxVerdict v = verdict == 0   ? maburgs::RxVerdict::Body
                               : verdict == 1 ? maburgs::RxVerdict::Short
                                              : maburgs::RxVerdict::Foreign;
  try {
    return c->card.on_rx(v, len, *a);
  } catch (...) {
    return -1;
  }
}

uint8_t pal_card_channel(const pal_card* c) { return c ? c->card.channel() : 0; }

}  // extern "C"
