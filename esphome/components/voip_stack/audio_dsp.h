#pragma once

// Elaborazione audio leggera (solo software, nessuna dipendenza ESP-IDF):
//  - Biquad RBJ (passa-alto e picco di presenza)
//  - RxDsp: gain + passa-alto + presenza + limitatore morbido sul flusso
//    PCM ricevuto, prima dell'altoparlante
//  - NoiseMeter: stima del rumore ambientale (dBFS RMS, percentile basso)
//
// Tutto e' spento di default: senza configurazione il comportamento di
// voip_stack resta identico a prima.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome {
namespace voip_dsp {

static constexpr float kPi = 3.14159265358979f;

static inline int16_t clamp_i16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32767.0f) return -32767;
  return static_cast<int16_t>(std::lrintf(v));
}

// Biquad in forma trasposta II (float). Coefficienti dal "Audio EQ Cookbook" di
// R. Bristow-Johnson.
struct Biquad {
  float b0{1.0f}, b1{0.0f}, b2{0.0f}, a1{0.0f}, a2{0.0f};
  float z1{0.0f}, z2{0.0f};

  void reset() {
    this->z1 = 0.0f;
    this->z2 = 0.0f;
  }

  inline float process(float x) {
    const float y = this->b0 * x + this->z1;
    this->z1 = this->b1 * x - this->a1 * y + this->z2;
    this->z2 = this->b2 * x - this->a2 * y;
    return y;
  }

  void set_identity() {
    this->b0 = 1.0f;
    this->b1 = this->b2 = this->a1 = this->a2 = 0.0f;
    this->reset();
  }

  // Passa-alto del 2o ordine (12 dB/ottava). Frequenze non valide -> identita'.
  void set_highpass(float fs, float f0, float q = 0.70710678f) {
    if (!(fs > 0.0f) || !(f0 > 0.0f) || f0 >= 0.45f * fs) {
      this->set_identity();
      return;
    }
    const float w0 = 2.0f * kPi * f0 / fs;
    const float cw = std::cos(w0);
    const float alpha = std::sin(w0) / (2.0f * q);
    const float a0 = 1.0f + alpha;
    this->b0 = ((1.0f + cw) * 0.5f) / a0;
    this->b1 = (-(1.0f + cw)) / a0;
    this->b2 = ((1.0f + cw) * 0.5f) / a0;
    this->a1 = (-2.0f * cw) / a0;
    this->a2 = (1.0f - alpha) / a0;
    this->reset();
  }

  // Filtro "peaking" (campana): esalta/attenua attorno a f0.
  void set_peaking(float fs, float f0, float q, float gain_db) {
    if (!(fs > 0.0f) || !(f0 > 0.0f) || f0 >= 0.45f * fs || gain_db == 0.0f) {
      this->set_identity();
      return;
    }
    const float A = std::pow(10.0f, gain_db / 40.0f);
    const float w0 = 2.0f * kPi * f0 / fs;
    const float cw = std::cos(w0);
    const float alpha = std::sin(w0) / (2.0f * q);
    const float a0 = 1.0f + alpha / A;
    this->b0 = (1.0f + alpha * A) / a0;
    this->b1 = (-2.0f * cw) / a0;
    this->b2 = (1.0f - alpha * A) / a0;
    this->a1 = (-2.0f * cw) / a0;
    this->a2 = (1.0f - alpha / A) / a0;
    this->reset();
  }
};

struct RxDspConfig {
  float highpass_hz{0.0f};        // 0 = spento
  float presence_hz{2500.0f};
  float presence_gain_db{0.0f};   // 0 = spento
  float gain_db{0.0f};            // guadagno digitale prima del limitatore
  bool limiter{true};
  float limiter_dbfs{-2.0f};      // soglia oltre la quale il segnale e' compresso
};

class RxDsp {
 public:
  void set_config(const RxDspConfig &cfg) {
    this->cfg_ = cfg;
    this->rate_ = 0;  // forza il ricalcolo dei coefficienti
    this->gain_lin_ = std::pow(10.0f, cfg.gain_db / 20.0f);
    this->limit_ = std::pow(10.0f, std::min(0.0f, cfg.limiter_dbfs) / 20.0f);
    if (this->limit_ > 0.999f) this->limit_ = 0.999f;
  }

  bool enabled() const {
    return this->cfg_.highpass_hz > 0.0f || this->cfg_.presence_gain_db != 0.0f || this->cfg_.gain_db != 0.0f;
  }

  void reset() {
    this->hp_.reset();
    this->pres_.reset();
  }

  // Ricalcola i coefficienti se la frequenza di campionamento e' cambiata.
  void prepare(uint32_t sample_rate) {
    if (sample_rate == this->rate_) return;
    this->rate_ = sample_rate;
    const float fs = static_cast<float>(sample_rate);
    this->hp_on_ = this->cfg_.highpass_hz > 0.0f;
    this->pres_on_ = this->cfg_.presence_gain_db != 0.0f;
    this->hp_.set_highpass(fs, this->cfg_.highpass_hz);
    this->pres_.set_peaking(fs, this->cfg_.presence_hz, 0.9f, this->cfg_.presence_gain_db);
  }

  // Elabora in place un blocco di campioni int16 mono.
  void process(int16_t *s, size_t n) {
    constexpr float kInv = 1.0f / 32768.0f;
    const float limit = this->limit_;
    const float knee = 1.0f - limit;
    for (size_t i = 0; i < n; i++) {
      float x = static_cast<float>(s[i]) * kInv;
      if (this->hp_on_) x = this->hp_.process(x);
      if (this->pres_on_) x = this->pres_.process(x);
      x *= this->gain_lin_;
      if (this->cfg_.limiter) {
        const float a = std::fabs(x);
        if (a > limit) {
          // Ginocchio morbido: sopra la soglia il segnale tende a 1.0 senza
          // mai superarlo (niente clipping digitale netto).
          x = std::copysign(limit + knee * std::tanh((a - limit) / knee), x);
        }
      }
      s[i] = clamp_i16(x * 32768.0f);
    }
  }

 private:
  RxDspConfig cfg_{};
  Biquad hp_{};
  Biquad pres_{};
  bool hp_on_{false};
  bool pres_on_{false};
  float gain_lin_{1.0f};
  float limit_{0.794f};
  uint32_t rate_{0};
};

// Stima del rumore ambientale su una finestra breve. Suddivide l'audio in
// blocchi da 20 ms, calcola la potenza di ciascuno (senza componente continua)
// e restituisce il 30o percentile: robusto contro un picco isolato (clacson,
// una voce vicina) senza ignorare il rumore costante del traffico.
class NoiseMeter {
 public:
  void configure(uint32_t skip_ms, uint32_t measure_ms) {
    this->skip_ms_ = skip_ms;
    this->measure_ms_ = measure_ms;
  }

  void begin(uint32_t sample_rate) {
    this->target_blocks_ = std::max<uint32_t>(1, this->measure_ms_ / kBlockMs);
    this->block_len_ = std::max<uint32_t>(1, sample_rate * kBlockMs / 1000u);
    this->skip_samples_ = static_cast<uint64_t>(sample_rate) * this->skip_ms_ / 1000u;
    this->seen_ = 0;
    this->in_block_ = 0;
    this->sum_ = 0.0;
    this->sumsq_ = 0.0;
    this->powers_.clear();
    this->powers_.reserve(this->max_blocks_() + 2);
  }

  void feed(const int16_t *s, size_t n) {
    for (size_t i = 0; i < n; i++) {
      this->seen_++;
      if (this->seen_ <= this->skip_samples_) continue;  // avvio I2S: da scartare
      const double v = static_cast<double>(s[i]) / 32768.0;
      this->sum_ += v;
      this->sumsq_ += v * v;
      if (++this->in_block_ >= this->block_len_) {
        const double mean = this->sum_ / this->in_block_;
        const double var = std::max(0.0, this->sumsq_ / this->in_block_ - mean * mean);
        if (this->powers_.size() < this->max_blocks_()) this->powers_.push_back(var);
        this->in_block_ = 0;
        this->sum_ = 0.0;
        this->sumsq_ = 0.0;
      }
    }
  }

  bool window_full() const { return this->powers_.size() >= this->target_blocks_; }
  bool has_min_blocks() const { return this->powers_.size() >= kMinBlocks; }

  // dBFS RMS (pieno scala = 0 dBFS per un'onda quadra).
  float result_dbfs() {
    if (this->powers_.empty()) return -120.0f;
    std::vector<double> p = this->powers_;
    const size_t k = static_cast<size_t>(p.size() * 0.3);
    std::nth_element(p.begin(), p.begin() + k, p.end());
    const double v = std::max(p[k], 1e-12);
    return static_cast<float>(10.0 * std::log10(v));
  }

 private:
  static constexpr uint32_t kBlockMs = 20;
  static constexpr size_t kMinBlocks = 5;  // almeno 100 ms utili
  size_t max_blocks_() const { return std::max<size_t>(8, this->target_blocks_ + 8); }

  uint32_t skip_ms_{200};
  uint32_t measure_ms_{600};
  uint32_t block_len_{320};
  uint64_t skip_samples_{0};
  uint64_t seen_{0};
  uint32_t in_block_{0};
  uint32_t target_blocks_{30};
  double sum_{0.0};
  double sumsq_{0.0};
  std::vector<double> powers_;
};

}  // namespace voip_dsp
}  // namespace esphome
