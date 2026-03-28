/* pico_synth_ex
 * Original author: ISGK Instruments (Ryo Ishigaki)
 * Original version: v0.1.0 (2021-09-02)
 * https://github.com/risgk/pico_synth_ex   https://risgk.github.io
 * Licensed under a CC0 license
 *
 * C module version by Turi Scandurra
 * version 1.0.0
 * Licensed under a MIT-0 license
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/float.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico_synth_ex.h"
#include "pico_synth_ex_presets.h"
#include "pico_synth_ex_tables.h"
#include "sound_i2s.h"
#include "pico_synth_bp.h"
#include "pico_synth_bp_table.h" // already included inside pico_synth_bp.h

#ifdef __cplusplus
extern "C" {
#endif

//////// Oscillator group //////////////////////////////
static volatile uint8_t Osc_waveform = 0; // waveform setting value
static volatile int8_t Osc_2_coarse_pitch = +0; // oscillator 2 coarse pitch setting value
static volatile int8_t Osc_2_fine_pitch = +4; // oscillator 2 fine pitch setting value
static volatile uint8_t Osc_1_2_mix = 16; // oscillator mix setting


static inline Q28 Osc_phase_to_audio(uint32_t phase, uint8_t pitch) {
  Q14* wave_table = Osc_wave_tables[Osc_waveform][(pitch + 3) >> 2];
  uint16_t curr_index = phase >> 23;
  uint16_t next_index = (curr_index + 1) & 0x000001FF;
  Q14 curr_sample = wave_table[curr_index];
  Q14 next_sample = wave_table[next_index];
  Q14 next_weight = (phase >> 9) & 0x3FFF;
  return (curr_sample << 14) + ((next_sample - curr_sample) * next_weight);
}

static inline Q28 Osc_process(uint8_t id,
                              uint16_t full_pitch, Q14 pitch_mod_in) {
  static uint32_t phase_1[4]; // Oscillator 1 phase
  int32_t full_pitch_1 = full_pitch + ((256 * pitch_mod_in) >> 14);
  full_pitch_1 += (full_pitch_1 < 0)          * (0 - full_pitch_1);
  full_pitch_1 -= (full_pitch_1 > (120 << 8)) * (full_pitch_1 - (120 << 8));
  uint8_t pitch_1 = (full_pitch_1 + 128) >> 8;
  uint8_t tune_1  = (full_pitch_1 + 128) & 0xFF;
  uint32_t freq_1 = Osc_freq_table[pitch_1];
  phase_1[id] += freq_1 + ((id - 1) << 8); // Shift by voice
  phase_1[id] += ((int32_t) (freq_1 >> 8) * Osc_tune_table[tune_1]) >> 6;

  static uint32_t phase_2[4]; // Oscillator 2 phase
  int32_t full_pitch_2 =
      full_pitch_1 + (Osc_2_coarse_pitch << 8) + (Osc_2_fine_pitch << 2);
  full_pitch_2 += (full_pitch_2 < 0)          * (0 - full_pitch_2);
  full_pitch_2 -= (full_pitch_2 > (120 << 8)) * (full_pitch_2 - (120 << 8));
  uint8_t pitch_2 = (full_pitch_2 + 128) >> 8;
  uint8_t tune_2  = (full_pitch_2 + 128) & 0xFF;
  uint32_t freq_2 = Osc_freq_table[pitch_2];
  phase_2[id] += freq_2 + ((id - 1) << 8); // Shift by voice
  phase_2[id] += ((int32_t) (freq_2 >> 8) * Osc_tune_table[tune_2]) >> 6;

  // TODO: I want to make wave_table switching smoother (Is it better to switch at the beginning of the cycle?)
  return ((Osc_phase_to_audio(phase_1[id], pitch_1) >> 14) *
                              Osc_mix_table[Osc_1_2_mix - 0]) +
         ((Osc_phase_to_audio(phase_2[id], pitch_2) >> 14) *
                              Osc_mix_table[64 - Osc_1_2_mix]);
}

//////// filter ///////////////////////////////////
static volatile uint8_t Filter_cutoff = 60; // Cutoff setting value
static volatile uint8_t Filter_resonance = 3; // Resonance setting value
static volatile int8_t Filter_mod_amount = +60; // Cutoff modulation amount setting value

static inline int32_t mul_s32_s32_h32(int32_t x, int32_t y) {
  // Higher 32 bits of signed 32-bit multiplication result
  int32_t x1 = x >> 16; uint32_t x0 = x & 0xFFFF;
  int32_t y1 = y >> 16; uint32_t y0 = y & 0xFFFF;
  int32_t x0_y1 = x0 * y1;
  int32_t z = ((x0 * y0) >> 16) + (x1 * y0) + (x0_y1 & 0xFFFF);
  return (z >> 16) + (x0_y1 >> 16) + (x1 * y1);
}

static inline Q28 Filter_process(uint8_t id, Q28 audio_in, Q14 cutoff_mod_in) {
  static uint16_t curr_cutoff[4]; // Cutoff current value
  int32_t targ_cutoff = Filter_cutoff << 2; // Cutoff target value
  targ_cutoff += (Filter_mod_amount * cutoff_mod_in) >> (14 - 2);
  targ_cutoff += (targ_cutoff < 0)   * (0 - targ_cutoff);
  targ_cutoff -= (targ_cutoff > 480) * (targ_cutoff - 480);
  curr_cutoff[id] += (curr_cutoff[id] < targ_cutoff);
  curr_cutoff[id] -= (curr_cutoff[id] > targ_cutoff);
  struct FILTER_COEFS* coefs_ptr =
      &Filter_coefs_table[Filter_resonance][curr_cutoff[id]];

  static Q28 x1[4], x2[4], y1[4], y2[4];
  Q28 x0 = audio_in;
  Q28 x3 = x0 + (x1[id] << 1) + x2[id];
  Q28 y0 = mul_s32_s32_h32(coefs_ptr->b0_a0, x3)     << 4;
  y0    -= mul_s32_s32_h32(coefs_ptr->a1_a0, y1[id]) << 4;
  y0    -= mul_s32_s32_h32(coefs_ptr->a2_a0, y2[id]) << 4;
  x2[id] = x1[id]; y2[id] = y1[id]; x1[id] = x0; y1[id] = y0;
  return y0;
}

//////// Amplifier //////////////////////////////////
static inline Q28 Amp_process(uint8_t id, Q28 audio_in, Q14 gain_in) {
  return (audio_in >> 14) * gain_in; // Simplify calculation
}

//////// EG (Envelope Generator) /////////////////
static uint32_t EG_exp_table[65]; // Exponential table

static volatile uint8_t EG_decay_time = 40; // Decay time setting value
static volatile uint8_t EG_sustain_level = 0; // Sustain level setting value

static inline Q14 EG_process(uint8_t id, uint8_t gate_in) {
  static int32_t curr_level[4]; // EG output level current value
  static uint8_t curr_gate[4]; // gate input level current value
  static uint8_t curr_attack_phase[4]; // current attack phase

  curr_attack_phase[id] |= (curr_gate[id] == 0) & gate_in;
  curr_attack_phase[id] &= (curr_level[id] < (1 << 24)) & gate_in;
  curr_gate[id]          =  gate_in;

  if (curr_attack_phase[id]) {
    int32_t attack_targ_level = (1 << 24) + (1 << 23);
    curr_level[id] += ((attack_targ_level - curr_level[id]) >> 5);
  } else {
    static uint32_t decay_counter[4]; // Decay counter
    ++decay_counter[id];
    decay_counter[id] =
        (decay_counter[id] < EG_exp_table[EG_decay_time]) * decay_counter[id];
    int32_t decay_targ_level = (EG_sustain_level << 18) * curr_gate[id];
    int32_t to_decay = (curr_level[id] > decay_targ_level) &
                       (decay_counter[id] == 0);
    curr_level[id] += to_decay *
                      ((decay_targ_level - curr_level[id]) >> 5);
  }

  return curr_level[id] >> 10;
}

//////// Low Frequency Oscillator (LFO) /////////
static volatile uint8_t LFO_depth = 16; // Depth setting value
static volatile uint8_t LFO_rate = 48; // Speed ​​setting value

static inline Q14 LFO_process(uint8_t id) {
  static uint32_t phase[4]; // Phase
  phase[id] += LFO_freq_table[LFO_rate] + ((id - 1) << 8); // Shift by voice

  // Generate triangle wave
  uint16_t phase_h16 = phase[id] >> 16;
  uint16_t out = phase_h16;
  out += (phase_h16 >= 32768) * (65536 - (phase_h16 << 1));
  return ((out - 16384) * LFO_depth) >> 7;
}

//////// I2S Audio output ////////////
bool i2s_timer_callback(repeating_timer_t *timer) {
  static int16_t *last_buffer;
  int16_t *buffer = sound_i2s_get_next_buffer();
  if (buffer == NULL) return true;
  if (buffer != last_buffer) {
    last_buffer = buffer;
    Q28 voice_out[4];
    for (int i = 0; i < SOUND_I2S_BUFFER_NUM_SAMPLES; i++) {
      voice_out[0] = process_voice(0);
      voice_out[1] = process_voice(1);
      voice_out[2] = process_voice(2);
      voice_out[3] = process_voice(3);
      Q28 mix = ((voice_out[0] + voice_out[1] +
                        voice_out[2] + voice_out[3]) >> 2);
      uint16_t level = (mix >> 14);

      // Copy to I2S buffer
      *buffer++ = level;
      *buffer++ = level;
    }
  }
  return true;
}


//////// PWM audio output block ///////////////////////
static bool use_pwm;
int8_t PWMA_R_GPIO = -1;
int8_t PWMA_L_GPIO = -1;
static int8_t PWMA_R_SLICE;
static int8_t PWMA_L_SLICE;
static int8_t PWMA_R_CHAN;
static int8_t PWMA_L_CHAN;

#define PWMA_CYCLE (FCLKSYS / FS) // PWM cycle

void PWMA_init(int8_t pwm_gpio_r, int8_t pwm_gpio_l) {
  use_pwm = true;
  PWMA_R_GPIO = pwm_gpio_r;
  PWMA_L_GPIO = pwm_gpio_l;
  if(PWMA_R_GPIO > -1) PWMA_R_SLICE = pwm_gpio_to_slice_num(PWMA_R_GPIO);
  if(PWMA_R_GPIO > -1) PWMA_R_CHAN = pwm_gpio_to_channel(PWMA_R_GPIO);
  if(PWMA_L_GPIO > -1) PWMA_L_SLICE = pwm_gpio_to_slice_num(PWMA_L_GPIO);
  if(PWMA_L_GPIO > -1) PWMA_L_CHAN = pwm_gpio_to_channel(PWMA_L_GPIO);
  if(PWMA_R_GPIO > -1) gpio_set_function(PWMA_R_GPIO, GPIO_FUNC_PWM);
  if(PWMA_L_GPIO > -1) gpio_set_function(PWMA_L_GPIO, GPIO_FUNC_PWM);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
  irq_set_enabled(PWM_IRQ_WRAP, true);
  pwm_set_irq_enabled(PWMA_L_SLICE, true); // Left channel must be active
  if(PWMA_R_GPIO > -1) pwm_set_wrap(PWMA_R_SLICE, PWMA_CYCLE - 1);
  if(PWMA_L_GPIO > -1) pwm_set_wrap(PWMA_L_SLICE, PWMA_CYCLE - 1);
  if(PWMA_R_GPIO > -1) pwm_set_chan_level(PWMA_R_SLICE, PWMA_R_CHAN, PWMA_CYCLE / 2);
  if(PWMA_L_GPIO > -1) pwm_set_chan_level(PWMA_L_SLICE, PWMA_L_CHAN, PWMA_CYCLE / 2);
  if(PWMA_R_GPIO > -1) pwm_set_enabled(PWMA_R_SLICE, true);
  if(PWMA_L_GPIO > -1) pwm_set_enabled(PWMA_L_SLICE, true);
}

static inline void PWMA_process(Q28 audio_l, Q28 audio_r) {
  int32_t level_l = (audio_l >> 18) + (PWMA_CYCLE / 2);
  int32_t level_r = (audio_r >> 18) + (PWMA_CYCLE / 2);
  uint16_t out_l = (level_l > 0) * level_l;
  uint16_t out_r = (level_r > 0) * level_r;
  if(PWMA_L_GPIO > -1) pwm_set_chan_level(PWMA_L_SLICE, PWMA_L_CHAN, out_l);
  if(PWMA_R_GPIO > -1) pwm_set_chan_level(PWMA_R_SLICE, PWMA_R_CHAN, out_r);
}

//////// Interrupt handler and main functions ////////////
static volatile uint16_t start_time = 0; // start time
static volatile uint16_t max_start_time = 0; // max start time
static volatile uint16_t proc_time = 0; // processing time
static volatile uint16_t max_proc_time = 0; // maximum processing time

static volatile uint8_t gate_voice[4]; // gate control value (per voice)
static volatile uint8_t pitch_voice[4]; // pitch control value (per voice)

// Per-voice pan: 0=hard left, 64=centre, 127=hard right
// Derived from pitch so pan is consistent regardless of voice slot assignment
static volatile uint8_t pan_voice[4] = {64, 64, 64, 64};

// Map a MIDI pitch to a pan position spread evenly across the keyboard
// Low notes pan left, high notes pan right
static inline uint8_t pitch_to_pan(uint8_t pitch) {
    // MIDI range 0-127, map to 8-119 to avoid extreme hard pan
    return 8 + ((uint16_t)pitch * 111) / 127;
}
static volatile int8_t Octave_shift; // key octave shift amount


//////// Vowel formant filter ///////////////////////////
// Three biquad bandpass filters per voice (F1 + F2 + F3 formants)
static BiquadBPF BPF_voice1[4];
static BiquadBPF BPF_voice2[4];
static BiquadBPF BPF_voice3[4];

static volatile vowel_t CurrentVowel = VOWEL_NONE;
static volatile vowel_t TargetVowel  = VOWEL_NONE;

// ── Formant data matching JUCE VowelFilter::vowelFormants exactly ──────────
// { f1, f2, f3, bw1, bw2, bw3, gain1, gain2, gain3 }  (freq/bw in Hz)
// Gains stored as Q8 fixed-point (1.0 = 256)
typedef struct {
    uint16_t f1, f2, f3;       // formant centre frequencies (Hz)
    uint8_t  bw1, bw2, bw3;    // bandwidths (Hz / 4, to fit uint8)
    uint16_t g1, g2, g3;       // gains Q8 (256 = 1.0)
} VowelFormantData;

// bandwidth/bw Halved for more pronounced peaks 
static const VowelFormantData VowelFormants[5] = {
    // A:  f=730,1090,2440  bw=40,50,60
    { 730, 1090, 2440,  10, 13, 15,  384, 256, 128 },
    // E:  f=530,1840,2480  bw=40,50,60
    { 530, 1840, 2480,  10, 13, 15,  384, 307, 154 },
    // I:  f=270,2290,3010  bw=20,50,60
    { 270, 2290, 3010,   5, 13, 15,  307, 384, 205 },
    // O:  f=570, 840,2410  bw=40,40,60
    { 570,  840, 2410,  10, 10, 15,  384, 205, 128 },
    // U:  f=440,1020,2240  bw=40,40,60
    { 440, 1020, 2240,  10, 10, 15,  307, 154, 102 },
};

// Continuous morph value: 0.0=A, 1.0=E, 2.0=I, 3.0=O, 4.0=U
// Stored as Q8 fixed-point (0–1024 maps to 0.0–4.0)
static volatile int32_t vowel_morph_q8 = 0;   // Q8: 256 per vowel step

// Per-voice smoothed morph (Q8) — interpolates toward vowel_morph_q8
static volatile int32_t smooth_morph_q8[4] = {0, 0, 0, 0};

// Compute biquad bandpass coefficients directly from Hz frequency + bandwidth.
// Fs = 44100. Returns coefficients in Q28 with unity peak gain.
// Called outside the IRQ (from set_vowel / morph update path).
static inline void BPF_set_from_freq(BiquadBPF* f, uint16_t freq_hz, uint16_t bw_hz) {
    // Guard against out-of-range values
    if (freq_hz < 50)  freq_hz = 50;
    if (freq_hz > 8000) freq_hz = 8000;
    if (bw_hz   < 20)  bw_hz   = 20;

    // Use float here — called at most every ~50ms, not in IRQ
    float w0    = (6.283185f * freq_hz) / 44100.0f;
    float sinw  = sinf(w0);
    float cosw  = cosf(w0);
    float alpha = sinw * bw_hz / (2.0f * freq_hz);  // = sin(w0)/(2Q), Q=f/bw

    float b0 =  alpha;
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cosw;
    float a2 =  1.0f - alpha;

    // Normalise by a0 then normalise peak gain to unity
    b0 /= a0;  a1 /= a0;  a2 /= a0;

    // Peak gain of BPF at w0 (exact closed-form)
    float peak = b0 / (1.0f - fabsf(a2));
    if (peak > 1e-6f) b0 /= peak;

    int32_t Q15scale = 1 << 15;
    f->b0 = (int32_t)( b0 * Q15scale);
    f->b2 = (int32_t)(-b0 * Q15scale);
    f->a1 = (int32_t)( a1 * Q15scale);
    f->a2 = (int32_t)( a2 * Q15scale);
}

// Interpolate formant parameters between two adjacent vowels.
// morph_frac is Q8 fraction between vowel lo and hi (0..255).
static inline void get_interpolated_formant(
    int vowel_lo, int vowel_frac_q8,
    uint16_t *f1, uint16_t *f2, uint16_t *f3,
    uint16_t *bw1, uint16_t *bw2, uint16_t *bw3,
    int32_t  *g1,  int32_t  *g2,  int32_t  *g3)
{
    int vowel_hi = vowel_lo + 1;
    if (vowel_hi > 4) vowel_hi = 4;
    const VowelFormantData *va = &VowelFormants[vowel_lo];
    const VowelFormantData *vb = &VowelFormants[vowel_hi];
    int t = vowel_frac_q8;  // 0..255

    // Linear interpolation: result = a + t*(b-a)/256
    *f1  = va->f1  + ((int32_t)(vb->f1  - va->f1)  * t >> 8);
    *f2  = va->f2  + ((int32_t)(vb->f2  - va->f2)  * t >> 8);
    *f3  = va->f3  + ((int32_t)(vb->f3  - va->f3)  * t >> 8);
    *bw1 = (va->bw1 + ((int32_t)(vb->bw1 - va->bw1) * t >> 8)) * 4;  // restore Hz
    *bw2 = (va->bw2 + ((int32_t)(vb->bw2 - va->bw2) * t >> 8)) * 4;
    *bw3 = (va->bw3 + ((int32_t)(vb->bw3 - va->bw3) * t >> 8)) * 4;
    *g1  = va->g1  + ((int32_t)(vb->g1  - va->g1)  * t >> 8);  // Q8
    *g2  = va->g2  + ((int32_t)(vb->g2  - va->g2)  * t >> 8);
    *g3  = va->g3  + ((int32_t)(vb->g3  - va->g3)  * t >> 8);
}

// Cached interpolated BPF coefficients (recomputed when morph changes).
// Updated in set_vowel_morph(), read inside process_voice() IRQ.
// One set per voice would be ideal but costs RAM; one shared set is fine
// for a mono-timbre drone instrument.
static volatile uint16_t cached_f1_hz,  cached_f2_hz,  cached_f3_hz;
static volatile uint16_t cached_bw1_hz, cached_bw2_hz, cached_bw3_hz;
static volatile int32_t  cached_g1_q8,  cached_g2_q8,  cached_g3_q8;
static volatile bool     cached_dirty = true;

// Called from synth_params / gesture handler when vowel changes.
// Accepts a float morph 0.0–4.0 matching the JUCE setVowelMorph() API.
void set_vowel_morph(float morph) {
    if (morph < 0.0f) morph = 0.0f;
    if (morph > 4.0f) morph = 4.0f;
    vowel_morph_q8 = (int32_t)(morph * 256.0f);

    // Ensure process_voice uses the formant path (not lowpass)
    TargetVowel = (vowel_t)((int)morph);
    if (TargetVowel > VOWEL_U) TargetVowel = VOWEL_U;

    // Precompute interpolated formant parameters (float, safe outside IRQ)
    int vowel_lo      = (int)morph;
    if (vowel_lo > 3) vowel_lo = 3;
    int vowel_frac_q8 = vowel_morph_q8 - (vowel_lo << 8);

    uint16_t f1, f2, f3, bw1, bw2, bw3;
    int32_t  g1, g2, g3;
    get_interpolated_formant(vowel_lo, vowel_frac_q8,
                             &f1, &f2, &f3, &bw1, &bw2, &bw3, &g1, &g2, &g3);
    cached_f1_hz  = f1;   cached_f2_hz  = f2;   cached_f3_hz  = f3;
    cached_bw1_hz = bw1;  cached_bw2_hz = bw2;  cached_bw3_hz = bw3;
    cached_g1_q8  = g1;   cached_g2_q8  = g2;   cached_g3_q8  = g3;
    cached_dirty  = true;
}

// Legacy hard-switch API — converts discrete vowel to morph float
void set_vowel(vowel_t vowel) {
    TargetVowel = vowel;
    if (vowel == VOWEL_NONE) {
        vowel_morph_q8 = -1;  // sentinel: use lowpass
    } else {
        set_vowel_morph((float)vowel);
    }
}

static inline Q28 process_voice(uint8_t id) {
    // LFO + EG
    Q14 lfo_out = LFO_process(id);
    Q14 eg_out  = EG_process(id, gate_voice[id]);

    // Oscillator
    Q28 osc_out = Osc_process(id, pitch_voice[id] << 8, lfo_out);

    Q28 filtered;

    if (TargetVowel == VOWEL_NONE) {
        // No vowel filter: use the original lowpass
        filtered = Filter_process(id, osc_out, lfo_out);
    } else {
        // ── Vowel formant filter with morph interpolation ───────────────
        // Update BPF coefficients when morph changed (only voice 0 triggers it)
        if (cached_dirty && id == 0) {
            BPF_set_from_freq(&BPF_voice1[0], cached_f1_hz, cached_bw1_hz);
            BPF_set_from_freq(&BPF_voice2[0], cached_f2_hz, cached_bw2_hz);
            BPF_set_from_freq(&BPF_voice3[0], cached_f3_hz, cached_bw3_hz);
            // Copy to other voices
            for (int v = 1; v < 4; v++) {
                BPF_voice1[v].b0 = BPF_voice1[0].b0; BPF_voice1[v].b2 = BPF_voice1[0].b2;
                BPF_voice1[v].a1 = BPF_voice1[0].a1; BPF_voice1[v].a2 = BPF_voice1[0].a2;
                BPF_voice2[v].b0 = BPF_voice2[0].b0; BPF_voice2[v].b2 = BPF_voice2[0].b2;
                BPF_voice2[v].a1 = BPF_voice2[0].a1; BPF_voice2[v].a2 = BPF_voice2[0].a2;
                BPF_voice3[v].b0 = BPF_voice3[0].b0; BPF_voice3[v].b2 = BPF_voice3[0].b2;
                BPF_voice3[v].a1 = BPF_voice3[0].a1; BPF_voice3[v].a2 = BPF_voice3[0].a2;
            }
            cached_dirty = false;
        }

        // Run three parallel formant filters
        Q28 f1_out = BPF_process(&BPF_voice1[id], osc_out);
        Q28 f2_out = BPF_process(&BPF_voice2[id], osc_out);
        Q28 f3_out = BPF_process(&BPF_voice3[id], osc_out);

        // Mix with per-vowel gain weights (cached_gN_q8 is Q8, 256=1.0)
        // Shift each output by its gain then sum; >>8 to normalise Q8
        filtered = (int32_t)(
            ((int64_t)f1_out * cached_g1_q8 >> 8) +
            ((int64_t)f2_out * cached_g2_q8 >> 8) +
            ((int64_t)f3_out * cached_g3_q8 >> 8)
        );

        // Boost to recover bandpass volume loss (peak-normalised BPF ~= 1/Q energy)
        filtered = filtered << 1;

        // Clamp to Q28 range
        #define Q28_MAX  0x07FFFFFF
        #define Q28_MIN -0x08000000
        if (filtered > Q28_MAX) filtered = Q28_MAX;
        if (filtered < Q28_MIN) filtered = Q28_MIN;
    }

    // Amplifier
    return Amp_process(id, filtered, eg_out);
}


static void pwm_irq_handler() {
  pwm_clear_irq(PWMA_L_SLICE);
  start_time = pwm_get_counter(PWMA_L_SLICE);

  Q28 voice_out[4];
  voice_out[0] = process_voice(0);
  voice_out[1] = process_voice(1);
  voice_out[2] = process_voice(2);
  voice_out[3] = process_voice(3);

  // Spread voices across stereo field:
  // Voice 0: hard left, Voice 1: centre-left
  // Voice 2: centre-right, Voice 3: hard right
  // Each channel is normalised to >>2 to match the old mono mix level
  // Constant-power stereo panning (sin/cos at 22.5°, 67.5° steps)
  // Each voice pair sums to equal total energy on L+R
  // Voice 0: L=0.924, R=0.383  (hard left)
  // Voice 1: L=0.707, R=0.707  (centre-left)  -- approximated as equal mix
  // Voice 2: L=0.707, R=0.707  (centre-right) -- approximated as equal mix
  // Voice 3: L=0.383, R=0.924  (hard right)
  // Using shift approximations: 0.924≈1-(1>>4), 0.383≈(1>>2)+(1>>3), 0.707≈(1>>1)+(1>>3)
  Q28 v0 = voice_out[0], v1 = voice_out[1], v2 = voice_out[2], v3 = voice_out[3];
  Q28 mix_l = ((v0 - (v0 >> 4)) + (v1 >> 1) + (v1 >> 3) + (v2 >> 1) + (v2 >> 3) + (v3 >> 2) + (v3 >> 3)) >> 2;
  Q28 mix_r = ((v0 >> 2) + (v0 >> 3) + (v1 >> 1) + (v1 >> 3) + (v2 >> 1) + (v2 >> 3) + (v3 - (v3 >> 4))) >> 2;
  PWMA_process(mix_l, mix_r);

  uint16_t end_time = pwm_get_counter(PWMA_L_SLICE);
  proc_time = end_time - start_time; // simplify calculation
  max_start_time +=
      (start_time > max_start_time) * (start_time - max_start_time);
  max_proc_time +=
      (proc_time > max_proc_time) * (proc_time - max_proc_time);
}

void note_toggle(uint8_t key) {
  uint8_t pitch = key + (Octave_shift * 12);
  if      (pitch_voice[0] == pitch) { gate_voice[0] = (gate_voice[0] == 0); }
  else if (pitch_voice[1] == pitch) { gate_voice[1] = (gate_voice[1] == 0); }
  else if (pitch_voice[2] == pitch) { gate_voice[2] = (gate_voice[2] == 0); }
  else if (pitch_voice[3] == pitch) { gate_voice[3] = (gate_voice[3] == 0); }
  else if (gate_voice[0] == 0) { pitch_voice[0] = pitch; pan_voice[0] = pitch_to_pan(pitch); gate_voice[0] = 1; }
  else if (gate_voice[1] == 0) { pitch_voice[1] = pitch; pan_voice[1] = pitch_to_pan(pitch); gate_voice[1] = 1; }
  else if (gate_voice[2] == 0) { pitch_voice[2] = pitch; pan_voice[2] = pitch_to_pan(pitch); gate_voice[2] = 1; }
  else                         { pitch_voice[3] = pitch; pan_voice[3] = pitch_to_pan(pitch); gate_voice[3] = 1; }
}

void note_on(uint8_t key) {
  uint8_t pitch = key + (Octave_shift * 12);
  static uint8_t current_voice;

  pitch_voice[current_voice] = pitch;
  gate_voice[current_voice] = 1;
  current_voice = (++current_voice % 4);
}

void note_off(uint8_t key) {
  uint8_t pitch = key + (Octave_shift * 12);
  for (uint8_t id = 0; id < 4; ++id) {
    if (pitch_voice[id] == pitch) { gate_voice[id] = 0; }
  }
}

void all_notes_off() {
  for (uint8_t id = 0; id < 4; ++id) { gate_voice[id] = 0; }
}

void startup_chord() {
  for (uint8_t id = 0; id < 4; ++id) { pitch_voice[id] = 60; }
  note_on(60); note_on(64); note_on(67); note_on(71);
}

int8_t get_octave_shift() {
  return Octave_shift;
}

static void load_factory_preset(uint8_t preset) {
  Octave_shift       = presets[preset].Octave_shift;
  Osc_waveform       = presets[preset].Osc_waveform;
  Filter_cutoff      = presets[preset].Filter_cutoff;
  Filter_resonance   = presets[preset].Filter_resonance;
  Filter_mod_amount  = presets[preset].Filter_mod_amount;
  EG_decay_time      = presets[preset].EG_decay_time;
  EG_sustain_level   = presets[preset].EG_sustain_level;
  Osc_2_coarse_pitch = presets[preset].Osc_2_coarse_pitch;
  Osc_2_fine_pitch   = presets[preset].Osc_2_fine_pitch;
  Osc_1_2_mix        = presets[preset].Osc_1_2_mix;
  LFO_depth          = presets[preset].LFO_depth;
  LFO_rate           = presets[preset].LFO_rate;
}

void load_preset(Preset_t preset) {
  Octave_shift       = preset.Octave_shift;
  Osc_waveform       = preset.Osc_waveform;
  Filter_cutoff      = preset.Filter_cutoff;
  Filter_resonance   = preset.Filter_resonance;
  Filter_mod_amount  = preset.Filter_mod_amount;
  EG_decay_time      = preset.EG_decay_time;
  EG_sustain_level   = preset.EG_sustain_level;
  Osc_2_coarse_pitch = preset.Osc_2_coarse_pitch;
  Osc_2_fine_pitch   = preset.Osc_2_fine_pitch;
  Osc_1_2_mix        = preset.Osc_1_2_mix;
  LFO_depth          = preset.LFO_depth;
  LFO_rate           = preset.LFO_rate;
}

void control_message(control_message_t message) {
  switch(message){
    case OCTAVE_SHIFT_DEC:        if (Octave_shift       > -5)  { --Octave_shift;       } break;
    case OCTAVE_SHIFT_INC:        if (Octave_shift       < +4)  { ++Octave_shift;       } break;
    case FILTER_CUTOFF_DEC:       if (Filter_cutoff      > 0)   { --Filter_cutoff;      } break;
    case FILTER_CUTOFF_INC:       if (Filter_cutoff      < 120) { ++Filter_cutoff;      } break;
    case FILTER_RESONANCE_DEC:    if (Filter_resonance   > 0)   { --Filter_resonance;   } break;
    case FILTER_RESONANCE_INC:    if (Filter_resonance   < 5)   { ++Filter_resonance;   } break;
    case FILTER_MOD_AMOUNT_DEC:   if (Filter_mod_amount  > +0)  { --Filter_mod_amount;  } break;
    case FILTER_MOD_AMOUNT_INC:   if (Filter_mod_amount  < +60) { ++Filter_mod_amount;  } break;
    case EG_DECAY_TIME_DEC:       if (EG_decay_time      > 0)   { --EG_decay_time;      } break;
    case EG_DECAY_TIME_INC:       if (EG_decay_time      < 64)  { ++EG_decay_time;      } break;
    case EG_SUSTAIN_LEVEL_DEC:    if (EG_sustain_level   > 0)   { --EG_sustain_level;   } break;
    case EG_SUSTAIN_LEVEL_INC:    if (EG_sustain_level   < 64)  { ++EG_sustain_level;   } break;
    case OSC_WAVEFORM_DEC:        if (Osc_waveform       > 0)   { --Osc_waveform;       } break;
    case OSC_WAVEFORM_INC:        if (Osc_waveform       < 1)   { ++Osc_waveform;       } break;
    case OSC_2_COARSE_PITCH_DEC:  if (Osc_2_coarse_pitch > +0)  { --Osc_2_coarse_pitch; } break;
    case OSC_2_COARSE_PITCH_INC:  if (Osc_2_coarse_pitch < +24) { ++Osc_2_coarse_pitch; } break;
    case OSC_2_FINE_PITCH_DEC:    if (Osc_2_fine_pitch   > +0)  { --Osc_2_fine_pitch;   } break;
    case OSC_2_FINE_PITCH_INC:    if (Osc_2_fine_pitch   < +32) { ++Osc_2_fine_pitch;   } break;
    case OSC_1_2_MIX_DEC:         if (Osc_1_2_mix        > 0)   { --Osc_1_2_mix;        } break;
    case OSC_1_2_MIX_INC:         if (Osc_1_2_mix        < 64)  { ++Osc_1_2_mix;        } break;
    case LFO_DEPTH_DEC:           if (LFO_depth          > 0)   { --LFO_depth;          } break;
    case LFO_DEPTH_INC:           if (LFO_depth          < 64)  { ++LFO_depth;          } break;
    case LFO_RATE_DEC:            if (LFO_rate           > 0)   { --LFO_rate;           } break;
    case LFO_RATE_INC:            if (LFO_rate           < 64)  { ++LFO_rate;           } break;
    case PRESET_0:                                      load_factory_preset(0);           break;
    case PRESET_1:                                      load_factory_preset(1);           break;
    case PRESET_2:                                      load_factory_preset(2);           break;
    case PRESET_3:                                      load_factory_preset(3);           break;
    case PRESET_4:                                      load_factory_preset(4);           break;
    case PRESET_5:                                      load_factory_preset(5);           break;
    case PRESET_6:                                      load_factory_preset(6);           break;
    case PRESET_7:                                      load_factory_preset(7);           break;
    case PRESET_8:                                      load_factory_preset(8);           break;
    case PRESET_9:                                      load_factory_preset(9);           break;
    case ALL_NOTES_OFF:                                 all_notes_off();                  break;
  }
}

void set_parameter(synth_parameter_t parameter, int8_t value){
  switch(parameter){
    case OCTAVE_SHIFT:       if (value >= -5 && value <= +4)  { Octave_shift = value;       } break;    
    case OSC_WAVEFORM:       if (value >=  0 && value <= 1)   { Osc_waveform = value;       } break;
    case OSC_2_COARSE_PITCH: if (value >=  0 && value <= 24)  { Osc_2_coarse_pitch = value; } break;
    case OSC_2_FINE_PITCH:   if (value >=  0 && value <= 32)  { Osc_2_fine_pitch = value;   } break;
    case OSC_1_2_MIX:        if (value >=  0 && value <= 64)  { Osc_1_2_mix = value;        } break;
    case EG_SUSTAIN_LEVEL:   if (value >=  0 && value <= 64)  { EG_sustain_level = value;   } break;
    case EG_DECAY_TIME:      if (value >=  0 && value <= 64)  { EG_decay_time = value;      } break;
    case FILTER_CUTOFF:      if (value >=  0 && value <= 120) { Filter_cutoff = value;      } break;
    case FILTER_RESONANCE:   if (value >=  0 && value <= 5)   { Filter_resonance = value;   } break;
    case FILTER_MOD_AMOUNT:  if (value >=  0 && value <= 60)  { Filter_mod_amount = value;  } break;
    case LFO_DEPTH:          if (value >=  0 && value <= 64)  { LFO_depth = value;          } break;
    case LFO_RATE:           if (value >=  0 && value <= 64)  { LFO_rate = value;           } break;
  }
}

int8_t get_parameter(synth_parameter_t parameter) {
    switch(parameter) {
        case OCTAVE_SHIFT:       return Octave_shift;
        case OSC_WAVEFORM:       return Osc_waveform;
        case OSC_2_COARSE_PITCH: return Osc_2_coarse_pitch;
        case OSC_2_FINE_PITCH:   return Osc_2_fine_pitch;
        case OSC_1_2_MIX:        return Osc_1_2_mix;
        case EG_SUSTAIN_LEVEL:   return EG_sustain_level;
        case EG_DECAY_TIME:      return EG_decay_time;
        case FILTER_CUTOFF:      return Filter_cutoff;
        case FILTER_RESONANCE:   return Filter_resonance;
        case FILTER_MOD_AMOUNT:  return Filter_mod_amount;
        case LFO_DEPTH:          return LFO_depth;
        case LFO_RATE:           return LFO_rate;
        default:                 return 0;
    }
}

void print_status(){
  printf("Pitch             : [ %3hhu, %3hhu, %3hhu, %3hhu ]\n",
      pitch_voice[0], pitch_voice[1], pitch_voice[2], pitch_voice[3]);
  printf("Gate              : [ %3hhu, %3hhu, %3hhu, %3hhu ]\n",
      gate_voice[0], gate_voice[1], gate_voice[2], gate_voice[3]);
  printf("Octave Shift      : %+3hd\n",       Octave_shift);
  printf("Osc Waveform      : %3hhu\n",       Osc_waveform);
  printf("Osc 2 Coarse Pitch: %+3hd\n",       Osc_2_coarse_pitch);
  printf("Osc 2 Fine Pitch  : %+3hd\n",       Osc_2_fine_pitch);
  printf("Osc 1/2 Mix       : %3hhu\n",       Osc_1_2_mix);
  printf("Filter Cutoff     : %3hhu\n",       Filter_cutoff);
  printf("Filter Resonance  : %3hhu\n",       Filter_resonance);
  printf("Filter EG Amount  : %+3hd\n",       Filter_mod_amount);
  printf("EG Decay Time     : %3hhu\n",       EG_decay_time);
  printf("EG Sustain Level  : %3hhu\n",       EG_sustain_level);
  printf("LFO Depth         : %3hhu\n",       LFO_depth);
  printf("LFO Rate          : %3hhu\n",       LFO_rate);
  printf("Start Time        : %4hu/%4hu\n",   start_time, max_start_time);
  printf("Processing Time   : %4hu/%4hu\n\n", proc_time, max_proc_time);
}

#ifdef __cplusplus
}
#endif