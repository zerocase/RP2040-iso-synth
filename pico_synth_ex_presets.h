#ifndef PRESETS_H_
#define PRESETS_H_

Preset_t presets[10] = {
  { 0,  0,  0,  4,  16,  60,  3,  60,  40,  0, 16, 48}, // Default
  { 0,  1,  4,  0,   8,  50,  1,  60,  40,  0, 16, 24}, // Vibrola
  { 0,  1,  0,  2,   1,  33,  4,  42,  35, 50,  7, 39}, // Recorder
  { -2, 0, 12, 12,  24,  68,  3,  45,  14, 46, 12, 45}, // Superlead
  { 0,  1,  3,  0,  0,  100,  4,  60,  20,  0, 12, 32}, // Chromabits
  { 0,  1, 12,  2,  21,  70,  4,  19,  53, 29,  4,  8}, // Bell
  { -2, 0,  0,  0,  55,  40,  5,  18,  34, 64,  0,  0}, // Oboe
  { -3, 0, 12,  0,  61,  97,  1,  40,  12, 50,  4, 45}, // Acid bass
  // Basic vowel-like presets
  { 0, 0, 0, 0,  60, 50,  2, 0, 32, 48,  0, 0}, // "Ah" vowel - single osc, medium filter
  { 0, 0, 0, 0,  40, 40,  2, 0, 32, 48,  0, 0}, // "Oo" vowel - single osc, darker, low filter
};

/* Parameter legend for clarity:
   { Octave_shift, Osc_waveform, Osc_2_coarse_pitch, Osc_2_fine_pitch,
     Osc_1_2_mix, Filter_cutoff, Filter_resonance, Filter_mod_amount,
     EG_decay_time, EG_sustain_level, LFO_depth, LFO_rate }
*/

#endif
