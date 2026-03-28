#ifndef PICO_SYNTH_BP_H_
#define PICO_SYNTH_BP_H_

#include "pico_synth_ex.h"        // for Q28 type
#include "pico_synth_bp_table.h"  // your precomputed table

// AFTER
typedef struct {
    int32_t b0, b2;   // Q15 — b1 is always 0 for BPF, dropped
    int32_t a1, a2;   // Q15
    Q28 x1, x2;       // state stays Q28
    Q28 y1, y2;
} BiquadBPF;

static BiquadBPF BPF_voice[4]; // 4 voices

// assign coefficients from precomputed table
static inline void BPF_set_from_table(BiquadBPF* f, int table_index) {
    if (table_index < 0) table_index = 0;
    if (table_index >= (int)(sizeof(BPF_sweep)/sizeof(BPF_sweep[0])))
        table_index = sizeof(BPF_sweep)/sizeof(BPF_sweep[0]) - 1;

    f->b0 = BPF_sweep[table_index].b0 >> 13;
    f->b2 = BPF_sweep[table_index].b2 >> 13;
    f->a1 = BPF_sweep[table_index].a1 >> 13;
    f->a2 = BPF_sweep[table_index].a2 >> 13;

}

// Q28 BPF process using 64-bit accumulator
static inline Q28 BPF_process(BiquadBPF* f, Q28 x0) {
    // coefficients are Q15, state is Q28 → product is Q43 → >> 15 gives Q28
    int64_t acc = (int64_t)f->b0 * x0 +
                  (int64_t)f->b2 * f->x2 -
                  (int64_t)f->a1 * f->y1 -
                  (int64_t)f->a2 * f->y2;

    Q28 y0 = (Q28)(acc >> 15);

    f->x2 = f->x1; f->x1 = x0;
    f->y2 = f->y1; f->y1 = y0;

    return y0;
}

#endif
