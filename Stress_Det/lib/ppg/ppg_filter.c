#include "ppg_filter.h"

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float x1, x2;
    float y1, y2;
} biquad_t;

static biquad_t hp, lp;

void ppg_filter_init()
{
    // --- High-pass 0.5 Hz (Fs = 200 Hz) ---
    hp.b0 = 0.9845f;
    hp.b1 = -1.9690f;
    hp.b2 = 0.9845f;

    hp.a1 = -1.9689f;
    hp.a2 = 0.9691f;

    hp.x1 = hp.x2 = 0.0f;
    hp.y1 = hp.y2 = 0.0f;

    // New
    lp.b0 = 0.0286f;
    lp.b1 = 0.0572f;
    lp.b2 = 0.0286f;
    lp.a1 = -1.4542f;
    lp.a2 = 0.5686f;
    /*
        lp.b0 = 0.0036f;
        lp.b1 = 0.0072f;
        lp.b2 = 0.0036f;
        lp.a1 = -1.8227f;
        lp.a2 = 0.8372f;*/

    // --- Low-pass 8 Hz (Fs = 200 Hz) ---
    /*
    lp.b0 = 0.0134f;
    lp.b1 = 0.0267f;
    lp.b2 = 0.0134f;

    lp.a1 = -1.6475f;
    lp.a2 = 0.7009f;*/

    lp.x1 = lp.x2 = 0.0f;
    lp.y1 = lp.y2 = 0.0f;
}

float biquad_process(biquad_t *f, float x)
{
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;

    f->x2 = f->x1;
    f->x1 = x;

    f->y2 = f->y1;
    f->y1 = y;

    return y;
}

float ppg_filter_process(float x)
{
    float y;

    // först high-pass
    y = biquad_process(&hp, x);

    // sedan low-pass
    y = biquad_process(&lp, y);

    return y;
}