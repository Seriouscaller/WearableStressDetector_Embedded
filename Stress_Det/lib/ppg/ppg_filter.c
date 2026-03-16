#include "ppg_filter.h"

/*
 * A simple biquad bandpass filter for PPG signal processing.
 * Coefficients are designed for a specific cutoff frequency and Q factor.
 * FS = 256 HZ Bandpass 0.5 - 8 HZ
 */

typedef struct
{
    float b0,b1,b2;
    float a1,a2;

    float x1,x2;
    float y1,y2;

} biquad_t;

static biquad_t bp;

void ppg_filter_init()
{
    bp.b0 = 0.2066f;
    bp.b1 = 0.0f;
    bp.b2 = -0.2066f;

    bp.a1 = -1.5610f;
    bp.a2 = 0.5868f;

    bp.x1 = bp.x2 = 0;
    bp.y1 = bp.y2 = 0;
}

float ppg_filter_process(float x)
{
    float y =
        bp.b0*x +
        bp.b1*bp.x1 +
        bp.b2*bp.x2 -
        bp.a1*bp.y1 -
        bp.a2*bp.y2;

    bp.x2 = bp.x1;
    bp.x1 = x;

    bp.y2 = bp.y1;
    bp.y1 = y;

    return y;
}