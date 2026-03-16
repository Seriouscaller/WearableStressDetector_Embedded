#include "ppg_hrv.h"
#include <math.h>

#define MAX_RR 128
#define WINDOW_SEC 30.0f

typedef struct
{
    float rr;
    float time;

} rr_entry_t;

static rr_entry_t rr_buffer[MAX_RR];
static int rr_count = 0;

void ppg_hrv_init()
{
    rr_count = 0;
}

void ppg_add_rr(float rr, float time)
{
    if(rr_count < MAX_RR)
    {
        rr_buffer[rr_count].rr = rr;
        rr_buffer[rr_count].time = time;
        rr_count++;
    }
    else
    {
        for(int i=1;i<MAX_RR;i++)
            rr_buffer[i-1] = rr_buffer[i];

        rr_buffer[MAX_RR-1].rr = rr;
        rr_buffer[MAX_RR-1].time = time;
    }
}

static float mean(float *x,int n)
{
    float s = 0;

    for(int i=0;i<n;i++)
        s += x[i];

    return s/n;
}

static float std(float *x,int n)
{
    float m = mean(x,n);

    float v = 0;

    for(int i=0;i<n;i++)
    {
        float d = x[i] - m;
        v += d*d;
    }

    return sqrtf(v/(n-1));
}

static float rmssd(float *x,int n)
{
    float s = 0;

    for(int i=1;i<n;i++)
    {
        float d = x[i] - x[i-1];
        s += d*d;
    }

    return sqrtf(s/(n-1));
}

ppg_features_t ppg_compute_hrv(float now)
{
    ppg_features_t f = {0};

    float window_start = now - WINDOW_SEC;

    float rr_window[MAX_RR];
    int n = 0;

    for(int i=0;i<rr_count;i++)
    {
        if(rr_buffer[i].time >= window_start)
        {
            rr_window[n++] = rr_buffer[i].rr;
        }
    }

    if(n < 5)
        return f;

    float m = mean(rr_window,n);

    f.hr = 60000.0f / m;

    f.sdnn = std(rr_window,n);

    f.rmssd = rmssd(rr_window,n);

    return f;
}