#include "lqr_controller.h"
#include <math.h>

static float Limit(float x, float max)
{
    if(x > max)
    {
        x = max;
    }

    if(x < -max)
    {
        x = -max;
    }

    return x;
}

void LQR_Init(
    LQR_t *lqr,
    float k1,
    float k2,
    float k3,
    float out_max
)
{
    lqr->k1 = k1;
    lqr->k2 = k2;
    lqr->k3 = k3;

    lqr->u_last = 0.0f;

    lqr->out_max = out_max;
}

void LQR_Reset(LQR_t *lqr)
{
    lqr->u_last = 0.0f;
}

float LQR_Update(
    LQR_t *lqr,
    float ref,
    float pos,
    float vel
)
{
    float e;
    float u;
    float ep = 0.0f;
    float ev = 0.0f;

    ep = pos - ref;
    ev = vel;
    u =
        -(lqr->k1 * ep)
        +(lqr->k2 * ev)
        -(lqr->k3 * lqr->u_last);

    u = Limit(
        u,
        lqr->out_max
    );

    lqr->u_last = u;

    return u;
}