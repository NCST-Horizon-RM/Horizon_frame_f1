#include "MotorIdentify.h"
#include "math.h"

#define PI 3.14159265358979323846f

static void Shift(float *buf, float val)
{
    buf[2] = buf[1];
    buf[1] = buf[0];
    buf[0] = val;
}

void MotorIdentify_Init(MotorIdentify_t *id,
                        float ts)
{
    for(int i = 0; i < 3; i++)
    {
        id->u[i] = 0.0f;
        id->y[i] = 0.0f;
    }

    for(int i = 0; i < 4; i++)
    {
        id->phi[i] = 0.0f;
    }

    id->ts = ts;

    id->time = 0.0f;

    id->amp = 0.0f;

    id->f0 = 0.0f;
    id->f1 = 0.0f;

    id->sweep_time = 1.0f;
}

void MotorIdentify_Update(MotorIdentify_t *id,
                          float u_now,
                          float y_now)
{
    Shift(id->u, u_now);
    Shift(id->y, y_now);

    id->phi[0] = -id->y[1];
    id->phi[1] = -id->y[2];

    id->phi[2] =  id->u[1];
    id->phi[3] =  id->u[2];
}

float *MotorIdentify_GetPhi(MotorIdentify_t *id)
{
    return id->phi;
}

void MotorIdentify_SetChirp(MotorIdentify_t *id,
                            float amp,
                            float f0,
                            float f1,
                            float sweep_time)
{
    id->amp = amp;

    id->f0 = f0;
    id->f1 = f1;

    id->sweep_time = sweep_time;

    id->time = 0.0f;
}

float MotorIdentify_Chirp(MotorIdentify_t *id)
{
    float t;
    float k;

    float phase;

    float u;

    t = id->time;

    if(t > id->sweep_time)
    {
        t = id->sweep_time;
    }

    k = (id->f1 - id->f0) / id->sweep_time;

    phase =
        2.0f * PI *
        (
            id->f0 * t +
            0.5f * k * t * t
        );

    u = id->amp * sinf(phase);

    id->time += id->ts;

    return u;
}