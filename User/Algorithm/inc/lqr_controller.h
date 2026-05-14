#ifndef __LQR_CONTROLLER_H
#define __LQR_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float k1;
    float k2;
    float k3;

    float u_last;

    float out_max;

} LQR_t;

void LQR_Init(
    LQR_t *lqr,
    float k1,
    float k2,
    float k3,
    float out_max
);

float LQR_Update(
    LQR_t *lqr,
    float ref,
    float pos,
    float vel
);

void LQR_Reset(
    LQR_t *lqr
);

#ifdef __cplusplus
}
#endif

#endif