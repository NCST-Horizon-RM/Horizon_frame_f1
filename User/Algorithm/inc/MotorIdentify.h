#ifndef __MOTOR_IDENTIFY_H__
#define __MOTOR_IDENTIFY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float u[3];
    float y[3];

    float phi[4];

    float ts;
    float time;

    float amp;

    float f0;
    float f1;

    float sweep_time;

} MotorIdentify_t;

void MotorIdentify_Init(MotorIdentify_t *id,
                        float ts);

void MotorIdentify_Update(MotorIdentify_t *id,
                          float u_now,
                          float y_now);

float *MotorIdentify_GetPhi(MotorIdentify_t *id);

void MotorIdentify_SetChirp(MotorIdentify_t *id,
                            float amp,
                            float f0,
                            float f1,
                            float sweep_time);

float MotorIdentify_Chirp(MotorIdentify_t *id);

#ifdef __cplusplus
}
#endif

#endif