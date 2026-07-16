/* motion.h */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>

typedef enum {
    MOTION_ERR_NONE = 0,
    MOTION_ERR_CLOSED_LOOP_TIMEOUT,
    MOTION_ERR_ENCODER_READ_FAILED,
    MOTION_ERR_TARGET_UNREACHABLE,
} motion_err_t;

/* One-shot: bring both axes to CLOSED_LOOP_CONTROL, read current pose,
   compute IK for (TARGET_X_MM, TARGET_Y_MM), build the synchronized
   trapezoidal profile. Returns MOTION_ERR_NONE on success. */
motion_err_t Motion_PrepareMove(void);

/* Call once per 100 Hz tick (from TIM6 ISR flag in main loop).
   Returns true while the move is still in progress, false once
   it's complete (caller should stop calling after that). */
bool Motion_StreamTick(void);

#endif
