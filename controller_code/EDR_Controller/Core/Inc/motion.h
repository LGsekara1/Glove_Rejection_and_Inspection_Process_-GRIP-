/* motion.h */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTION_ERR_NONE = 0,
    MOTION_ERR_CLOSED_LOOP_TIMEOUT,
    MOTION_ERR_ENCODER_READ_FAILED,
} motion_init_err_t;

/* Call once at startup. Brings both axes to CLOSED_LOOP_CONTROL and reads
   the current encoder position as the initial known pose. Must succeed
   before Motion_MoveTo() / Motion_Update() are used. */
motion_init_err_t Motion_Init(void);

/* Non-blocking: enqueues a target (x, y in mm), returns immediately.
   Call it repeatedly to queue several moves - they execute in the order
   they were enqueued, one after another, without stopping between them
   for anything other than the queue itself.
   Returns false (and logs why) if the queue is full or the target is
   unreachable given the current geometry/elbow config. */
bool Motion_MoveTo(float x_mm, float y_mm);

/* Call once per 100 Hz control tick (from the TIM6 ISR flag in main()). */
void Motion_Update(void);

bool Motion_IsMoving(void);
uint8_t Motion_QueueDepth(void);
void Motion_ClearQueue(void); /* drops all pending (not-yet-started) moves; does not stop a move already in progress */

#endif
