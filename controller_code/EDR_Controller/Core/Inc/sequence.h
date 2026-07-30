#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <stdbool.h>
#include "stm32h7xx_hal.h"

//typedef enum {
//    STEP_MOVE,
//    STEP_GPIO,
//    STEP_DELAY,
//} step_type_t;

/* sequence.h - add to step_type_t and add one function */
typedef enum {
    STEP_MOVE,
    STEP_GPIO,
    STEP_DELAY,
    STEP_WAIT_UNTIL,   /* waits until an absolute local tick set via Sequence_SetWaitTarget() */
} step_type_t;

/* Call before Sequence_Start() if the sequence contains a STEP_WAIT_UNTIL -
   sets the absolute HAL_GetTick() value that step will wait for. */
void Sequence_SetWaitTarget(uint32_t target_tick_ms);

typedef struct {
    step_type_t type;
    /* STEP_MOVE */
    float x, y;
    /* STEP_GPIO */
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState pin_state;
    /* STEP_DELAY */
    uint32_t delay_ms;
} sequence_step_t;

/* Starts playback of `steps` (array of length `count`) from the top.
   Non-blocking - returns immediately. */
void Sequence_Start(const sequence_step_t *steps, uint8_t count);

/* Call once per control tick (same cadence as Motion_Update()). Advances
   the sequence by at most one step per call for MOVE/DELAY waits; GPIO
   steps fire and advance in the same call. */
void Sequence_Update(void);

bool Sequence_IsRunning(void);

#endif
