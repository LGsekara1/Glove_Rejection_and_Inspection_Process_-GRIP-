/* sequence.c */
#include "sequence.h"
#include "motion.h"
#include "app_log.h"

//typedef enum {
//    SEQ_IDLE,
//    SEQ_MOVE_WAIT,
//    SEQ_DELAY_WAIT,
//} seq_state_t;

typedef enum {
	SEQ_IDLE, SEQ_MOVE_WAIT, SEQ_DELAY_WAIT, SEQ_WAIT_UNTIL_WAIT,
} seq_state_t;

static const sequence_step_t *s_steps = NULL;
static uint8_t s_count = 0;
static uint8_t s_index = 0;
static seq_state_t s_state = SEQ_IDLE;
static uint32_t s_step_t0 = 0;
static bool s_running = false;

static uint32_t s_wait_until_target = 0;

void Sequence_SetWaitTarget(uint32_t target_tick_ms) {
	s_wait_until_target = target_tick_ms;
}

void Sequence_Start(const sequence_step_t *steps, uint8_t count) {
	s_steps = steps;
	s_count = count;
	s_index = 0;
	s_state = SEQ_IDLE;
	s_running = (count > 0);
	cdc_log("Sequence: starting, %d step(s).\r\n", (int) count);
}

bool Sequence_IsRunning(void) {
	return s_running;
}

void Sequence_Update(void) {
	if (!s_running)
		return;

	if (s_index >= s_count) {
		s_running = false;
		cdc_log("Sequence: complete.\r\n");
		return;
	}

	const sequence_step_t *step = &s_steps[s_index];

	switch (s_state) {
	case SEQ_IDLE:
		switch (step->type) {
		case STEP_MOVE:
			if (Motion_MoveTo(step->x, step->y)) {
				cdc_log("Sequence[%d]: move -> (%.2f, %.2f)\r\n", (int) s_index,
						(double) step->x, (double) step->y);
				s_state = SEQ_MOVE_WAIT;
			} else {
				cdc_log("Sequence[%d]: move rejected, aborting sequence.\r\n",
						(int) s_index);
				s_running = false;
			}
			break;

		case STEP_GPIO:
			HAL_GPIO_WritePin(step->port, step->pin, step->pin_state);
			cdc_log("Sequence[%d]: gpio -> %s\r\n", (int) s_index,
					(step->pin_state == GPIO_PIN_SET) ? "SET" : "RESET");
			s_index++; /* instant - advance right away */
			break;

		case STEP_DELAY:
			s_step_t0 = HAL_GetTick();
			s_state = SEQ_DELAY_WAIT;
			break;

		case STEP_WAIT_UNTIL: {
			/* signed subtraction handles HAL_GetTick() wraparound correctly */
			int32_t remaining = (int32_t) s_wait_until_target
					- (int32_t) HAL_GetTick();
			if (remaining < 0) {
				cdc_log(
						"Sequence[%d]: WAIT_UNTIL target already passed by %ld ms - grabbing now (late).\r\n",
						(int) s_index, (long) (-remaining));
			} else {
				cdc_log(
						"Sequence[%d]: waiting %ld ms for scheduled grab...\r\n",
						(int) s_index, (long) remaining);
			}
			s_state = SEQ_WAIT_UNTIL_WAIT;
			break;

		}
		}
		break;

	case SEQ_MOVE_WAIT:
		if (!Motion_IsMoving()) {
			s_index++;
			s_state = SEQ_IDLE;
		}
		break;

	case SEQ_DELAY_WAIT:
		if (HAL_GetTick() - s_step_t0 >= step->delay_ms) {
			s_index++;
			s_state = SEQ_IDLE;
		}
		break;
	case SEQ_WAIT_UNTIL_WAIT:
		if ((int32_t) (HAL_GetTick() - s_wait_until_target) >= 0) {
			s_index++;
			s_state = SEQ_IDLE;
		}
		break;
	}
}
