/*
 * odrive_uart.h
 *
 * Minimal driver for the ODrive v3.6 ASCII protocol over a UART, written
 * for bare-metal STM32 HAL (no odrive python lib, obviously - this is the
 * hand-rolled equivalent of what that lib does over USB/UART for you).
 *
 * ASCII protocol quick reference (fw 0.5.x):
 *   w <path> <value>\n        - write a property                 (no reply)
 *   r <path>\n                - read a property                  ("<value>\n")
 *   f <axis>\n                - fast feedback (pos, vel)          ("<pos> <vel>\n")
 *   ss\n                      - save configuration + reboot
 *   sr\n                      - reboot
 *   se\n                      - erase configuration + reboot
 *
 * All commands are terminated with '\n'. Replies (for 'r' and 'f') are
 * terminated with '\n' (commonly "\r\n" from the device - we accept either).
 *
 * Threading model: bare superloop, no RTOS assumed. RX is interrupt driven
 * into a small ring buffer; the blocking odrive_read_line() call spins
 * calling HAL_GetTick() until a full line arrives or times out. Since this
 * driver talks to a stepper... er, servo... one line at a time and homing
 * is inherently slow, blocking here is fine and keeps the code simple. If
 * you later add high-rate trajectory streaming (like the PC dashboard's
 * background thread), switch that path to fire-and-forget 'w
 * axis.controller.input_pos' writes with no reply wait, same as this file's
 * odrive_write() already does.
 */

#ifndef ODRIVE_UART_H
#define ODRIVE_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ODRIVE_RX_BUF_SIZE      256
#define ODRIVE_LINE_BUF_SIZE    128
#define ODRIVE_DEFAULT_TIMEOUT_MS 300

/* ODrive axis state constants (fw 0.5.x AXIS_STATE_*) */
typedef enum {
    ODRIVE_AXIS_STATE_UNDEFINED = 0,
    ODRIVE_AXIS_STATE_IDLE = 1,
    ODRIVE_AXIS_STATE_MOTOR_CALIBRATION = 4,
    ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
    ODRIVE_AXIS_STATE_HOMING = 11,
} odrive_axis_state_t;

typedef struct {
    UART_HandleTypeDef *huart;

    /* RX ring buffer, filled one byte at a time from the UART RX-complete
     * ISR via odrive_uart_rx_byte_isr(). */
    volatile uint8_t rx_buf[ODRIVE_RX_BUF_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;

    uint8_t rx_isr_byte; /* scratch byte HAL_UART_Receive_IT writes into */
} odrive_uart_t;

/* Call once at startup. huart must already be Init'd by CubeMX. */
void odrive_uart_init(odrive_uart_t *ctx, UART_HandleTypeDef *huart);

/* Feed this one byte at a time from your main.c's HAL_UART_RxCpltCallback
 * for the ODrive UART instance, then immediately re-arm with
 * HAL_UART_Receive_IT(huart, &ctx->rx_isr_byte, 1). See odrive_uart.c
 * top-of-file comment for the exact main.c snippet. */
void odrive_uart_rx_byte_isr(odrive_uart_t *ctx, uint8_t byte);

/* ---- Generic property access ---- */

/* "w <path> <value>\n" - fire and forget, no reply expected. */
bool odrive_write_raw(odrive_uart_t *ctx, const char *path, float value);

/* Convenience: "w axis<axis>.<subpath> <value>\n" */
bool odrive_write(odrive_uart_t *ctx, int axis, const char *subpath, float value);

/* "r <path>\n" -> parses reply as float. Returns false on timeout/parse
 * error, in which case *out_value is left untouched. */
bool odrive_read_raw(odrive_uart_t *ctx, const char *path, float *out_value, uint32_t timeout_ms);

/* Convenience: "r axis<axis>.<subpath>\n" */
bool odrive_read(odrive_uart_t *ctx, int axis, const char *subpath, float *out_value, uint32_t timeout_ms);

/* "f <axis>\n" -> "<pos> <vel>\n" (turns, turns/s). Fast path for polling
 * during homing / status refresh - one round trip gets both values instead
 * of two separate 'r' calls. */
bool odrive_get_feedback(odrive_uart_t *ctx, int axis, float *out_pos_turns, float *out_vel_turns_s,
                          uint32_t timeout_ms);

/* ---- Higher-level helpers built on the above ---- */

bool odrive_clear_errors(odrive_uart_t *ctx);

/* Reads axis.error, axis.motor.error, axis.encoder.error. Returns true if
 * all three read successfully; *any_error is set true if any is nonzero. */
bool odrive_check_errors(odrive_uart_t *ctx, int axis, bool *any_error, uint32_t timeout_ms);

/* Reads axis.current_state as an odrive_axis_state_t. */
bool odrive_read_current_state(odrive_uart_t *ctx, int axis, odrive_axis_state_t *out_state,
                                uint32_t timeout_ms);

/* Writes axis.requested_state. Non-blocking - just kicks off the state
 * transition; caller polls odrive_read_current_state() to see it finish
 * (mirrors the Python dashboard's _run_state_blocking, but split into
 * non-blocking pieces so it fits a superloop state machine instead of a
 * worker thread). */
bool odrive_request_state(odrive_uart_t *ctx, int axis, odrive_axis_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_UART_H */
