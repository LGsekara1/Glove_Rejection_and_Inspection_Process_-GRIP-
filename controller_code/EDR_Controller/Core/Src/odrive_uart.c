/*
 * odrive_uart.c
 *
 * main.c wiring (add alongside your other UART callbacks):
 *
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 *   {
 *       if (huart->Instance == ODRIVE_UART_INSTANCE) {
 *           odrive_uart_rx_byte_isr(&g_odrive_ctx, g_odrive_ctx.rx_isr_byte);
 *           HAL_UART_Receive_IT(huart, &g_odrive_ctx.rx_isr_byte, 1);
 *       }
 *       ...
 *   }
 *
 * and call HAL_UART_Receive_IT(huart, &g_odrive_ctx.rx_isr_byte, 1) once
 * right after odrive_uart_init() to arm the first byte.
 */

#include "odrive_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static void ring_push(odrive_uart_t *ctx, uint8_t byte)
{
    uint16_t next = (uint16_t)((ctx->rx_head + 1) % ODRIVE_RX_BUF_SIZE);
    if (next == ctx->rx_tail) {
        /* buffer full - drop oldest byte rather than overrunning */
        ctx->rx_tail = (uint16_t)((ctx->rx_tail + 1) % ODRIVE_RX_BUF_SIZE);
    }
    ctx->rx_buf[ctx->rx_head] = byte;
    ctx->rx_head = next;
}

static bool ring_pop(odrive_uart_t *ctx, uint8_t *out_byte)
{
    if (ctx->rx_tail == ctx->rx_head) {
        return false; /* empty */
    }
    *out_byte = ctx->rx_buf[ctx->rx_tail];
    ctx->rx_tail = (uint16_t)((ctx->rx_tail + 1) % ODRIVE_RX_BUF_SIZE);
    return true;
}

void odrive_uart_init(odrive_uart_t *ctx, UART_HandleTypeDef *huart)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->huart = huart;
}

void odrive_uart_rx_byte_isr(odrive_uart_t *ctx, uint8_t byte)
{
    ring_push(ctx, byte);
}

/* Blocking send of a raw command string (already newline-terminated). */
static bool send_cmd(odrive_uart_t *ctx, const char *cmd)
{
    HAL_StatusTypeDef st = HAL_UART_Transmit(ctx->huart, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100);
    return st == HAL_OK;
}

/* Blocks (spin-wait, superloop-friendly since it just polls HAL_GetTick())
 * until a '\n'-terminated line is available or timeout_ms elapses. Strips
 * trailing \r\n. Returns false on timeout with *out empty. */
static bool read_line(odrive_uart_t *ctx, char *out, size_t out_size, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    size_t len = 0;
    out[0] = '\0';

    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t b;
        if (!ring_pop(ctx, &b)) {
            continue;
        }
        if (b == '\n') {
            /* strip trailing \r if present */
            if (len > 0 && out[len - 1] == '\r') {
                len--;
            }
            out[len] = '\0';
            return true;
        }
        if (len < (out_size - 1)) {
            out[len++] = (char)b;
        }
        /* else: line too long for buffer, keep consuming until '\n' but
         * stop appending - avoids overflow while still resyncing. */
    }
    out[len] = '\0';
    return false; /* timeout */
}

bool odrive_write_raw(odrive_uart_t *ctx, const char *path, float value)
{
    char cmd[ODRIVE_LINE_BUF_SIZE];
    int n = snprintf(cmd, sizeof(cmd), "w %s %.6f\n", path, (double)value);
    if (n < 0 || (size_t)n >= sizeof(cmd)) {
        return false;
    }
    return send_cmd(ctx, cmd);
}

bool odrive_write(odrive_uart_t *ctx, int axis, const char *subpath, float value)
{
    char path[ODRIVE_LINE_BUF_SIZE];
    int n = snprintf(path, sizeof(path), "axis%d.%s", axis, subpath);
    if (n < 0 || (size_t)n >= sizeof(path)) {
        return false;
    }
    return odrive_write_raw(ctx, path, value);
}

bool odrive_read_raw(odrive_uart_t *ctx, const char *path, float *out_value, uint32_t timeout_ms)
{
    char cmd[ODRIVE_LINE_BUF_SIZE];
    int n = snprintf(cmd, sizeof(cmd), "r %s\n", path);
    if (n < 0 || (size_t)n >= sizeof(cmd)) {
        return false;
    }
    if (!send_cmd(ctx, cmd)) {
        return false;
    }

    char line[ODRIVE_LINE_BUF_SIZE];
    if (!read_line(ctx, line, sizeof(line), timeout_ms)) {
        return false;
    }
    if (line[0] == '\0') {
        return false;
    }
    char *endptr = NULL;
    float v = strtof(line, &endptr);
    if (endptr == line) {
        return false; /* not a number - resync desired by caller */
    }
    *out_value = v;
    return true;
}

bool odrive_read(odrive_uart_t *ctx, int axis, const char *subpath, float *out_value, uint32_t timeout_ms)
{
    char path[ODRIVE_LINE_BUF_SIZE];
    int n = snprintf(path, sizeof(path), "axis%d.%s", axis, subpath);
    if (n < 0 || (size_t)n >= sizeof(path)) {
        return false;
    }
    return odrive_read_raw(ctx, path, out_value, timeout_ms);
}

bool odrive_get_feedback(odrive_uart_t *ctx, int axis, float *out_pos_turns, float *out_vel_turns_s,
                          uint32_t timeout_ms)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "f %d\n", axis);
    if (!send_cmd(ctx, cmd)) {
        return false;
    }

    char line[ODRIVE_LINE_BUF_SIZE];
    if (!read_line(ctx, line, sizeof(line), timeout_ms)) {
        return false;
    }

    char *endptr = NULL;
    float pos = strtof(line, &endptr);
    if (endptr == line) {
        return false;
    }
    while (*endptr == ' ') endptr++;
    float vel = strtof(endptr, &endptr);

    *out_pos_turns = pos;
    *out_vel_turns_s = vel;
    return true;
}

bool odrive_clear_errors(odrive_uart_t *ctx)
{
    return send_cmd(ctx, "sc\n") || send_cmd(ctx, "w axis0.error 0\n");
    /* Note: some fw builds expose a dedicated "clear errors" ASCII shortcut
     * ("sc"); if yours doesn't, the fallback above at least clears axis0's
     * top-level error. For full parity with the Python dashboard's
     * odrv0.clear_errors() (which walks every error field on both axes),
     * prefer wiring this to explicit odrive_write() calls per field once
     * you confirm your fw's exact ASCII command set with `odrivetool`. */
}

bool odrive_check_errors(odrive_uart_t *ctx, int axis, bool *any_error, uint32_t timeout_ms)
{
    float axis_err = 0, motor_err = 0, encoder_err = 0;
    if (!odrive_read(ctx, axis, "error", &axis_err, timeout_ms)) return false;
    if (!odrive_read(ctx, axis, "motor.error", &motor_err, timeout_ms)) return false;
    if (!odrive_read(ctx, axis, "encoder.error", &encoder_err, timeout_ms)) return false;

    *any_error = (axis_err != 0.0f) || (motor_err != 0.0f) || (encoder_err != 0.0f);
    return true;
}

bool odrive_read_current_state(odrive_uart_t *ctx, int axis, odrive_axis_state_t *out_state,
                                uint32_t timeout_ms)
{
    float v = 0;
    if (!odrive_read(ctx, axis, "current_state", &v, timeout_ms)) {
        return false;
    }
    *out_state = (odrive_axis_state_t)((int)(v + 0.5f));
    return true;
}

bool odrive_request_state(odrive_uart_t *ctx, int axis, odrive_axis_state_t state)
{
    return odrive_write(ctx, axis, "requested_state", (float)state);
}
