/* odrive_link.c */
#include "odrive_link.h"
#include "app_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static UART_HandleTypeDef *s_huart;
#define TX_TIMEOUT_MS   20
#define RX_TIMEOUT_MS   50
#define RX_BUF_LEN      64

void ODriveLink_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    __HAL_UART_CLEAR_OREFLAG(s_huart);
}\

static bool uart_send_line(const char *line, uint16_t len)
{
    return HAL_UART_Transmit(s_huart, (uint8_t *)line, len, TX_TIMEOUT_MS) == HAL_OK;
}

/* Reads bytes until '\n' or timeout. Returns number of bytes stored
   (excluding CR/LF), or -1 on timeout/error. */
static int uart_read_line(char *buf, int buf_size)
{
    __HAL_UART_CLEAR_OREFLAG(s_huart);   /* clear any latched overrun first */
    memset(buf, 0, buf_size);

    HAL_UART_Receive(s_huart, (uint8_t *)buf, buf_size - 1, RX_TIMEOUT_MS);

    /* HAL fills the buffer with whatever it received even on timeout;
       RxXferCount tells us how many bytes are still "missing" from the
       requested size, so subtract to get how many actually arrived. */
    int received = (buf_size - 1) - s_huart->RxXferCount;
    if (received <= 0) return -1;

    for (int i = 0; i < received; i++) {
        if (buf[i] == '\n') {
            buf[i] = '\0';
            if (i > 0 && buf[i - 1] == '\r') buf[i - 1] = '\0';
            return i;
        }
    }
    buf[received] = '\0';
    return received;
}

bool ODriveLink_SetPosition(uint8_t axis, float pos_turns, float vel_ff_turns_s)
{
    char line[64];
    int len = snprintf(line, sizeof(line), "p %u %.6f %.6f 0\n",
                        (unsigned)axis, pos_turns, vel_ff_turns_s);
    return uart_send_line(line, (uint16_t)len);
}

bool ODriveLink_WriteProperty(const char *property, float value)
{
    char line[80];
    int len = snprintf(line, sizeof(line), "w %s %.6f\n", property, value);
    return uart_send_line(line, (uint16_t)len);
}

bool ODriveLink_WriteIntProperty(const char *property, int value)
{
    char line[80];
    int len = snprintf(line, sizeof(line), "w %s %d\n", property, value);
    return uart_send_line(line, (uint16_t)len);
}

bool ODriveLink_ReadProperty(const char *property, float *value_out)
{
    char cmd[64];
    int clen = snprintf(cmd, sizeof(cmd), "r %s\n", property);
    if (!uart_send_line(cmd, (uint16_t)clen)) return false;

    char reply[RX_BUF_LEN];
    int n = uart_read_line(reply, sizeof(reply));
    if (n < 0) return false;

    char *endptr;
    float val = strtof(reply, &endptr);
    if (endptr == reply) return false;
    *value_out = val;
    return true;
}

bool ODriveLink_ReadEncoderPos(uint8_t axis, float *turns_out)
{
    char prop[32];
    snprintf(prop, sizeof(prop), "axis%u.encoder.pos_estimate", (unsigned)axis);
    return ODriveLink_ReadProperty(prop, turns_out);
}

bool ODriveLink_ReadAxisState(uint8_t axis, int *state_out)
{
    char prop[32];
    snprintf(prop, sizeof(prop), "axis%u.current_state", (unsigned)axis);
    float v;
    if (!ODriveLink_ReadProperty(prop, &v)) return false;
    *state_out = (int)v;
    return true;
}

bool ODriveLink_Ping(char *reply_out, int reply_out_size)
{
    const char *cmd = "r vbus_voltage\n";
    if (!uart_send_line(cmd, (uint16_t)strlen(cmd))) return false;
    int n = uart_read_line(reply_out, reply_out_size);
    return (n > 0);
}

bool ODriveLink_RequestClosedLoop(uint8_t axis)
{
    char ctrl_mode_prop[40], input_mode_prop[40], state_prop[40];
    snprintf(ctrl_mode_prop, sizeof(ctrl_mode_prop), "axis%u.controller.config.control_mode", axis);
    snprintf(input_mode_prop, sizeof(input_mode_prop), "axis%u.controller.config.input_mode", axis);
    snprintf(state_prop, sizeof(state_prop), "axis%u.requested_state", axis);

    bool ok = true;
    ok &= ODriveLink_WriteIntProperty(ctrl_mode_prop, ODRIVE_CONTROL_MODE_POSITION_CONTROL);
    ok &= ODriveLink_WriteIntProperty(input_mode_prop, ODRIVE_INPUT_MODE_PASSTHROUGH);
    ok &= ODriveLink_WriteIntProperty(state_prop, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
    return ok;
}

bool ODriveLink_RequestIdle(uint8_t axis)
{
    char state_prop[40];
    snprintf(state_prop, sizeof(state_prop), "axis%u.requested_state", axis);
    return ODriveLink_WriteIntProperty(state_prop, ODRIVE_AXIS_STATE_IDLE);
}
