/* odrive_link.h */
#ifndef ODRIVE_LINK_H
#define ODRIVE_LINK_H

#include <stdbool.h>
#include "stm32h7xx_hal.h"

void ODriveLink_Init(UART_HandleTypeDef *huart);

bool ODriveLink_Ping(char *reply_out, int reply_out_size);

bool ODriveLink_WriteIntProperty(const char *property, int value);

/* Position command with velocity feed-forward (turns, turns/s). cur_ff left 0. */
bool ODriveLink_SetPosition(uint8_t axis, float pos_turns, float vel_ff_turns_s);

/* Generic property write: "w axis0.requested_state 8\r\n" */
bool ODriveLink_WriteProperty(const char *property, float value);

/* Generic property read: sends "r <property>\r\n", parses the numeric reply. */
bool ODriveLink_ReadProperty(const char *property, float *value_out);

bool ODriveLink_ReadEncoderPos(uint8_t axis, float *turns_out);
bool ODriveLink_ReadAxisState(uint8_t axis, int *state_out);
bool ODriveLink_RequestClosedLoop(uint8_t axis);
bool ODriveLink_RequestIdle(uint8_t axis);

#endif
