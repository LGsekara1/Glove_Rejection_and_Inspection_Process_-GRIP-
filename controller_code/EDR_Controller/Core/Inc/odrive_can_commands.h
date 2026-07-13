#ifndef ODRIVE_CAN_COMMANDS_H
#define ODRIVE_CAN_COMMANDS_H

#include "main.h"

// ODrive CAN Protocol v0.5.6 Command IDs

#define CMD_CANOPEN_NMT              0x000
#define CMD_HEARTBEAT                0x001
#define CMD_ESTOP                    0x002
#define CMD_GET_MOTOR_ERROR          0x003
#define CMD_GET_ENCODER_ERROR        0x004
#define CMD_GET_SENSORLESS_ERROR     0x005
#define CMD_SET_AXIS_NODE_ID         0x006
#define CMD_SET_AXIS_REQUESTED_STATE 0x007
#define CMD_SET_AXIS_STARTUP_CONFIG  0x008
#define CMD_GET_ENCODER_ESTIMATES    0x009
#define CMD_GET_ENCODER_COUNT        0x00A
#define CMD_SET_CONTROLLER_MODES     0x00B
#define CMD_SET_INPUT_POS            0x00C
#define CMD_SET_INPUT_VEL            0x00D
#define CMD_SET_INPUT_TORQUE         0x00E
#define CMD_SET_LIMITS               0x00F
#define CMD_START_ANTICOGGING        0x010
#define CMD_SET_TRAJ_VEL_LIMIT       0x011
#define CMD_SET_TRAJ_ACCEL_LIMITS    0x012
#define CMD_SET_TRAJ_INERTIA         0x013
#define CMD_GET_IQ                   0x014
#define CMD_GET_SENSORLESS_ESTIMATES 0x015
#define CMD_REBOOT                   0x016
#define CMD_GET_BUS_VOLTAGE_AND_CURRENT 0x017
#define CMD_CLEAR_ERRORS 			 0x018

void ODrive_CAN_Send(uint8_t axis_id,
                     uint8_t cmd_id,
                     uint8_t *data,
                     uint8_t length);

uint32_t ODrive_Get_CAN_ID(uint8_t axis_id, uint32_t cmd_id);

#endif // ODRIVE_CAN_COMMANDS_H
