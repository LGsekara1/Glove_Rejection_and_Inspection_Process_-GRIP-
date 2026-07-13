#ifndef SCARA_APP_H
#define SCARA_APP_H

#include "odrive_uart.h"
#include "nextion_uart.h"
#include "kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Change these two to match your CubeMX-generated handles ---- */
#define ODRIVE_UART_HANDLE   huart5
#define NEXTION_UART_HANDLE  huart4

/* ---- Nextion Page 0 component IDs (must match NEXTION_HMI_DESIGN.md) ---- */
#define NX_PAGE_STATUS        0
#define NX_COMP_T_TITLE       0
#define NX_COMP_T_STATUS      1
#define NX_COMP_T_TH1         2
#define NX_COMP_T_TH2         3
#define NX_COMP_T_X           4
#define NX_COMP_T_Y           5
#define NX_COMP_T_LOG         6
#define NX_COMP_B_HOME0       7
#define NX_COMP_B_HOME1       8
#define NX_COMP_B_HOMEBOTH    9
#define NX_COMP_B_ENABLE      10
#define NX_COMP_B_IDLE        11
#define NX_COMP_B_CLEARERR    12
#define NX_COMP_B_ESTOP       13

typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_HOMING_AXIS0,
    APP_STATE_HOMING_AXIS1,
    APP_STATE_HOMING_BOTH_STEP0, /* homing axis0 as part of "Home Both" */
    APP_STATE_HOMING_BOTH_STEP1, /* homing axis1 as part of "Home Both" */
} app_state_t;

/* Call once at startup, after both UART peripherals are Init'd by CubeMX
 * and BEFORE arming HAL_UART_Receive_IT for either. */
void scara_app_init(void);

/* Call every super-loop iteration (or from a ~50-100 Hz RTOS task).
 * Non-blocking except for the odrive_uart.c read calls it makes
 * periodically, which have bounded timeouts (see ODRIVE_DEFAULT_TIMEOUT_MS). */
void scara_app_poll(void);

/* Expose the two driver contexts so main.c's HAL_UART_RxCpltCallback can
 * feed bytes in (see the wiring snippets at the top of odrive_uart.c /
 * nextion_uart.c). */
extern odrive_uart_t g_odrive_ctx;
extern nextion_uart_t g_nextion_ctx;

#ifdef __cplusplus
}
#endif

#endif /* SCARA_APP_H */
