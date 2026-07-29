/* nextion_display.c
 *
 * Polls ODrive axis state (idle / closed loop / calibrating / etc.) and
 * displays it as text on the Nextion NX3224F028_011.
 *
 * Assumes Nextion page has two text components named "t0" and "t1"
 * (axis0 status, axis1 status). Rename in StateToString()/PollOdriveState()
 * calls below if your components use different names/IDs.
 */

#include "nextion_display.h"
#include "odrive_link.h"
#include "app_config.h"   /* for ODRIVE_AXIS0_IDX / ODRIVE_AXIS1_IDX */
#include <string.h>
#include <stdio.h>
#include "app_log.h"
#include <stddef.h>
#include "cdc_jobs.h"

static UART_HandleTypeDef *s_nx_uart = NULL;
static const uint8_t NX_TERM[3] = { 0xFF, 0xFF, 0xFF };

/* --- Touch event handling -------------------------------------------- */

/* Nextion touch event packet: 0x65, page_id, component_id, event, FF FF FF
 * event = 0x01 (press) or 0x00 (release). We act on release only. */
#define NX_EVT_PRESS   0x01
#define NX_EVT_RELEASE 0x00

/* TODO: set these to match the actual page/component IDs assigned to your
 * Idle and Closed Loop buttons in the Nextion Editor (Attribute panel	). */
#define NX_PAGE_MAIN           0
#define NX_BTN_ID_IDLE         4
#define NX_BTN_ID_CLOSED_LOOP  5
#define NX_BTN_ID_06 6
#define NX_BTN_ID_07 7
#define NX_BTN_ID_08 8
#define NX_BTN_ID_09 9
#define NX_BTN_ID_10 10
#define NX_BTN_ID_11 11
#define NX_BTN_ID_12 12
#define NX_BTN_ID_13 13
#define NX_BTN_ID_14 14
#define NX_BTN_ID_15 15
#define NX_BTN_ID_16 16
#define NX_BTN_ID_17 17

static uint8_t s_nx_rx_byte;
static uint8_t s_nx_pkt[7];
static uint8_t s_nx_pkt_idx = 0;

/* Standard ODrive firmware axis states (AXIS_STATE_*). Values match what
 * odrive_link.c / motion.c already compare against
 * (e.g. ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL == 8). */
typedef enum {
    ODRV_STATE_UNDEFINED                  = 0,
    ODRV_STATE_IDLE                       = 1,
    ODRV_STATE_STARTUP_SEQUENCE           = 2,
    ODRV_STATE_FULL_CALIBRATION_SEQUENCE  = 3,
    ODRV_STATE_MOTOR_CALIBRATION          = 4,
    ODRV_STATE_ENCODER_INDEX_SEARCH       = 6,
    ODRV_STATE_ENCODER_OFFSET_CALIBRATION = 7,
    ODRV_STATE_CLOSED_LOOP_CONTROL        = 8,
    ODRV_STATE_LOCKIN_SPIN                = 9,
    ODRV_STATE_ENCODER_DIR_FIND           = 10,
    ODRV_STATE_HOMING                     = 11,
    ODRV_STATE_ENCODER_HALL_POLARITY_CAL  = 12,
    ODRV_STATE_ENCODER_HALL_PHASE_CAL     = 13,
} odrive_axis_state_t;

static void nx_send(const char *cmd)
{
    if (!s_nx_uart) return;
    HAL_UART_Transmit(s_nx_uart, (uint8_t *)cmd, strlen(cmd), 100);
    HAL_UART_Transmit(s_nx_uart, (uint8_t *)NX_TERM, 3, 100);
}

static const char *StateToString(int state)
{
    switch (state) {
        case ODRV_STATE_IDLE:                      return "IDLE";
        case ODRV_STATE_STARTUP_SEQUENCE:           return "STARTUP";
        case ODRV_STATE_FULL_CALIBRATION_SEQUENCE:  return "CALIBRATING";
        case ODRV_STATE_MOTOR_CALIBRATION:          return "MOTOR CAL";
        case ODRV_STATE_ENCODER_INDEX_SEARCH:       return "ENC INDEX";
        case ODRV_STATE_ENCODER_OFFSET_CALIBRATION: return "ENC OFFSET";
        case ODRV_STATE_CLOSED_LOOP_CONTROL:        return "CLOSED LOOP";
        case ODRV_STATE_LOCKIN_SPIN:                return "LOCKIN SPIN";
        case ODRV_STATE_ENCODER_DIR_FIND:           return "ENC DIR FIND";
        case ODRV_STATE_HOMING:                     return "HOMING";
        case ODRV_STATE_ENCODER_HALL_POLARITY_CAL:  return "HALL POL CAL";
        case ODRV_STATE_ENCODER_HALL_PHASE_CAL:     return "HALL PHASE CAL";
        case ODRV_STATE_UNDEFINED:
        default:                                    return "UNDEFINED";
    }
}

void NextionDisplay_Init(UART_HandleTypeDef *huart)
{
    s_nx_uart = huart;
    HAL_UART_Receive_IT(s_nx_uart, &s_nx_rx_byte, 1); /* arm for first touch-event byte */
}

void NextionDisplay_PollOdriveState(void)
{
    int st0 = -1, st1 = -1;

    bool got0 = ODriveLink_ReadAxisState(ODRIVE_AXIS0_IDX, &st0);
    bool got1 = ODriveLink_ReadAxisState(ODRIVE_AXIS1_IDX, &st1);

    char cmd[48];


    if (got0) {
        snprintf(cmd, sizeof(cmd), "t0.txt=\"%s\"", StateToString(st0));

    } else {
        snprintf(cmd, sizeof(cmd), "t0.txt=\"NO REPLY - Axis 0\"");
        cdc_log("No Reply Axis 0");

    }
    nx_send(cmd);

    if (got1) {
        snprintf(cmd, sizeof(cmd), "t1.txt=\"%s\"", StateToString(st1));

    } else {
        snprintf(cmd, sizeof(cmd), "t1.txt=\"NO REPLY - Axis 1\"");
        cdc_log("No Reply Axis 2");
    }
    nx_send(cmd);
}

typedef void (*nx_button_handler_t)(void);

static void Button_Idle(void)
{
    bool ok0 = ODriveLink_RequestIdle(ODRIVE_AXIS0_IDX);
    bool ok1 = ODriveLink_RequestIdle(ODRIVE_AXIS1_IDX);
    cdc_log("Idle button: axis0=%s axis1=%s\r\n",
            ok0 ? "OK" : "FAIL", ok1 ? "OK" : "FAIL");
}

static void Button_ClosedLoop(void)
{
    bool ok0 = ODriveLink_RequestClosedLoop(ODRIVE_AXIS0_IDX);
    bool ok1 = ODriveLink_RequestClosedLoop(ODRIVE_AXIS1_IDX);
    cdc_log("Closed loop button: axis0=%s axis1=%s\r\n",
            ok0 ? "OK" : "FAIL", ok1 ? "OK" : "FAIL");
}

static void Button_06(void) { CdcJobs_InjectLine("249, 500"); }
static void Button_07(void) { CdcJobs_InjectLine("123, 493"); }
static void Button_08(void) { CdcJobs_InjectLine("-8, 497"); }
static void Button_09(void) { CdcJobs_InjectLine("-140, 500"); }
static void Button_10(void) { CdcJobs_InjectLine("235, 391"); }
static void Button_11(void) { CdcJobs_InjectLine("110, 404"); }
static void Button_12(void) { CdcJobs_InjectLine("-24, 396"); }
static void Button_13(void) { CdcJobs_InjectLine("-145, 409"); }
static void Button_14(void) { CdcJobs_InjectLine("250, 312"); }
static void Button_15(void) { CdcJobs_InjectLine("114, 307"); }
static void Button_16(void) { CdcJobs_InjectLine("-24, 300"); }
static void Button_17(void) { CdcJobs_InjectLine("-166, 316"); }

typedef struct {
    uint8_t component_id;
    nx_button_handler_t handler;
} nx_button_map_t;

static const nx_button_map_t s_button_map[] = {
    { NX_BTN_ID_IDLE,        Button_Idle },
    { NX_BTN_ID_CLOSED_LOOP, Button_ClosedLoop },
    { NX_BTN_ID_06,          Button_06 },
    { NX_BTN_ID_07,          Button_07 },
    { NX_BTN_ID_08,          Button_08 },
    { NX_BTN_ID_09,          Button_09 },
    { NX_BTN_ID_10,          Button_10 },
    { NX_BTN_ID_11,          Button_11 },
    { NX_BTN_ID_12,          Button_12 },
    { NX_BTN_ID_13,          Button_13 },
    { NX_BTN_ID_14,          Button_14 },
    { NX_BTN_ID_15,          Button_15 },
    { NX_BTN_ID_16,          Button_16 },
	{ NX_BTN_ID_17,          Button_17 },
};
#define NX_BUTTON_MAP_COUNT (sizeof(s_button_map) / sizeof(s_button_map[0]))

#define NX_BUTTON_DEBOUNCE_MS 250

static uint32_t s_last_press_tick[NX_BUTTON_MAP_COUNT];

static void HandleButtonEvent(uint8_t page, uint8_t component_id, uint8_t event)
{
    if (event != NX_EVT_RELEASE) return; /* ignore press, act on release */
    if (page != NX_PAGE_MAIN) return;

    for (size_t i = 0; i < NX_BUTTON_MAP_COUNT; i++) {
        if (s_button_map[i].component_id == component_id) {
            uint32_t now = HAL_GetTick();
            if ((now - s_last_press_tick[i]) < NX_BUTTON_DEBOUNCE_MS) {
                return; /* too soon after the last accepted press - ignore */
            }
            s_last_press_tick[i] = now;
            s_button_map[i].handler();
            return;
        }
    }
    /* No mapped handler for this component_id - ignore silently. */
}

void NextionDisplay_UART_RxByteISR(void)
{
    uint8_t b = s_nx_rx_byte;

    if (s_nx_pkt_idx == 0 && b != 0x65) {
        /* not a touch-event start byte - ignore and keep waiting */
    } else {
        s_nx_pkt[s_nx_pkt_idx++] = b;

        if (s_nx_pkt_idx == 7) {
            if (s_nx_pkt[4] == 0xFF && s_nx_pkt[5] == 0xFF && s_nx_pkt[6] == 0xFF) {
                HandleButtonEvent(s_nx_pkt[1], s_nx_pkt[2], s_nx_pkt[3]);
            }
            s_nx_pkt_idx = 0;
        }
    }

    HAL_UART_Receive_IT(s_nx_uart, &s_nx_rx_byte, 1); /* re-arm for next byte */
}
