/*
 * scara_app.c
 *
 * Ties odrive_uart + nextion_uart + kinematics together for the first
 * screen: per-axis / combined homing, plus a live status readout (joint
 * angles, end-effector X/Y, connection/error state).
 *
 * Extending later for Joint/IK/Path screens: add new Nextion buttons per
 * NEXTION_HMI_DESIGN.md, new component #defines in scara_app.h, and new
 * `case NX_COMP_...:` arms in handle_touch_event() below - reuse
 * odrive_write()/odrive_read()/odrive_get_feedback() exactly as this file
 * does. The one thing you'll want to add for smooth multi-point moves is a
 * trapezoidal setpoint streamer (see the Python dashboard's
 * synchronized_two_axis_profile / _stream_joint_trajectory_blocking) driven
 * from a periodic timer interrupt instead of a background thread - there's
 * no OS thread pool here, so that's the bare-metal equivalent.
 */

#include "scara_app.h"
#include <string.h>
#include <stdio.h>

/* Provided by CubeMX-generated code (usart.c) */
extern UART_HandleTypeDef ODRIVE_UART_HANDLE;
extern UART_HandleTypeDef NEXTION_UART_HANDLE;

odrive_uart_t g_odrive_ctx;
nextion_uart_t g_nextion_ctx;

/* ---- Link geometry / axis config: fill these in with YOUR real values,
 * same numbers as the Python dashboard's self.params / self.axis_cfg /
 * self.home_angle_deg (or load from flash/EEPROM if you have persistence
 * on this board - out of scope here). ---- */
static const link_params_t s_link_params = {
    .L0 = 300.0f,
    .l1a = 300.0f,
    .l2a = 450.0f,
    .l1b = 300.0f,
    .l2b = 450.0f,
    .fk_branch_upper = true,
};

static axis_cfg_t s_axis_cfg[2] = {
    { .gear_ratio = 1.0f, .offset_turns = 0.0f, .direction = -1.0f }, /* axis0 */
    { .gear_ratio = 1.0f, .offset_turns = 0.0f, .direction = -1.0f }, /* axis1 */
};
static float s_home_angle_deg = 90.0f;

static app_state_t s_state = APP_STATE_IDLE;
static uint32_t s_homing_start_tick = 0;
#define HOMING_TIMEOUT_MS 60000

static uint32_t s_last_status_refresh_tick = 0;
#define STATUS_REFRESH_INTERVAL_MS 200

static bool s_odrive_connected = false; /* set true once a feedback read succeeds */

static void log_line(const char *msg)
{
    nextion_set_text(&g_nextion_ctx, "t_log", msg);
}

static void set_status_text(const char *msg)
{
    nextion_set_text(&g_nextion_ctx, "t_status", msg);
}

/* ------------------------------------------------------------------ */
/* Homing state machine                                                */
/* ------------------------------------------------------------------ */

static void start_homing_axis(int axis)
{
    if (!odrive_request_state(&g_odrive_ctx, axis, ODRIVE_AXIS_STATE_HOMING)) {
        log_line("Homing: failed to send request");
        return;
    }
    s_homing_start_tick = HAL_GetTick();
    char buf[48];
    snprintf(buf, sizeof(buf), "Homing axis%d...", axis);
    log_line(buf);
    set_status_text(axis == 0 ? "Homing axis0" : "Homing axis1");
}

/* Returns true once the axis has returned to IDLE (homing complete or
 * failed - caller checks errors separately). Returns false while still
 * homing or on timeout (timeout also returns true so the state machine
 * doesn't hang forever, with an error logged). */
static bool poll_homing_axis(int axis)
{
    odrive_axis_state_t state;
    if (!odrive_read_current_state(&g_odrive_ctx, axis, &state, ODRIVE_DEFAULT_TIMEOUT_MS)) {
        return false; /* transient read miss, try again next poll */
    }

    if (state == ODRIVE_AXIS_STATE_IDLE) {
        bool any_error = false;
        char buf[64];
        if (odrive_check_errors(&g_odrive_ctx, axis, &any_error, ODRIVE_DEFAULT_TIMEOUT_MS) && !any_error) {
            snprintf(buf, sizeof(buf), "axis%d: homing OK", axis);
        } else {
            snprintf(buf, sizeof(buf), "axis%d: homing finished w/ errors", axis);
        }
        log_line(buf);
        return true;
    }

    if ((HAL_GetTick() - s_homing_start_tick) > HOMING_TIMEOUT_MS) {
        char buf[48];
        snprintf(buf, sizeof(buf), "axis%d: homing TIMEOUT", axis);
        log_line(buf);
        return true;
    }

    return false; /* still homing */
}

/* ------------------------------------------------------------------ */
/* Touch event handling                                                 */
/* ------------------------------------------------------------------ */

static void handle_touch_event(const nextion_touch_event_t *ev)
{
    /* Only react on release, per NEXTION_HMI_DESIGN.md. */
    if (ev->event != 0) {
        return;
    }

    switch (ev->component_id) {
        case NX_COMP_B_HOME0:
            if (s_state == APP_STATE_IDLE) {
                s_state = APP_STATE_HOMING_AXIS0;
                start_homing_axis(0);
            }
            break;

        case NX_COMP_B_HOME1:
            if (s_state == APP_STATE_IDLE) {
                s_state = APP_STATE_HOMING_AXIS1;
                start_homing_axis(1);
            }
            break;

        case NX_COMP_B_HOMEBOTH:
            /* Sequential, mirrors the Python dashboard's home_both(): axis0
             * fully to completion, then axis1 - never both moving at once. */
            if (s_state == APP_STATE_IDLE) {
                s_state = APP_STATE_HOMING_BOTH_STEP0;
                start_homing_axis(0);
            }
            break;

        case NX_COMP_B_ENABLE:
            if (s_state == APP_STATE_IDLE) {
                odrive_request_state(&g_odrive_ctx, 0, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
                odrive_request_state(&g_odrive_ctx, 1, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
                log_line("Enable requested (both axes)");
            }
            break;

        case NX_COMP_B_IDLE:
            if (s_state == APP_STATE_IDLE) {
                odrive_request_state(&g_odrive_ctx, 0, ODRIVE_AXIS_STATE_IDLE);
                odrive_request_state(&g_odrive_ctx, 1, ODRIVE_AXIS_STATE_IDLE);
                log_line("Idle requested (both axes)");
            }
            break;

        case NX_COMP_B_CLEARERR:
            odrive_clear_errors(&g_odrive_ctx);
            log_line("Errors cleared");
            break;

        case NX_COMP_B_ESTOP:
            /* Always allowed, regardless of s_state - mirrors the Python
             * dashboard's emergency_stop() being reachable at any time. */
            odrive_request_state(&g_odrive_ctx, 0, ODRIVE_AXIS_STATE_IDLE);
            odrive_request_state(&g_odrive_ctx, 1, ODRIVE_AXIS_STATE_IDLE);
            s_state = APP_STATE_IDLE;
            log_line("EMERGENCY STOP");
            set_status_text("E-STOPPED - both axes idle");
            break;

        default:
            break;
    }
}

/* ------------------------------------------------------------------ */
/* Status screen refresh                                                */
/* ------------------------------------------------------------------ */

static void refresh_status_screen(void)
{
    float pos0, vel0, pos1, vel1;
    bool ok0 = odrive_get_feedback(&g_odrive_ctx, 0, &pos0, &vel0, ODRIVE_DEFAULT_TIMEOUT_MS);
    bool ok1 = ok0 && odrive_get_feedback(&g_odrive_ctx, 1, &pos1, &vel1, ODRIVE_DEFAULT_TIMEOUT_MS);

    if (!ok0 || !ok1) {
        if (s_odrive_connected) {
            /* only spam the "disconnected" state change once */
            set_status_text("ODrive: NOT RESPONDING");
            s_odrive_connected = false;
        }
        return;
    }

    if (!s_odrive_connected) {
        set_status_text("ODrive: Connected");
        s_odrive_connected = true;
    }

    float th1 = turns_to_joint_deg(pos0, &s_axis_cfg[0], s_home_angle_deg);
    float th2 = turns_to_joint_deg(pos1, &s_axis_cfg[1], s_home_angle_deg);

    nextion_set_text_fmt(&g_nextion_ctx, "t_th1", "th1: %.1f deg", (double)th1);
    nextion_set_text_fmt(&g_nextion_ctx, "t_th2", "th2: %.1f deg", (double)th2);

    float x, y;
    if (forward_kinematics(th1, th2, &s_link_params, &x, &y)) {
        nextion_set_text_fmt(&g_nextion_ctx, "t_x", "X: %.1f mm", (double)x);
        nextion_set_text_fmt(&g_nextion_ctx, "t_y", "Y: %.1f mm", (double)y);
    } else {
        nextion_set_text(&g_nextion_ctx, "t_x", "X: (invalid pose)");
        nextion_set_text(&g_nextion_ctx, "t_y", "Y: (invalid pose)");
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void scara_app_init(void)
{
    odrive_uart_init(&g_odrive_ctx, &ODRIVE_UART_HANDLE);
    nextion_uart_init(&g_nextion_ctx, &NEXTION_UART_HANDLE);

    /* Arm the first RX byte on both UARTs. */
    HAL_UART_Receive_IT(&ODRIVE_UART_HANDLE, &g_odrive_ctx.rx_isr_byte, 1);
    HAL_UART_Receive_IT(&NEXTION_UART_HANDLE, &g_nextion_ctx.rx_isr_byte, 1);

    nextion_goto_page(&g_nextion_ctx, NX_PAGE_STATUS);
    nextion_set_text(&g_nextion_ctx, "t_title", "SCARA Control");
    set_status_text("ODrive: connecting...");
    nextion_set_text(&g_nextion_ctx, "t_th1", "th1: --.- deg");
    nextion_set_text(&g_nextion_ctx, "t_th2", "th2: --.- deg");
    nextion_set_text(&g_nextion_ctx, "t_x", "X: --.- mm");
    nextion_set_text(&g_nextion_ctx, "t_y", "Y: --.- mm");
    log_line("Booted");
}

void scara_app_poll(void)
{
    nextion_uart_poll(&g_nextion_ctx);

    nextion_touch_event_t ev;
    while (nextion_uart_pop_event(&g_nextion_ctx, &ev)) {
        handle_touch_event(&ev);
    }

    switch (s_state) {
        case APP_STATE_IDLE:
            break;

        case APP_STATE_HOMING_AXIS0:
            if (poll_homing_axis(0)) {
                s_state = APP_STATE_IDLE;
                set_status_text("ODrive: Connected");
            }
            break;

        case APP_STATE_HOMING_AXIS1:
            if (poll_homing_axis(1)) {
                s_state = APP_STATE_IDLE;
                set_status_text("ODrive: Connected");
            }
            break;

        case APP_STATE_HOMING_BOTH_STEP0:
            if (poll_homing_axis(0)) {
                s_state = APP_STATE_HOMING_BOTH_STEP1;
                start_homing_axis(1);
            }
            break;

        case APP_STATE_HOMING_BOTH_STEP1:
            if (poll_homing_axis(1)) {
                s_state = APP_STATE_IDLE;
                log_line("Home Both: complete");
                set_status_text("ODrive: Connected");
            }
            break;
    }

    /* Status refresh: skip while a homing move is in progress so we're not
     * interleaving 'r current_state' polls with 'f' feedback polls on the
     * same axis and confusing the line-based reply parser - mirrors the
     * Python dashboard's poll_live() backing off while _motion_active. */
    if (s_state == APP_STATE_IDLE) {
        uint32_t now = HAL_GetTick();
        if ((now - s_last_status_refresh_tick) >= STATUS_REFRESH_INTERVAL_MS) {
            s_last_status_refresh_tick = now;
            refresh_status_screen();
        }
    }
}
