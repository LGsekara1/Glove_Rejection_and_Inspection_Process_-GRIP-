/*
 * nextion_uart.h
 *
 * Minimal driver for the Nextion display serial protocol (works for
 * Basic/Enhanced series alike, including the NX3224F028_011).
 *
 * TX: component.attr=value style commands, each terminated by 0xFF 0xFF 0xFF.
 * RX: touch events as 0x65 <page_id> <component_id> <event> 0xFF 0xFF 0xFF,
 *     where event = 0x01 (press) or 0x00 (release). Only emitted per-
 *     component if that component's "Send Component ID" attribute is
 *     checked in Nextion Editor - see NEXTION_HMI_DESIGN.md.
 */

#ifndef NEXTION_UART_H
#define NEXTION_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NEXTION_RX_BUF_SIZE   128
#define NEXTION_EVENT_QUEUE_LEN 8

typedef struct {
    uint8_t page_id;
    uint8_t component_id;
    uint8_t event; /* 1 = press, 0 = release */
} nextion_touch_event_t;

typedef struct {
    UART_HandleTypeDef *huart;

    volatile uint8_t rx_buf[NEXTION_RX_BUF_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    uint8_t rx_isr_byte;

    /* Small in-order queue of fully-parsed touch events, filled by
     * nextion_uart_poll() (called from the app's main loop, NOT the ISR -
     * parsing is done outside interrupt context to keep the ISR itself
     * tiny). App code drains it with nextion_uart_pop_event(). */
    nextion_touch_event_t event_queue[NEXTION_EVENT_QUEUE_LEN];
    volatile uint8_t evq_head;
    volatile uint8_t evq_tail;
} nextion_uart_t;

void nextion_uart_init(nextion_uart_t *ctx, UART_HandleTypeDef *huart);

/* Feed one byte at a time from HAL_UART_RxCpltCallback for the Nextion
 * UART instance (mirrors odrive_uart_rx_byte_isr - see odrive_uart.h). */
void nextion_uart_rx_byte_isr(nextion_uart_t *ctx, uint8_t byte);

/* Call every main-loop iteration. Scans the raw ring buffer for complete
 * 0x65 ... 0xFF 0xFF 0xFF touch-event packets and pushes them into
 * event_queue. Non-blocking. */
void nextion_uart_poll(nextion_uart_t *ctx);

/* Pops the oldest queued touch event. Returns false if none pending. */
bool nextion_uart_pop_event(nextion_uart_t *ctx, nextion_touch_event_t *out_event);

/* Sets a Text component's .txt attribute, e.g.
 *   nextion_set_text(ctx, "t_status", "Connected");
 * -> sends: t_status.txt="Connected" 0xFF 0xFF 0xFF */
bool nextion_set_text(nextion_uart_t *ctx, const char *component_name, const char *text);

/* printf-style convenience wrapper around nextion_set_text(). */
bool nextion_set_text_fmt(nextion_uart_t *ctx, const char *component_name, const char *fmt, ...);

/* Sets a numeric-ish attribute, e.g. nextion_set_attr(ctx, "b_estop", "bco", 63488) */
bool nextion_set_attr(nextion_uart_t *ctx, const char *component_name, const char *attr, int32_t value);

/* Switches page, e.g. nextion_goto_page(ctx, 0) -> "page 0" */
bool nextion_goto_page(nextion_uart_t *ctx, uint8_t page_id);

#ifdef __cplusplus
}
#endif

#endif /* NEXTION_UART_H */
