/*
 * nextion_uart.c
 *
 * main.c wiring (alongside the ODrive one):
 *
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 *   {
 *       if (huart->Instance == NEXTION_UART_INSTANCE) {
 *           nextion_uart_rx_byte_isr(&g_nextion_ctx, g_nextion_ctx.rx_isr_byte);
 *           HAL_UART_Receive_IT(huart, &g_nextion_ctx.rx_isr_byte, 1);
 *       }
 *       ...
 *   }
 */

#include "nextion_uart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static void ring_push(nextion_uart_t *ctx, uint8_t byte)
{
    uint16_t next = (uint16_t)((ctx->rx_head + 1) % NEXTION_RX_BUF_SIZE);
    if (next == ctx->rx_tail) {
        ctx->rx_tail = (uint16_t)((ctx->rx_tail + 1) % NEXTION_RX_BUF_SIZE);
    }
    ctx->rx_buf[ctx->rx_head] = byte;
    ctx->rx_head = next;
}

static bool ring_peek_at(nextion_uart_t *ctx, uint16_t offset, uint8_t *out_byte)
{
    uint16_t avail = (uint16_t)((ctx->rx_head - ctx->rx_tail + NEXTION_RX_BUF_SIZE) % NEXTION_RX_BUF_SIZE);
    if (offset >= avail) {
        return false;
    }
    uint16_t idx = (uint16_t)((ctx->rx_tail + offset) % NEXTION_RX_BUF_SIZE);
    *out_byte = ctx->rx_buf[idx];
    return true;
}

static void ring_drop(nextion_uart_t *ctx, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++) {
        if (ctx->rx_tail == ctx->rx_head) break;
        ctx->rx_tail = (uint16_t)((ctx->rx_tail + 1) % NEXTION_RX_BUF_SIZE);
    }
}

void nextion_uart_init(nextion_uart_t *ctx, UART_HandleTypeDef *huart)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->huart = huart;
}

void nextion_uart_rx_byte_isr(nextion_uart_t *ctx, uint8_t byte)
{
    ring_push(ctx, byte);
}

static void push_event(nextion_uart_t *ctx, uint8_t page, uint8_t comp, uint8_t ev)
{
    uint8_t next = (uint8_t)((ctx->evq_head + 1) % NEXTION_EVENT_QUEUE_LEN);
    if (next == ctx->evq_tail) {
        return; /* queue full, drop - status refresh will still show truth on next poll */
    }
    ctx->event_queue[ctx->evq_head].page_id = page;
    ctx->event_queue[ctx->evq_head].component_id = comp;
    ctx->event_queue[ctx->evq_head].event = ev;
    ctx->evq_head = next;
}

void nextion_uart_poll(nextion_uart_t *ctx)
{
    /* Look for a 0x65 header, then require 6 more bytes total available:
     * page_id, component_id, event, 0xFF, 0xFF, 0xFF. If a header byte
     * shows up but the full packet isn't buffered yet, just return and try
     * again next poll (don't drop it). */
    for (;;) {
        uint8_t b0;
        if (!ring_peek_at(ctx, 0, &b0)) {
            return; /* buffer empty */
        }
        if (b0 != 0x65) {
            ring_drop(ctx, 1); /* resync: discard stray byte */
            continue;
        }

        uint8_t page, comp, ev, t1, t2, t3;
        if (!ring_peek_at(ctx, 1, &page)) return;
        if (!ring_peek_at(ctx, 2, &comp)) return;
        if (!ring_peek_at(ctx, 3, &ev)) return;
        if (!ring_peek_at(ctx, 4, &t1)) return;
        if (!ring_peek_at(ctx, 5, &t2)) return;
        if (!ring_peek_at(ctx, 6, &t3)) return;

        if (t1 == 0xFF && t2 == 0xFF && t3 == 0xFF) {
            push_event(ctx, page, comp, ev);
            ring_drop(ctx, 7);
        } else {
            /* Not a clean touch-event packet (or we're mid-stream from a
             * different reply type) - drop just the header byte and
             * resync from the next byte rather than the whole packet. */
            ring_drop(ctx, 1);
        }
    }
}

bool nextion_uart_pop_event(nextion_uart_t *ctx, nextion_touch_event_t *out_event)
{
    if (ctx->evq_tail == ctx->evq_head) {
        return false;
    }
    *out_event = ctx->event_queue[ctx->evq_tail];
    ctx->evq_tail = (uint8_t)((ctx->evq_tail + 1) % NEXTION_EVENT_QUEUE_LEN);
    return true;
}

static bool send_terminated(nextion_uart_t *ctx, const char *payload, size_t len)
{
    static const uint8_t term[3] = {0xFF, 0xFF, 0xFF};
    if (HAL_UART_Transmit(ctx->huart, (uint8_t *)payload, (uint16_t)len, 100) != HAL_OK) {
        return false;
    }
    return HAL_UART_Transmit(ctx->huart, (uint8_t *)term, 3, 100) == HAL_OK;
}

bool nextion_set_text(nextion_uart_t *ctx, const char *component_name, const char *text)
{
    char buf[160];
    /* Nextion strings use double quotes; if `text` might itself contain a
     * '"' or backslash you'd need to escape it here - out of scope for the
     * fixed status strings this app sends. */
    int n = snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", component_name, text);
    if (n < 0 || (size_t)n >= sizeof(buf)) {
        return false;
    }
    return send_terminated(ctx, buf, (size_t)n);
}

bool nextion_set_text_fmt(nextion_uart_t *ctx, const char *component_name, const char *fmt, ...)
{
    char text[96];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    if (n < 0) {
        return false;
    }
    return nextion_set_text(ctx, component_name, text);
}

bool nextion_set_attr(nextion_uart_t *ctx, const char *component_name, const char *attr, int32_t value)
{
    char buf[96];
    int n = snprintf(buf, sizeof(buf), "%s.%s=%ld", component_name, attr, (long)value);
    if (n < 0 || (size_t)n >= sizeof(buf)) {
        return false;
    }
    return send_terminated(ctx, buf, (size_t)n);
}

bool nextion_goto_page(nextion_uart_t *ctx, uint8_t page_id)
{
    char buf[16];
    int n = snprintf(buf, sizeof(buf), "page %u", (unsigned)page_id);
    if (n < 0 || (size_t)n >= sizeof(buf)) {
        return false;
    }
    return send_terminated(ctx, buf, (size_t)n);
}
