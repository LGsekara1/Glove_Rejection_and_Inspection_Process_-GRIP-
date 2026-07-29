/* cdc_jobs.c */
#include "cdc_jobs.h"
#include "kinematics.h"
#include "app_log.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/* Single-producer (USB RX interrupt) / single-consumer (main loop) ring
   buffer. Safe without locks as long as the producer only ever advances
   s_tail (after fully writing the slot) and the consumer only ever
   advances s_head - the classic SPSC pattern. */
#define JOB_QUEUE_SIZE 8
static volatile pick_job_t s_jobs[JOB_QUEUE_SIZE];
static volatile uint8_t s_head = 0;
static volatile uint8_t s_tail = 0;
static volatile uint8_t s_count = 0;

/* Line accumulation buffer - only ever touched from the RX interrupt, so
   no locking needed for this part either. */
#define LINE_BUF_SIZE 64
static char s_line_buf[LINE_BUF_SIZE];
static uint8_t s_line_len = 0;

#include "stm32h7xx_hal.h"  /* for __disable_irq/__enable_irq */

static bool job_queue_push(float x, float y)
{
    __disable_irq();
    if (s_count >= JOB_QUEUE_SIZE) { __enable_irq(); return false; }
    s_jobs[s_tail].x = x;
    s_jobs[s_tail].y = y;
    s_tail = (uint8_t)((s_tail + 1) % JOB_QUEUE_SIZE);
    s_count++;
    __enable_irq();
    return true;
}

bool CdcJobs_Pop(pick_job_t *out)
{
    __disable_irq();
    if (s_count == 0) { __enable_irq(); return false; }
    out->x = s_jobs[s_head].x;
    out->y = s_jobs[s_head].y;
    s_head = (uint8_t)((s_head + 1) % JOB_QUEUE_SIZE);
    s_count--;
    __enable_irq();
    return true;
}

uint8_t CdcJobs_PendingCount(void)
{
    return s_count;
}

/* Parses "x,y" or "x y" (leading/trailing whitespace tolerated). Returns
   false if the line doesn't contain two valid numbers. */
static bool parse_line(const char *line, float *x_out, float *y_out)
{
    char *endptr;
    float x = strtof(line, &endptr);
    if (endptr == line) return false;

    while (*endptr == ' ' || *endptr == '\t' || *endptr == ',') endptr++;

    float y = strtof(endptr, &endptr);
    if (endptr == line) return false; /* nothing consumed for y either */

    *x_out = x;
    *y_out = y;
    return true;
}

static void handle_complete_line(char *line)
{
    /* strip trailing \r and whitespace */
    int len = (int)strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ')) {
        line[--len] = '\0';
    }
    if (len == 0) return; /* blank line, ignore */

    float x, y;
    if (!parse_line(line, &x, &y)) {
        cdc_log("CdcJobs: could not parse \"%s\" as \"x,y\" - ignored.\r\n", line);
        return;
    }

    float t1, t2;
    if (!inverse_kinematics(x, y, &t1, &t2)) {
        cdc_log("CdcJobs: (%.2f, %.2f) unreachable - ignored.\r\n", (double)x, (double)y);
        return;
    }

    if (job_queue_push(x, y)) {
        cdc_log("CdcJobs: queued pick (%.2f, %.2f) [queue depth %d]\r\n",
                (double)x, (double)y, (int)s_count);
    } else {
        cdc_log("CdcJobs: queue full (%d) - (%.2f, %.2f) dropped.\r\n",
                (int)JOB_QUEUE_SIZE, (double)x, (double)y);
    }
}

void CdcJobs_OnRxBytes(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n') {
            s_line_buf[s_line_len] = '\0';
            handle_complete_line(s_line_buf);
            s_line_len = 0;
        } else {
            if (s_line_len < (LINE_BUF_SIZE - 1)) {
                s_line_buf[s_line_len++] = c;
            } else {
                cdc_log("CdcJobs: line too long, discarding.\r\n");
                s_line_len = 0;
            }
        }
    }
}

void CdcJobs_InjectRaw(const char *str)
{
    CdcJobs_OnRxBytes((const uint8_t *)str, (uint32_t)strlen(str));
}

void CdcJobs_InjectLine(const char *line_without_newline)
{
    char buf[LINE_BUF_SIZE + 2];
    int n = snprintf(buf, sizeof(buf), "%s\n", line_without_newline);
    if (n > 0) {
        CdcJobs_OnRxBytes((const uint8_t *)buf, (uint32_t)n);
    }
}


