/* cdc_jobs.c */
#include "cdc_jobs.h"
#include "kinematics.h"
#include "app_config.h"
#include "app_log.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "stm32h7xx_hal.h"

/* ---------------- debug x,y job queue (unchanged behavior) ---------------- */
#define JOB_QUEUE_SIZE 8
static volatile pick_job_t s_jobs[JOB_QUEUE_SIZE];
static volatile uint8_t s_head = 0, s_tail = 0, s_count = 0;

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

uint8_t CdcJobs_PendingCount(void) { return s_count; }

/* ---------------- conveyor detection-event queue ---------------- */
#define CONVEYOR_QUEUE_SIZE 8
static volatile conveyor_job_t s_conv_jobs[CONVEYOR_QUEUE_SIZE];
static volatile uint8_t s_conv_head = 0, s_conv_tail = 0, s_conv_count = 0;

static bool conveyor_queue_push(uint32_t arrival_ms)
{
    __disable_irq();
    if (s_conv_count >= CONVEYOR_QUEUE_SIZE) { __enable_irq(); return false; }
    s_conv_jobs[s_conv_tail].arrival_local_ms = arrival_ms;
    s_conv_tail = (uint8_t)((s_conv_tail + 1) % CONVEYOR_QUEUE_SIZE);
    s_conv_count++;
    __enable_irq();
    return true;
}

bool CdcJobs_PopConveyorJob(conveyor_job_t *out)
{
    __disable_irq();
    if (s_conv_count == 0) { __enable_irq(); return false; }
    out->arrival_local_ms = s_conv_jobs[s_conv_head].arrival_local_ms;
    s_conv_head = (uint8_t)((s_conv_head + 1) % CONVEYOR_QUEUE_SIZE);
    s_conv_count--;
    __enable_irq();
    return true;
}

uint8_t CdcJobs_ConveyorPendingCount(void) { return s_conv_count; }

/* ---------------- clock sync state ---------------- */
static volatile bool s_sync_received = false;
static volatile int32_t s_clock_offset_ms = 0; /* local_tick - vision_ms, captured at SYNC */

/* ---------------- per-source line accumulation ---------------- */
#define NUM_SOURCES 2
#define LINE_BUF_SIZE 64
static char s_line_buf[NUM_SOURCES][LINE_BUF_SIZE];
static uint8_t s_line_len[NUM_SOURCES] = {0};

static bool parse_xy(const char *line, float *x_out, float *y_out)
{
    char *endptr;
    float x = strtof(line, &endptr);
    if (endptr == line) return false;
    while (*endptr == ' ' || *endptr == '\t' || *endptr == ',') endptr++;
    float y = strtof(endptr, &endptr);
    if (endptr == line) return false;
    *x_out = x;
    *y_out = y;
    return true;
}

static void handle_sync(const char *value_str)
{
    long vision_ms = strtol(value_str, NULL, 10);
    uint32_t local_now = HAL_GetTick();
    s_clock_offset_ms = (int32_t)local_now - (int32_t)vision_ms;
    s_sync_received = true;
    cdc_log("CdcJobs: SYNC received (vision=%ld ms, local=%lu ms, offset=%ld ms)\r\n",
            vision_ms, (unsigned long)local_now, (long)s_clock_offset_ms);
}

static void handle_timestamp(const char *value_str)
{
    if (!s_sync_received) {
        cdc_log("CdcJobs: TIMESTAMP_MS received before SYNC_MS - ignored (send SYNC_MS first).\r\n");
        return;
    }
    long vision_ms = strtol(value_str, NULL, 10);
    float travel_time_ms = (CAMERA_TO_PICK_DISTANCE_MM / CONVEYOR_VELOCITY_MM_S) * 1000.0f;
    int32_t predicted_local_arrival = (int32_t)vision_ms + s_clock_offset_ms
                                     + (int32_t)travel_time_ms + (int32_t)PICK_TIME_OFFSET_MS;

    int32_t lead_time_ms = predicted_local_arrival - (int32_t)HAL_GetTick();
    if (lead_time_ms < (int32_t)MIN_LEAD_TIME_MS) {
        cdc_log("CdcJobs: WARNING - detection gives only %ld ms lead time (min recommended %d ms) - "
                "pick may be late.\r\n", (long)lead_time_ms, (int)MIN_LEAD_TIME_MS);
    }

    if (conveyor_queue_push((uint32_t)predicted_local_arrival)) {
        cdc_log("CdcJobs: queued conveyor pick, grab scheduled in %ld ms [queue depth %d]\r\n",
                (long)lead_time_ms, (int)s_conv_count);
    } else {
        cdc_log("CdcJobs: conveyor queue full (%d) - detection dropped.\r\n", (int)CONVEYOR_QUEUE_SIZE);
    }
}

static void handle_complete_line(char *line)
{
    int len = (int)strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ')) {
        line[--len] = '\0';
    }
    if (len == 0) return;

    if (strncmp(line, "SYNC_MS=", 8) == 0) {
        handle_sync(line + 8);
        return;
    }
    if (strncmp(line, "TIMESTAMP_MS=", 13) == 0) {
        handle_timestamp(line + 13);
        return;
    }

    /* Fallback: existing debug "x,y" behavior, unchanged. */
    float x, y;
    if (!parse_xy(line, &x, &y)) {
        cdc_log("CdcJobs: could not parse \"%s\" - ignored.\r\n", line);
        return;
    }
    float t1, t2;
    if (!inverse_kinematics(x, y, &t1, &t2)) {
        cdc_log("CdcJobs: [debug] (%.2f, %.2f) unreachable - ignored.\r\n", (double)x, (double)y);
        return;
    }
    if (job_queue_push(x, y)) {
        cdc_log("CdcJobs: [debug] queued pick (%.2f, %.2f) [queue depth %d]\r\n",
                (double)x, (double)y, (int)s_count);
    } else {
        cdc_log("CdcJobs: [debug] queue full (%d) - (%.2f, %.2f) dropped.\r\n",
                (int)JOB_QUEUE_SIZE, (double)x, (double)y);
    }
}

void CdcJobs_OnRxBytesSrc(const uint8_t *data, uint32_t len, uint8_t src)
{
    if (src >= NUM_SOURCES) return;
    for (uint32_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n') {
            s_line_buf[src][s_line_len[src]] = '\0';
            handle_complete_line(s_line_buf[src]);
            s_line_len[src] = 0;
        } else if (s_line_len[src] < (LINE_BUF_SIZE - 1)) {
            s_line_buf[src][s_line_len[src]++] = c;
        } else {
            cdc_log("CdcJobs: line too long on source %d, discarding.\r\n", (int)src);
            s_line_len[src] = 0;
        }
    }
}

void CdcJobs_OnRxBytes(const uint8_t *data, uint32_t len)
{
    CdcJobs_OnRxBytesSrc(data, len, 0); /* USB CDC = source 0 */
}

void CdcJobs_InjectRaw(const char *str)
{
    CdcJobs_OnRxBytesSrc((const uint8_t *)str, (uint32_t)strlen(str), 0);
}

void CdcJobs_InjectLine(const char *line_without_newline)
{
    char buf[LINE_BUF_SIZE + 2];
    int n = snprintf(buf, sizeof(buf), "%s\n", line_without_newline);
    if (n > 0) {
        CdcJobs_OnRxBytesSrc((const uint8_t *)buf, (uint32_t)n, 0);
    }
}
