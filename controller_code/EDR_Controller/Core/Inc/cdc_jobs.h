/* cdc_jobs.h */
#ifndef CDC_JOBS_H
#define CDC_JOBS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct { float x, y; } pick_job_t;
typedef struct { uint32_t arrival_local_ms; } conveyor_job_t;

/* Feeds bytes as if received over a UART/USB line. `source_id` lets
   multiple independent byte streams (USB CDC, Nextion UART, ...) each
   keep their own line-accumulation buffer without corrupting each
   other's partial lines. */
void CdcJobs_OnRxBytesSrc(const uint8_t *data, uint32_t len, uint8_t source_id);
void CdcJobs_OnRxBytes(const uint8_t *data, uint32_t len); /* = source 0 (USB CDC) */

/* Debug "x,y" job queue - kept exactly as before, for manual testing. */
bool CdcJobs_Pop(pick_job_t *out);
uint8_t CdcJobs_PendingCount(void);

/* Conveyor detection-event queue: each entry is a predicted LOCAL tick
   (HAL_GetTick() timeline) at which the object will be at the pick
   point, already including travel time and PICK_TIME_OFFSET_MS. */
bool CdcJobs_PopConveyorJob(conveyor_job_t *out);
uint8_t CdcJobs_ConveyorPendingCount(void);

/* Test/debug injection helpers (unchanged) */
void CdcJobs_InjectRaw(const char *str);
void CdcJobs_InjectLine(const char *line_without_newline);

#endif
