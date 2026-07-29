#ifndef CDC_JOBS_H
#define CDC_JOBS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct { float x, y; } pick_job_t;

/* Called from the USB CDC RX callback (interrupt context) with newly
   arrived bytes. Buffers partial lines internally; on each complete line
   ("x,y" or "x y", CR/LF terminated) parses it and pushes a job onto the
   queue if the position is reachable. */
void CdcJobs_OnRxBytes(const uint8_t *data, uint32_t len);

/* Main-loop side only: pops the oldest pending job. Returns false if the
   queue is empty. */
bool CdcJobs_Pop(pick_job_t *out);

uint8_t CdcJobs_PendingCount(void);

void CdcJobs_InjectRaw(const char *str);

/* Test helper: injects one complete "x,y" line - appends '\n' for you,
   so you don't have to remember to add it in test code. */
void CdcJobs_InjectLine(const char *line_without_newline);

#endif
