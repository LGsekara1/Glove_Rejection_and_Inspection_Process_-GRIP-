#ifndef NEXTION_DISPLAY_H
#define NEXTION_DISPLAY_H

#include "main.h"
#include <stdbool.h>

/* Call once, after the Nextion UART (UART7) has been initialized. */
void NextionDisplay_Init(UART_HandleTypeDef *huart);

/* Call periodically (e.g. every 500 ms) from the main loop.
 * Reads axis0/axis1 state over the ODrive UART link and pushes
 * a human-readable string to text fields t0 / t1 on the Nextion. */
void NextionDisplay_PollOdriveState(void);

void NextionDisplay_UART_RxByteISR(void);

#endif /* NEXTION_DISPLAY_H */
