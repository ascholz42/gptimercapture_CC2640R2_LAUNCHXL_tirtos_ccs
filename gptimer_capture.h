/*
 * gptimer_capture.h
 *
 *  Created on: 03.12.2023
 *      Author: User
 */

#ifndef GPTIMER_CAPTURE_H_
#define GPTIMER_CAPTURE_H_

/*
 * Task creation function
 * Exported to make it useable for BLE projects
 */
extern void GPTimerCapture_createTask(void);

/*
 * Main Thread function exported for POSIX-like Thread creation (main_tirtos.c)
 * Left here fore reference to the example project
 */
// extern void *GPTimerCapture_mainThread(void *arg0);

#endif /* GPTIMER_CAPTURE_H_ */
