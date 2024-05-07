/*
 * gptimer_capture.h
 *
 *  Created on: 03.12.2023
 *      Author: User
 */

#ifndef RECEIVER433_H_
#define RECEIVER433_H_

/* Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/*********************************************************************
 * TYPEDEFS
 */

// environment value
typedef struct {
    // raw message received from 433receiver
    // TODO: message can be removed - dev code only
    uint64_t     message;
    signed short temperature;
    uint8_t      humidity;
} environmentValue_t;

// Callback when an exposed environment value has changed
typedef void (*envValueChange_t)(environmentValue_t *newValue);
typedef void (*loggingMessage_t)(char* message, int len);
typedef void (*batteryMessage_t)(uint16_t battery);
typedef void (*errorMessage_t)(uint8_t error);

typedef struct {
    envValueChange_t pfnEnvValueChangeCB;  // Called when temperature value changes
    loggingMessage_t pfnLoggingMessageCB;  // Called when rcv433 wants to log a message
    batteryMessage_t pfnBatteryMessageCB;  // Called when a battery message is received
    errorMessage_t   pfnErrorMessageCB;    // Called when a error message is received
} envValueCBs_t;

// Return Codes
typedef enum {
    RCV433_OK                   = 0,
    RCV433_ERROR_PIN            = 1,
    RCV433_ERROR_CAPTURE_TIMER  = 2,
    RCV433_ERROR_OVERFLOW_TIMER = 3,
} receiver433_error_t;

/*
 * Main function. Starts receiving messages
 * TODO: hier könnte man als parameter die PIN-Nummer und auch die Fequenz mit geben
 */
extern receiver433_error_t Receiver433_start();

/*
  * Stop function. Releases hardware ressources (PIN, GP timers)
  */
extern void Receiver433_stop();

/*
 * Registration function for observers
 */
extern bool Receiver433_RegisterAppCBs(const envValueCBs_t* appCBs);

#endif /* RECEIVER433_H_ */
