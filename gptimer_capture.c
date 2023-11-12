
/*
 *  ======== empty.c ========
 *
 *  Dieses Projekt ist von Projektbeispiel "empty" abgeleitet.
 *  Es ist ein reines TIRTOS Projekt und benutzt den BLE stack nicht
 *
 *  Es soll die Nachrichten vom 433MHz Temperatur- und Feuchtigkeitssensor empfangen
 *  Das Design ist
 *  (1) Über eine Capture Timer die Signale vom Empfänger registrieren
 *  (2) In eine Queue schreiben
 *  (3) in einer low-prio task die signale auslesen und verarbeiten
 *
 *  Erfahrung:
 *  (1) Das Rauschen eines Superheteodyne Empfängers erzeugt zu viele zu schnelle Queue Einträge
 *  (2) Daher filtern wir in der ISR doch die ganz kurzen Impulse raus.
 *  (3) -> Design trägt so nicht. Gleich in der ISR komplett verarbeiten
 *  (4) -> aufteilung in gptimer_capture.c und receiver.h macht nicht so viel sinn,
 *         weil die ISR doch viel über den Status receiver.c wissen müsste um optimal
 *         zu filtern
 *
 *  gptimercapture_CC2640_LAUNCHXL_tirtos: hier ist wieder alles zusammen in einem Modul
 *
 *
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>

/* Driver Header files */
#include <ti/drivers/timer/GPTimerCC26XX.h>
//#include <ti/drivers/GPIO.h>
#include <ti/drivers/pin/PINCC26XX.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Debug Output */
#include <ti/display/Display.h>

/* Board Header file */
#include "Board.h"

/* BIOS Header file */
#include <ti/sysbios/BIOS.h>
/* Kernel Header files */
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Queue.h>
#include <xdc/runtime/Types.h>

/* App Header files */
#include "receiver.h"

// Struct for messages
typedef struct
{
  Queue_Elem       _elem;
  uint32_t         number;
  app_msg_edge_t   edge;
  uint32_t         width;
} app_msg_t;


/* PIN Configuration */
PIN_State  pinState;
PIN_Handle hPIN;

/* GPTimer Configuration*/
GPTimerCC26XX_Handle hCaptureTimer;
GPTimerCC26XX_Handle hOverflowTimer;

/* App data */
volatile uint32_t ledValue = CC2640R2_LAUNCHXL_PIN_LED_ON;
// Note: pressing the button will bring the button pin from HIGH to LOW
// so, initally we are waiting for a negative edge when playing with button 0
GPTimerCC26XX_Edge nextEdge = GPTimerCC26XX_NEG_EDGE;
volatile app_msg_edge_t e = APP_MSG_NEG_EDGE;
volatile bool errorStatus = FALSE;
volatile uint32_t pulse_no = 0;
volatile uint32_t overflow_count = 0;

uint32_t ticks_per_us;
uint32_t count_400us;


/* Message Queue */
// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;

/* Display */

/* App Fxn */

/*
 * CB function for the overflow timer. It counts the timeout/overflow events.
 */
void timerOverflowCB(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask) {
    // only timeout interrupts should occur
    ASSERT(interruptMask == GPT_INT_TIMEOUT);
    // count the event
    overflow_count++;
}

GPTimerCC26XX_Value lastTimerEventValue = 0;
uint32_t lastOverflowCount = 0;

void timerCaptureCB(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask) {

    // only capture interrupts should occur
    ASSERT(interruptMask == GPT_INT_CAPTURE);

    pulse_no++;

    // Read the timer value when the event occured (may be different from free running timer value)
    GPTimerCC26XX_Value timerEventValue = GPTimerCC26XX_getValue(handle);
    if (nextEdge == GPTimerCC26XX_POS_EDGE) {
        e = APP_MSG_POS_EDGE;
        nextEdge = GPTimerCC26XX_NEG_EDGE;
        ledValue = CC2640R2_LAUNCHXL_PIN_LED_ON;
    } else {
        e = APP_MSG_NEG_EDGE;
        nextEdge = GPTimerCC26XX_POS_EDGE;
        ledValue = CC2640R2_LAUNCHXL_PIN_LED_OFF;
    }
    PIN_setOutputValue(hPIN, Board_PIN_LED0, ledValue );

    GPTimerCC26XX_setCaptureEdge(handle, nextEdge );

    uint32_t ov_delta = overflow_count - lastOverflowCount;
    uint32_t pulse_width = 0;
    // Note: avoid multiplication for the frequent cases
    if(ov_delta == 0) {
        pulse_width = timerEventValue - lastTimerEventValue;
    } else if(ov_delta == 1) {
        pulse_width = (0x01000000 + timerEventValue) - lastTimerEventValue;
    } else {
        pulse_width = (0x01000000 * ov_delta + timerEventValue) - lastTimerEventValue;
    }

    if( pulse_width > count_400us ) {
        app_msg_t *pMsg = malloc( sizeof(app_msg_t));
        errorStatus = (pMsg == NULL);
        if (!errorStatus) {
            pMsg->number = pulse_no;
            pMsg->edge   = e;
            pMsg->width   = pulse_width;
            Queue_enqueue(hApplicationMsgQ ,&pMsg->_elem);
        } //endif errorStatus
    } else {
        // too short - ignore
    }
    lastTimerEventValue = timerEventValue;
    lastOverflowCount = overflow_count;
} //end timerCaptureCB()

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Init Display */
    Display_Handle hDisplay;
    hDisplay = Display_open(Display_Type_UART, NULL);

    /* Message Queue */
    // Initialize queue for application messages.
    // Note: Used to transfer control to application thread from e.g. interrupts.
    Queue_construct(&applicationMsgQ, NULL);
    hApplicationMsgQ = Queue_handle(&applicationMsgQ);

    /* Open access to PIN / GPIO */
    const PIN_Config gptPinInitTable[] = {
      Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//      Board_PIN_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES |PIN_HYSTERESIS,
      PIN_ID(23) | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES |PIN_HYSTERESIS,
      PIN_TERMINATE
    };

    // Get handle to this collection of pins
    hPIN = PIN_open(&pinState, gptPinInitTable);
    if (hPIN == NULL) {
        Display_printf(hDisplay, 0, 0, "Failed to open PINs");
        Task_exit();
    }

    /* Init and Configure Timer */
    GPTimerCC26XX_Params params;
    GPTimerCC26XX_Params_init(&params);
    params.width          = GPT_CONFIG_16BIT;
    params.mode           = GPT_MODE_EDGE_TIME;
    params.direction      = GPTimerCC26XX_DIRECTION_UP;
    params.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
    hCaptureTimer = GPTimerCC26XX_open(CC2640R2_LAUNCHXL_GPTIMER0A, &params);
    if(hCaptureTimer == NULL) {
        Display_printf(hDisplay, 0, 0, "Failed to open capture timer");
        Task_exit();
    }

    params.width = GPT_CONFIG_32BIT;
    params.mode  = GPT_MODE_PERIODIC;
    params.direction = GPTimerCC26XX_DIRECTION_UP;
    params.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
    hOverflowTimer = GPTimerCC26XX_open( CC2640R2_LAUNCHXL_GPTIMER1A, &params );
    if( hOverflowTimer == NULL ) {
        Display_printf(hDisplay, 0, 0, "Failed to open overflow timer");
        Task_exit();
    }

    Types_FreqHz  freq;
    BIOS_getCpuFreq(&freq);
#ifdef DEBUG
    Display_printf(hDisplay, 0, 0, "%d Freq Hz", freq.lo);
#endif

    ticks_per_us  = (freq.lo / 1000000);
    count_400us   = ticks_per_us * 400;
    count_3800_us = ticks_per_us * 3800;
    count_4200_us = ticks_per_us * 4200;
    count_18000_us = ticks_per_us * 18000;
    count_1800_us = ticks_per_us * 1800;
    count_2200_us = ticks_per_us * 2200;
    count_800_us  = ticks_per_us * 800;
    count_1200_us = ticks_per_us * 1200;

#ifdef DEBUG
    Display_printf(hDisplay, 0, 0, "%d t_p_us", ticks_per_us);
    Display_printf(hDisplay, 0, 0, "%d count_4000_us", ticks_per_us * 4000);
    Display_printf(hDisplay, 0, 0, "%d count_2000_us", ticks_per_us * 2000);
    Display_printf(hDisplay, 0, 0, "%d count_1000_us", ticks_per_us * 1000);
#endif

    const GPTimerCC26XX_Value loadVal = 0xFFFFFF;
    // Configure the capture timer GPT1A as 24 bit timer
    GPTimerCC26XX_setLoadValue(hCaptureTimer, loadVal);
    // Configure timer #2 as 24 bit timer
    // Note timer 2 is used to create overflow signals.
    // The capture timer cannot generate overflow interrupts.
    GPTimerCC26XX_setLoadValue(hOverflowTimer, loadVal);

    // Set the input pin for the capture timer
    GPTimerCC26XX_PinMux pinMux = GPTimerCC26XX_getPinMux(hCaptureTimer);
    Display_printf(hDisplay, 0, 0, "pinMux: %d", pinMux);
//    PINCC26XX_setMux(hPIN, PIN_ID(Board_PIN_BUTTON0), pinMux);
    PINCC26XX_setMux(hPIN, PIN_ID(23), pinMux);
    GPTimerCC26XX_setCaptureEdge(hCaptureTimer, nextEdge );
    GPTimerCC26XX_registerInterrupt(hCaptureTimer, timerCaptureCB, GPT_INT_CAPTURE );
    GPTimerCC26XX_registerInterrupt(hOverflowTimer, timerOverflowCB, GPT_INT_TIMEOUT );
    // start timer 2
    GPTimerCC26XX_start(hOverflowTimer);
    GPTimerCC26XX_start(hCaptureTimer);

    /* Turn on red LED */
    if( PIN_setOutputValue(hPIN, Board_PIN_LED0, CC2640R2_LAUNCHXL_PIN_LED_ON) != PIN_SUCCESS ) {
#ifdef DEBUG
        Display_printf(hDisplay,0,0, "Failed to turn LED on");
#endif
    }
    uint32_t lastNum = 0;

    while (1) {
        while(!Queue_empty(hApplicationMsgQ)) {
            app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);
            if( pMsg != NULL ) {
                bool missed_edges = pMsg->number != (lastNum + 1);
                int nibble_count = processEdge(pMsg->width, pMsg->edge, missed_edges);
                if((nibble_count == 9) && (pMsg->edge == GPTimerCC26XX_POS_EDGE && !errorStatus)) {
                    uint64_t message = getMessage();
                    Display_printf(hDisplay, 0, 0, "%08lx%08lx", (uint32_t)(message>>32), (uint32_t)message);
                }
//                Display_printf(hDisplay, 0, 0, "no: %08d, w: %08d n:%02d", pMsg->number, pMsg->width, nibble_count);
                free(pMsg);
            }
            lastNum = pMsg->number;
            if( errorStatus ) {
                Display_printf(hDisplay, 0, 0, "Error");
            }
        }
    } // end while()

    // The following cleanup code is unreachable; put it here fore safety
#pragma diag_suppress=112
    Display_printf(hDisplay, 0, 0, "Task Terminated");
    Display_close(hDisplay);
    PIN_close(hPIN);
    GPTimerCC26XX_close(hCaptureTimer);
    GPTimerCC26XX_close(hOverflowTimer);
} // end mainThread()
