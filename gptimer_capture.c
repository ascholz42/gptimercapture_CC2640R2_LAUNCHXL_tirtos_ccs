
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
 * Note: Empfang ist sehr abhängig von der Polarisierung, wenn Antenne richtig gedreht ist -> besser
 * Projektbeispiel empty nutzt pthread.h. zur Threadgenerierung -> umgestellt auf TIRTOS Task creation
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
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <xdc/runtime/Types.h>

/* Application header */
#include "gptimer_capture.h"

// Struct for messages
typedef struct
{
  Queue_Elem    _elem;
  uint64_t      message;
} app_msg_t;


/* PIN Configuration */
PIN_State  pinState;
PIN_Handle hPIN;

/* GPTimer Configuration*/
GPTimerCC26XX_Handle hCaptureTimer;
GPTimerCC26XX_Handle hOverflowTimer;

/* Periodic Clock */
static Clock_Struct periodicClock;
static Clock_Params periodicClockParams;
static Clock_Handle hPeriodicClock;

/* 57s Clock */
static Clock_Struct c57Clock;
static Clock_Params c57ClockParams;
static Clock_Handle hC57Clock;

/* Events */
static Event_Struct eventStruct;
static Event_Params eventParams;
static Event_Handle hEvent;

#define MESSAGE_EVT Event_Id_00

// Task configuration
#define GPTCapture_TASK_STACK_SIZE 1024
Task_Struct gptCaptureTask;
char gptTaskStack[GPTCapture_TASK_STACK_SIZE];
// Declare the app task function
static void GPTimerCapture_taskFxn(UArg a0, UArg a1);


/* App data */
volatile uint32_t ledValue = CC2640R2_LAUNCHXL_PIN_LED_ON;
GPTimerCC26XX_Edge nextEdge = GPTimerCC26XX_NEG_EDGE;
volatile uint32_t overflow_count = 0;

/* Receiver data */
bool    synced = false;
uint8_t nibble_count;
uint8_t bit_mask = 0;
uint8_t input = 0;
uint64_t result = 0;

/* Message Queue */
// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;

// pre-computed tick count values for all timing parameters
uint32_t count_1_us;      //number of timer ticks per 1 us
uint32_t count_380_us;
uint32_t count_650_us;
uint32_t count_3800_us;
uint32_t count_4300_us;
uint32_t count_18000_us;
uint32_t count_1800_us;
uint32_t count_2200_us;
uint32_t count_800_us;
uint32_t count_1200_us;


/* Display */

/* App Fxn */
void startTimers();
void stopTimers();

static void clockHandler(UArg arg){
    startTimers();
    Clock_stop(hPeriodicClock);
}

static void c57Handler(UArg arg){
    app_msg_t *pMsg = malloc( sizeof(app_msg_t));
    if (pMsg) {
        pMsg->message = 0xFF;
        Queue_enqueue(hApplicationMsgQ ,&pMsg->_elem);
        Event_post(hEvent, MESSAGE_EVT);
    }
}

// Note these functions will be called in interrput handling context. Dontuse Display...

void shiftToNextBit() {
    bit_mask = bit_mask >> 1;
    if (bit_mask == 0b00000000) {
      // A nibble was received
      result = result << 4;
      result = result | (input & 0x0F);
      // wrap around to receive the next nibble
      bit_mask = 0b00001000;
      input = 0;
      // count the nibble
      nibble_count++;
    }// endif nibble received
} // end shiftToNextBit()

void outOfSync() {
    synced = false;
    nibble_count = 0;
} //end outOfSync()

void inSync() {
    synced = true;
    nibble_count = 0;
    input = 0;
    bit_mask = 0b00001000;
    result = 0;
} //end inSync()

void enqueueResult() {
    app_msg_t *pMsg = malloc( sizeof(app_msg_t));
    if (pMsg) {
        pMsg->message = result;
        Queue_enqueue(hApplicationMsgQ ,&pMsg->_elem);
        Event_post(hEvent, MESSAGE_EVT);
    } //endif
} // end enqueueResult()

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

    static GPTimerCC26XX_Edge edgeType;
    // only capture interrupts should occur
    ASSERT(interruptMask == GPT_INT_CAPTURE);

    // Read the timer value when the event occured (may be different from free running timer value)
    GPTimerCC26XX_Value timerEventValue = GPTimerCC26XX_getValue(handle);
    edgeType = nextEdge;
    if (nextEdge == GPTimerCC26XX_POS_EDGE) {
        nextEdge = GPTimerCC26XX_NEG_EDGE;
        ledValue = CC2640R2_LAUNCHXL_PIN_LED_ON;
    } else {
        ASSERT(nextEdge == GPTimerCC26XX_NEG_EDGE);
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

    if( !synced                 &&
        (pulse_width > count_3800_us) &&
        (pulse_width < count_4300_us) &&
        (edgeType == GPTimerCC26XX_POS_EDGE) )
    { // This is a sync signal; Initialise receiving a message
        inSync();
    } else if (synced) {
        if (edgeType == GPTimerCC26XX_NEG_EDGE) {
            if((pulse_width < count_380_us) || (pulse_width > count_650_us)) {
                // dont accept -> out of sync
                outOfSync();
            } else {
                // a ca. 500 us HIGH  -> do nothing
            } //end check width
        } // end negative edge
        else {
            ASSERT(edgeType == GPTimerCC26XX_POS_EDGE);
            if(pulse_width < count_800_us) {
                // out of sync
                outOfSync();
            } // end < 800 us
            else if(pulse_width < count_1200_us) {
                // logical zero
                input &= ~bit_mask;
                shiftToNextBit();
            } else if(pulse_width < count_1800_us) {
                // out of sync
                outOfSync();
            } else if( pulse_width < count_2200_us) {
                // logical one
                input |= bit_mask;
                shiftToNextBit();
            } else if( pulse_width < count_3800_us) {
                // out of sync
                outOfSync();
            } else {
                // This is a re-sync and a repeat will follow
                if(nibble_count == 9) {
                    // a full message was received
                    enqueueResult();
                    stopTimers();
                    Clock_start(hPeriodicClock);
                }
                // otherweise throw away any result or and start from the beginning

                if( pulse_width < count_4300_us) {
                    // re-sync
                    inSync();
                } else {
                    // terminating long LOW
                    outOfSync();
                }
            } // endif width
        } //endif pos edge
    } // endif synced

    // remember the current time
    lastTimerEventValue = timerEventValue;
    lastOverflowCount = overflow_count;
} //end timerCaptureCB()

void startTimers() {
    nextEdge = GPTimerCC26XX_NEG_EDGE;
    GPTimerCC26XX_setCaptureEdge(hCaptureTimer, nextEdge );

    // TODO: set the timer values to zero; set the prescaler values to zero
    // However, not supported by the driver....
    // maybe thats not so important
    // Es könnte auch sein, dass GPT..._start das TnEN-bit schreibt und dabei
    // automatisch der Timer zurück gesetzt wird -> Datenblatt.....

    // enable interrupts
    GPTimerCC26XX_enableInterrupt(hCaptureTimer, GPT_INT_CAPTURE);
    GPTimerCC26XX_enableInterrupt(hOverflowTimer, GPT_INT_TIMEOUT);
    // start timers
    GPTimerCC26XX_start(hOverflowTimer);
    GPTimerCC26XX_start(hCaptureTimer);
    /* Turn on red LED */
    ledValue = CC2640R2_LAUNCHXL_PIN_LED_ON;
    if( PIN_setOutputValue(hPIN, Board_PIN_LED0, ledValue) != PIN_SUCCESS ) {
#ifdef DEBUG
//        Display_printf(hDisplay,0,0, "Failed to turn LED on");
#endif
    } //endif
} //end startTimers()

void stopTimers() {
    // disable interrupts
    GPTimerCC26XX_disableInterrupt(hCaptureTimer, GPT_INT_CAPTURE);
    GPTimerCC26XX_disableInterrupt(hOverflowTimer, GPT_INT_TIMEOUT);
    // stop timers
    GPTimerCC26XX_stop(hOverflowTimer);
    GPTimerCC26XX_stop(hCaptureTimer);
    ledValue = CC2640R2_LAUNCHXL_PIN_LED_OFF;
    PIN_setOutputValue(hPIN, Board_PIN_LED0, ledValue);
} //end stopTimers()

/*
 * @brief   Task creation function for the user task.
 *
 * @param   None.
 *
 * @return  None.
 */

void GPTimerCapture_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = gptTaskStack;
  taskParams.stackSize = GPTCapture_TASK_STACK_SIZE;
  taskParams.priority = 1;

  Task_construct(&gptCaptureTask, GPTimerCapture_taskFxn, &taskParams, NULL);
}

/*
 *  ======== mainThread ========
 *  POSIX thread function signature
void *GPTimerCapture_mainThread(void *arg0)
 */
static void GPTimerCapture_taskFxn(UArg a0, UArg a1) {

    /* Init Display */
    Display_Handle hDisplay;
    hDisplay = Display_open(Display_Type_UART, NULL);
    Display_printf(hDisplay, 0, 0, "Starting main thread");

    /* Events */
    Event_Params_init(&eventParams); // use the default params
    Event_construct(&eventStruct, &eventParams);
    hEvent = Event_handle(&eventStruct);

    /* Message Queue */
    // Initialize queue for application messages.
    Queue_construct(&applicationMsgQ, NULL);
    hApplicationMsgQ = Queue_handle(&applicationMsgQ);

    /* Periodic Clock */
    Clock_Params_init(&periodicClockParams);
    periodicClockParams.period = 0; // One-Shot timer
    periodicClockParams.startFlag = FALSE; // Clock_start required to start the clock
    Clock_construct(&periodicClock, clockHandler, 55000*(1000/Clock_tickPeriod), &periodicClockParams);
    hPeriodicClock = Clock_handle(&periodicClock);

    /* 57 sec Clock */
    Clock_Params_init(&c57ClockParams);
    c57ClockParams.period = 57000*(1000/Clock_tickPeriod); // periodic clock
    c57ClockParams.startFlag = TRUE; // clock will start right away
    Clock_construct(&c57Clock, c57Handler, 57000*(1000/Clock_tickPeriod), &c57ClockParams); // timout = 0; start right without delay
    hC57Clock = Clock_handle(&c57Clock);

    /* Open access to PIN / GPIO */
    const PIN_Config gptPinInitTable[] = {
      Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      PIN_ID(23) | PIN_INPUT_EN | PIN_NOPULL,
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

    count_1_us  = (freq.lo / 1000000);
    count_380_us   = count_1_us * 380;
    count_650_us   = count_1_us * 650;
    count_3800_us  = count_1_us * 3800;
    count_4300_us  = count_1_us * 4300;
    count_18000_us = count_1_us * 18000;
    count_1800_us  = count_1_us * 1800;
    count_2200_us  = count_1_us * 2200;
    count_800_us   = count_1_us * 800;
    count_1200_us  = count_1_us * 1200;

#ifdef DEBUG
    Display_printf(hDisplay, 0, 0, "%d t_p_us", count_1_us);
    Display_printf(hDisplay, 0, 0, "%d count_4000_us", count_1_us * 4000);
    Display_printf(hDisplay, 0, 0, "%d count_2000_us", count_1_us * 2000);
    Display_printf(hDisplay, 0, 0, "%d count_1000_us", count_1_us * 1000);
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
#ifdef DEBUG
    Display_printf(hDisplay, 0, 0, "pinMux: %d", pinMux);
#endif
    PINCC26XX_setMux(hPIN, PIN_ID(23), pinMux);
    GPTimerCC26XX_registerInterrupt(hCaptureTimer, timerCaptureCB, GPT_INT_CAPTURE );
    GPTimerCC26XX_registerInterrupt(hOverflowTimer, timerOverflowCB, GPT_INT_TIMEOUT );
    startTimers();

    uint32_t events;
    while (1) {
        /* Wait for Message event */
        events = Event_pend(hEvent, MESSAGE_EVT, 0, BIOS_WAIT_FOREVER);
        ASSERT(events == MESSAGE_EVT);

        while(!Queue_empty(hApplicationMsgQ)) {
            app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);
            if( pMsg != NULL ) {
                if (pMsg->message == 0xFF)
                {
                    Display_printf(hDisplay, 0, 0, "----------");
                } else {
                    Display_printf(hDisplay, 0, 0, "%08lx%08lx", (uint32_t)(pMsg->message >> 32), (uint32_t)pMsg->message);
                }
                free(pMsg);
            } //endif
        } //end while Queue not empty
    } // end while(1)

    // The following cleanup code is unreachable; put it here fore safety
#pragma diag_suppress=112
    Display_printf(hDisplay, 0, 0, "Task Terminated");
    Display_close(hDisplay);
    PIN_close(hPIN);
    GPTimerCC26XX_close(hCaptureTimer);
    GPTimerCC26XX_close(hOverflowTimer);
} // end mainThread()
