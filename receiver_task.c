/*
 * receiver_thread.c
 *
 *  Created on: 10.12.2023
 *      Author: User
 */



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

#include <receiver433.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>

/* Driver Header files */

/* Debug Output */
#include <ti/display/Display.h>

/* Board Header file */
//#include "Board.h"

/* BIOS Header file */
#include <ti/sysbios/BIOS.h>
/* Kernel Header files */
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <xdc/runtime/Types.h>

/* Application header */

/* Client app */

typedef enum {
    VALUE_MSG   = 0,
    LOGGING_MSG = 1,
} messageType_t;

// Struct for messages
typedef struct
{
  Queue_Elem         _elem;
  messageType_t      msgType;
  environmentValue_t *envValue;
  char               *loggingString;
} app_msg_t;

/* 57s Clock */
static Clock_Struct c57Clock;
static Clock_Params c57ClockParams;
static Clock_Handle hC57Clock;

/* Events */
static Event_Struct eventStruct;
static Event_Params eventParams;
static Event_Handle hEvent;

#define MESSAGE_EVT Event_Id_04
#define CLOCK57_EVT Event_Id_05

// Task configuration
#define RCV_TASK_STACK_SIZE 1024
Task_Struct rcvTask;
char gptTaskStack[RCV_TASK_STACK_SIZE];
// Declare the app task function
static void Receiver_taskFxn(UArg a0, UArg a1);

/* Message Queue */
// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;

/* Display */

/* App callback functions */
void envValueChangeCB(environmentValue_t *newValue); // will be called when an envirnoment value changes
void loggingMessageCB(char *message);                // will be called when something neeeds to be logged

const envValueCBs_t appCBs = {
  .pfnEnvValueChangeCB = envValueChangeCB,
  .pfnLoggingMessageCB = loggingMessageCB,
};

/*
 *
 */

void c57Handler(UArg arg){
    Event_post(hEvent, CLOCK57_EVT);
}

void envValueChangeCB(environmentValue_t *newValue) {
    app_msg_t *pMsg = malloc( sizeof(app_msg_t));
    if (pMsg) {
        pMsg->msgType = VALUE_MSG;
        pMsg->envValue = newValue;
        Queue_enqueue(hApplicationMsgQ ,&pMsg->_elem);
        Event_post(hEvent, MESSAGE_EVT);
    } //endif
} // end enqueueResult()

void loggingMessageCB(char *message) {
    app_msg_t *pMsg = malloc( sizeof(app_msg_t));
    if (pMsg) {
        pMsg->msgType = LOGGING_MSG;
        pMsg->envValue = NULL;
        pMsg->loggingString = message;
        Queue_enqueue(hApplicationMsgQ ,&pMsg->_elem);
        Event_post(hEvent, MESSAGE_EVT);
    } //endif
} // end enqueueResult()

/*
 * @brief   Task creation function for the user task.
 *
 * @param   None.
 *
 * @return  None.
 */

void Receiver_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = gptTaskStack;
  taskParams.stackSize = RCV_TASK_STACK_SIZE;
  taskParams.priority = 1;

  Task_construct(&rcvTask, Receiver_taskFxn, &taskParams, NULL);
}

/*
 *  ======== mainThread ========
 *  POSIX thread function signature
void *GPTimerCapture_mainThread(void *arg0)
 */
static void Receiver_taskFxn(UArg a0, UArg a1) {

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

    /* 57 sec Clock */
    Clock_Params_init(&c57ClockParams);
    c57ClockParams.period = 57000*(1000/Clock_tickPeriod); // periodic clock
    c57ClockParams.startFlag = TRUE; // clock will start right away
    Clock_construct(&c57Clock, c57Handler, 57000*(1000/Clock_tickPeriod), &c57ClockParams); // timout = 0; start right without delay
    hC57Clock = Clock_handle(&c57Clock);

    // register callback fxns with receiver433
    Receiver433_RegisterAppCBs(&appCBs);

    receiver433_error_t result = Receiver433_start();
    if (result != RCV433_OK) {
        Display_printf(hDisplay, 0, 0, "Failed to start Receiver433 %d", result);
        Task_exit();
    }

    while (1) {
        /* Wait for Message event */
        uint32_t events = Event_pend(hEvent, Event_Id_NONE, MESSAGE_EVT | CLOCK57_EVT, BIOS_WAIT_FOREVER);
        if (events & CLOCK57_EVT) {
            Display_printf(hDisplay, 0, 0, "----------");
        } else {
            while(!Queue_empty(hApplicationMsgQ)) {
                app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);
                if( (pMsg != NULL) && (pMsg->msgType == VALUE_MSG)) {
                    Display_printf(hDisplay, 0, 0, "%08lx%08lx", (uint32_t)(pMsg->envValue->message >> 32), (uint32_t)pMsg->envValue->message);
                    Display_printf(hDisplay, 0, 0, "%d %d", pMsg->envValue->humidity, pMsg->envValue->temperature);
                } else if ((pMsg != NULL) && (pMsg->msgType == LOGGING_MSG)) {
                    Display_printf(hDisplay, 0, 0, "%s", pMsg->loggingString);
                }
                free(pMsg);
            } //end while Queue not empty
        } //endif event type
    } // end while(1)

    // The following cleanup code is unreachable; put it here fore safety
#pragma diag_suppress=112
    Display_printf(hDisplay, 0, 0, "Task Terminated");
    Receiver433_stop();
    Display_close(hDisplay);
} // end Receiver_taskFxn()
