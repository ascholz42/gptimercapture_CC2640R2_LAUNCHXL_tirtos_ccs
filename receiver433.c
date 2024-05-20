
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
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

/* Driver Header files */
#include <ti/drivers/timer/GPTimerCC26XX.h>
//#include <ti/drivers/GPIO.h>
#include <ti/drivers/pin/PINCC26XX.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header file */
#include "Board.h"

/* BIOS Header file */
#include <ti/sysbios/BIOS.h>
/* Kernel Header files */
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/Types.h>

/* Application header */

/* Client app */
// callbacks
static const envValueCBs_t *pAppCBs = NULL;
// Environment Value
static environmentValue_t currentEnvironmentValue;
// Battery Voltage
static uint16_t currentBatteryVoltage;

/* PIN Configuration */
PIN_State  pinState;
PIN_Handle hPIN;

/* GPTimer Configuration*/
GPTimerCC26XX_Handle hCaptureTimer;
GPTimerCC26XX_Handle hOverflowTimer;

/* Periodic Clock */
Clock_Struct periodicClock;
Clock_Params periodicClockParams;
Clock_Handle hPeriodicClock;

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


/* App Fxn */
void startTimers();
void stopTimers();
bool resultToCurrentValue(uint8_t p_numNibbles);

static void clockHandler(UArg arg){
    // start receiving
    startTimers();
    // stop this clock; will be restarted when a message was received
    Clock_stop(hPeriodicClock);
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
                bool stopReceiving = false;
                // This is a re-sync and a repeat will follow
                if((nibble_count == 9) || (nibble_count == 6) || (nibble_count == 4)) {
                    // a full message was received
                    stopReceiving = resultToCurrentValue(nibble_count);
                }
                // otherweise throw away any result or and start from the beginning

                if( pulse_width < count_4300_us) {
                    // re-sync
                    inSync();
                } else {
                    // terminating long LOW
                    outOfSync();
                    if( stopReceiving ) {
                        // stop receiving
                        stopTimers();
                        // start clock. After a period of time receiving will be started again.
                        Clock_start(hPeriodicClock);
                    }
                } // endif pulse_width < 4300 us
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


bool resultToCurrentValue(uint8_t p_numNibbles) {
  bool returnValue = false;
  if( p_numNibbles == 9 ) {
    if( currentEnvironmentValue.message != result ) {
      currentEnvironmentValue.message = result;
      currentEnvironmentValue.humidity = result & 0xFF;
      currentEnvironmentValue.temperature = (result >> 12) & 0x0FFF;
      if(currentEnvironmentValue.temperature & 0x0800) {
        //Bit 11 ist set -> negative -> fill up bits 12-15 to make a signed short
        currentEnvironmentValue.temperature |= 0xF000;
      }
      // tell the app if call backs are registered
      if(pAppCBs && pAppCBs->pfnEnvValueChangeCB) {
        // Note: we are passing on a pointer here;
        // it needs to point to a static global value
        pAppCBs->pfnEnvValueChangeCB(&currentEnvironmentValue);
        // Note: code left here as a pattern for logging
        //      pAppCBs->pfnLoggingMessageCB(x, sizeof(x));
      } // endif
    }
    returnValue = false; // do not stop receiving
  } //end 9 nibbles
  else if( p_numNibbles == 6 ) {
    // Message with battery voltage
    uint16_t batteryVoltage = result & 0xFFFF;
    if( currentBatteryVoltage != batteryVoltage ) {
      currentBatteryVoltage = batteryVoltage;
      if( pAppCBs && pAppCBs->pfnBatteryMessageCB ) {
        pAppCBs->pfnBatteryMessageCB(batteryVoltage);
      }
    }
    returnValue = false; // do not stop receiving
  } //endif 6 nibbles
  else if( p_numNibbles == 4){
    // This is a message with the error status
    uint8_t errorStatus = result & 0xFF;
    if( (errorStatus != 0x00) && pAppCBs && pAppCBs->pfnErrorMessageCB ) {
      pAppCBs->pfnErrorMessageCB(errorStatus);
    }
    returnValue = true; // this was the last message, stop receiving
  } //endif 4 nibbles
  return returnValue;
} // end resultToCurrentValue()

receiver433_error_t Receiver433_start() {

    /* Periodic Clock */
    Clock_Params_init(&periodicClockParams);
    periodicClockParams.period = 0; // One-Shot timer
    periodicClockParams.startFlag = FALSE; // Clock_start required to start the clock
    Clock_construct(&periodicClock, clockHandler, 55000*(1000/Clock_tickPeriod), &periodicClockParams);
    hPeriodicClock = Clock_handle(&periodicClock);

    /* Open access to PIN / GPIO */
    const PIN_Config gptPinInitTable[] = {
      Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      PIN_ID(23) | PIN_INPUT_EN | PIN_NOPULL,
      PIN_TERMINATE
    };

    // Get handle to this collection of pins
    hPIN = PIN_open(&pinState, gptPinInitTable);
    if (hPIN == NULL) {
        return RCV433_ERROR_PIN;
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
        return RCV433_ERROR_CAPTURE_TIMER;
    }

    params.width = GPT_CONFIG_32BIT;
    params.mode  = GPT_MODE_PERIODIC;
    params.direction = GPTimerCC26XX_DIRECTION_UP;
    params.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
    hOverflowTimer = GPTimerCC26XX_open( CC2640R2_LAUNCHXL_GPTIMER1A, &params );
    if( hOverflowTimer == NULL ) {
        return RCV433_ERROR_OVERFLOW_TIMER;
    }

    Types_FreqHz  freq;
    BIOS_getCpuFreq(&freq);

//    if(pAppCBs && pAppCBs->pfnLoggingMessageCB) {
//        /* Logging Buffer */
//        static char logFrequency[] = "000000000000 Freq Hz";
//        // Note: we are passing on a pointer here;
//        // it needs to point to a static global value
//        sprintf(logFrequency, "%d Freq Hz", freq.lo);
//        pAppCBs->pfnLoggingMessageCB(logFrequency, sizeof(logFrequency));
//    }

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

    const GPTimerCC26XX_Value loadVal = 0xFFFFFF;
    // Configure the capture timer GPT1A as 24 bit timer
    GPTimerCC26XX_setLoadValue(hCaptureTimer, loadVal);
    // Configure timer #2 as 24 bit timer
    // Timer 2 is only used to create overflow interrupts
    // as capture timer cannot generate overflow interrupts
    // in capture mode
    GPTimerCC26XX_setLoadValue(hOverflowTimer, loadVal);

    // Set the input pin for the capture timer
    GPTimerCC26XX_PinMux pinMux = GPTimerCC26XX_getPinMux(hCaptureTimer);
    PINCC26XX_setMux(hPIN, PIN_ID(23), pinMux);
    GPTimerCC26XX_registerInterrupt(hCaptureTimer, timerCaptureCB, GPT_INT_CAPTURE );
    GPTimerCC26XX_registerInterrupt(hOverflowTimer, timerOverflowCB, GPT_INT_TIMEOUT );

    // start receiving
    startTimers();

    return RCV433_OK;
} // end receiver433_start()

void Receiver433_stop() {
    PIN_close(hPIN);
    GPTimerCC26XX_close(hCaptureTimer);
    GPTimerCC26XX_close(hOverflowTimer);
} //end receiver433_stop()

/*
 * Registers app callback functions to call when an exposed environment value changes
 */
bool Receiver433_RegisterAppCBs(const envValueCBs_t* appCBs) {
    bool result;
    if ( appCBs ) {
      // Note: just overwrite
      pAppCBs = appCBs;
      result = true;
    } else {
      result = false;
    } //endif
    return result;
} //end Receiver433_RegisterAppCBs()

