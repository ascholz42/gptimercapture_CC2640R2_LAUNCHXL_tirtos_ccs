/*
 * receiver.c
 *
 *  Created on: 05.11.2023
 *      Author: User
 */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include "receiver.h"

bool synced = false;
uint8_t nibble_count;
uint8_t bit_mask = 0;
uint8_t input = 0;
uint64_t result = 0;

uint32_t count_3800_us;
uint32_t count_4200_us;
uint32_t count_18000_us;
uint32_t count_1800_us;
uint32_t count_2200_us;
uint32_t count_800_us;
uint32_t count_1200_us;


int processEdge(uint32_t width, app_msg_edge_t edgeType, bool missed_edges) {
    if( !synced && (width > count_3800_us) && (width < count_4200_us) && (edgeType == APP_MSG_POS_EDGE) )
    {
      synced = true;
      nibble_count = 0;
      input = 0;
      bit_mask = 0b00001000;
      result = 0;
    } else if (synced && missed_edges) {
      // anything shorter kicks the app out of sync
      // ignore everything until the next sync
      synced = false;
      nibble_count = 0;
    } else if (synced && (edgeType == APP_MSG_POS_EDGE)) {
      //
      if (width > count_18000_us) {
        // A long LOW phase terminates the transmission
        // After sending the superhetheo receiver will
        // not receive anything until it starts to
        // raise its sensitivity and returns to receiving
        // white noise again.
        // This app waits for the next sync
#ifdef DEBUG
        // Display something?
#endif
        // wait for the next sync
        synced = false;
        nibble_count = 0;
      } else if ((width > count_3800_us) && (width < count_4200_us)) {
        // This is a re-sync and a repeat will follow
        nibble_count = 0;
        input = 0;
        bit_mask = 0b00001000;
        result = 0;
      } else {
        // looks like a valid bit
        if (width > count_800_us && width < count_1200_us) {
            // logical zero
            input &= ~bit_mask;
        } else if ((width > count_1800_us) && (width < count_2200_us)) {
          // logical one
          input |= bit_mask;
        } else {
          // Error: this is neither One nor Zero -> wait for the next sync
          synced = false;
        } // endif check for valid bit
        // adjust the bit_mask for the next bit
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
        } // endif bitmask wrap
      } // endif re-sync then... else
    }// endif synced && RISING_EDGE
return nibble_count;
}

uint64_t getMessage() {
    return result;
}
