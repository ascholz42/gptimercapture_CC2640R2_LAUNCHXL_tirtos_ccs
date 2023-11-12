/*
 * receiver.h
 *
 *  Created on: 05.11.2023
 *      Author: User
 */

#ifndef RECEIVER_H_
#define RECEIVER_H_

typedef enum
{
  APP_MSG_POS_EDGE = 0,
  APP_MSG_NEG_EDGE = 1
} app_msg_edge_t;

extern uint32_t count_3800_us;
extern uint32_t count_4200_us;
extern uint32_t count_18000_us;
extern uint32_t count_1800_us;
extern uint32_t count_2200_us;
extern uint32_t count_800_us;
extern uint32_t count_1200_us;


extern int processEdge(uint32_t width_us, app_msg_edge_t edgeType, bool missed_edges);

extern uint64_t getMessage();

#endif /* RECEIVER_H_ */
