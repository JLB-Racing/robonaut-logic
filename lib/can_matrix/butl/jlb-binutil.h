#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "dbccodeconf.h"

#include "jlb.h"

typedef struct
{
  measurements_1_t measurements_1;
  measurements_2_t measurements_2;
  measurements_3_t measurements_3;
} jlb_rx_t;

// There is no any TX mapped massage.

uint32_t jlb_Receive(jlb_rx_t* m, const uint8_t* d, uint32_t msgid, uint8_t dlc);

#ifdef __DEF_JLB__

extern jlb_rx_t jlb_rx;

#endif // __DEF_JLB__

#ifdef __cplusplus
}
#endif
