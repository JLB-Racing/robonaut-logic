#include "jlb-binutil.h"

// DBC file version
#if (VER_JLB_MAJ != (0U)) || (VER_JLB_MIN != (0U))
#error The JLB binutil source file has inconsistency with core dbc lib!
#endif

#ifdef __DEF_JLB__

jlb_rx_t jlb_rx;

#endif // __DEF_JLB__

uint32_t jlb_Receive(jlb_rx_t* _m, const uint8_t* _d, uint32_t _id, uint8_t dlc_)
{
 uint32_t recid = 0;
 if (_id == 0x0U) {
  recid = Unpack_measurements_1_jlb(&(_m->measurements_1), _d, dlc_);
 } else {
  if (_id == 0x1U) {
   recid = Unpack_measurements_2_jlb(&(_m->measurements_2), _d, dlc_);
  } else if (_id == 0x2U) {
   recid = Unpack_measurements_3_jlb(&(_m->measurements_3), _d, dlc_);
  }
 }

 return recid;
}

