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
 if ((_id >= 0x1U) && (_id < 0x11U)) {
  if ((_id >= 0x1U) && (_id < 0x4U)) {
   if (_id == 0x1U) {
    recid = Unpack_measurements_1_jlb(&(_m->measurements_1), _d, dlc_);
   } else {
    if (_id == 0x2U) {
     recid = Unpack_measurements_2_jlb(&(_m->measurements_2), _d, dlc_);
    } else if (_id == 0x3U) {
     recid = Unpack_measurements_3_jlb(&(_m->measurements_3), _d, dlc_);
    }
   }
  } else {
   if (_id == 0x4U) {
    recid = Unpack_measurements_4_jlb(&(_m->measurements_4), _d, dlc_);
   } else {
    if (_id == 0x5U) {
     recid = Unpack_measurements_5_jlb(&(_m->measurements_5), _d, dlc_);
    } else if (_id == 0x6U) {
     recid = Unpack_measurements_6_jlb(&(_m->measurements_6), _d, dlc_);
    }
   }
  }
 } else {
  if ((_id >= 0x11U) && (_id < 0x22U)) {
   if (_id == 0x11U) {
    recid = Unpack_odometry_1_jlb(&(_m->odometry_1), _d, dlc_);
   } else {
    if (_id == 0x12U) {
     recid = Unpack_odometry_2_jlb(&(_m->odometry_2), _d, dlc_);
    } else if (_id == 0x21U) {
     recid = Unpack_logic_1_jlb(&(_m->logic_1), _d, dlc_);
    }
   }
  } else {
   if ((_id >= 0x22U) && (_id < 0x24U)) {
    if (_id == 0x22U) {
     recid = Unpack_logic_2_jlb(&(_m->logic_2), _d, dlc_);
    } else if (_id == 0x23U) {
     recid = Unpack_logic_3_jlb(&(_m->logic_3), _d, dlc_);
    }
   } else {
    if (_id == 0x24U) {
     recid = Unpack_logic_4_jlb(&(_m->logic_4), _d, dlc_);
    } else if (_id == 0x25U) {
     recid = Unpack_logic_5_jlb(&(_m->logic_5), _d, dlc_);
    }
   }
  }
 }

 return recid;
}

