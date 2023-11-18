#include "jlb.h"


// DBC file version
#if (VER_JLB_MAJ != (0U)) || (VER_JLB_MIN != (0U))
#error The JLB dbc source files have different versions
#endif

#ifdef JLB_USE_DIAG_MONITORS
// Function prototypes to be called each time CAN frame is unpacked
// FMon function may detect RC, CRC or DLC violation
#include "jlb-fmon.h"

#endif // JLB_USE_DIAG_MONITORS

// This macro guard for the case when you need to enable
// using diag monitors but there is no necessity in proper
// SysTick provider. For providing one you need define macro
// before this line - in dbccodeconf.h

#ifndef GetSystemTick
#define GetSystemTick() (0u)
#endif

// This macro guard is for the case when you want to build
// app with enabled optoin auto CSM, but don't yet have
// proper getframehash implementation

#ifndef GetFrameHash
#define GetFrameHash(a,b,c,d,e) (0u)
#endif

// This function performs extension of sign for the signals
// which have non-aligned to power of 2 bit's width.
// The types 'bitext_t' and 'ubitext_t' define maximal bit width which
// can be correctly handled. You need to select type which can contain
// n+1 bits where n is the largest signed signal width. For example if
// the most wide signed signal has a width of 31 bits you need to set
// bitext_t as int32_t and ubitext_t as uint32_t
// Defined these typedefs in @dbccodeconf.h or locally in 'dbcdrvname'-config.h
static bitext_t __ext_sig__(ubitext_t val, uint8_t bits)
{
  ubitext_t const m = 1u << (bits - 1);
  return (val ^ m) - m;
}

uint32_t Unpack_measurements_1_jlb(measurements_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->line_sensor_1 = (uint8_t) ( (_d[0] & (0x03U)) );
  _m->line_sensor_2 = (uint8_t) ( ((_d[0] >> 2U) & (0x03U)) );
  _m->line_sensor_3 = (uint8_t) ( ((_d[0] >> 4U) & (0x03U)) );
  _m->line_sensor_4 = (uint8_t) ( ((_d[0] >> 6U) & (0x03U)) );
  _m->line_sensor_5 = (uint8_t) ( (_d[1] & (0x03U)) );
  _m->line_sensor_6 = (uint8_t) ( ((_d[1] >> 2U) & (0x03U)) );
  _m->line_sensor_7 = (uint8_t) ( ((_d[1] >> 4U) & (0x03U)) );
  _m->line_sensor_8 = (uint8_t) ( ((_d[1] >> 6U) & (0x03U)) );
  _m->line_sensor_9 = (uint8_t) ( (_d[2] & (0x03U)) );
  _m->line_sensor_10 = (uint8_t) ( ((_d[2] >> 2U) & (0x03U)) );
  _m->line_sensor_11 = (uint8_t) ( ((_d[2] >> 4U) & (0x03U)) );
  _m->line_sensor_12 = (uint8_t) ( ((_d[2] >> 6U) & (0x03U)) );
  _m->line_sensor_13 = (uint8_t) ( (_d[3] & (0x03U)) );
  _m->line_sensor_14 = (uint8_t) ( ((_d[3] >> 2U) & (0x03U)) );
  _m->line_sensor_15 = (uint8_t) ( ((_d[3] >> 4U) & (0x03U)) );
  _m->line_sensor_16 = (uint8_t) ( ((_d[3] >> 6U) & (0x03U)) );
  _m->line_sensor_17 = (uint8_t) ( (_d[4] & (0x03U)) );
  _m->line_sensor_18 = (uint8_t) ( ((_d[4] >> 2U) & (0x03U)) );
  _m->line_sensor_19 = (uint8_t) ( ((_d[4] >> 4U) & (0x03U)) );
  _m->line_sensor_20 = (uint8_t) ( ((_d[4] >> 6U) & (0x03U)) );
  _m->line_sensor_21 = (uint8_t) ( (_d[5] & (0x03U)) );
  _m->line_sensor_22 = (uint8_t) ( ((_d[5] >> 2U) & (0x03U)) );
  _m->line_sensor_23 = (uint8_t) ( ((_d[5] >> 4U) & (0x03U)) );
  _m->line_sensor_24 = (uint8_t) ( ((_d[5] >> 6U) & (0x03U)) );
  _m->line_sensor_25 = (uint8_t) ( (_d[6] & (0x03U)) );
  _m->line_sensor_26 = (uint8_t) ( ((_d[6] >> 2U) & (0x03U)) );
  _m->line_sensor_27 = (uint8_t) ( ((_d[6] >> 4U) & (0x03U)) );
  _m->line_sensor_28 = (uint8_t) ( ((_d[6] >> 6U) & (0x03U)) );
  _m->line_sensor_29 = (uint8_t) ( (_d[7] & (0x03U)) );
  _m->line_sensor_30 = (uint8_t) ( ((_d[7] >> 2U) & (0x03U)) );
  _m->line_sensor_31 = (uint8_t) ( ((_d[7] >> 4U) & (0x03U)) );
  _m->line_sensor_32 = (uint8_t) ( ((_d[7] >> 6U) & (0x03U)) );

#ifdef JLB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < measurements_1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_measurements_1_jlb(&_m->mon1, measurements_1_CANID);
#endif // JLB_USE_DIAG_MONITORS

  return measurements_1_CANID;
}

#ifdef JLB_USE_CANSTRUCT

uint32_t Pack_measurements_1_jlb(measurements_1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(measurements_1_DLC); cframe->Data[i++] = JLB_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->line_sensor_1 & (0x03U)) | ((_m->line_sensor_2 & (0x03U)) << 2U) | ((_m->line_sensor_3 & (0x03U)) << 4U) | ((_m->line_sensor_4 & (0x03U)) << 6U) );
  cframe->Data[1] |= (uint8_t) ( (_m->line_sensor_5 & (0x03U)) | ((_m->line_sensor_6 & (0x03U)) << 2U) | ((_m->line_sensor_7 & (0x03U)) << 4U) | ((_m->line_sensor_8 & (0x03U)) << 6U) );
  cframe->Data[2] |= (uint8_t) ( (_m->line_sensor_9 & (0x03U)) | ((_m->line_sensor_10 & (0x03U)) << 2U) | ((_m->line_sensor_11 & (0x03U)) << 4U) | ((_m->line_sensor_12 & (0x03U)) << 6U) );
  cframe->Data[3] |= (uint8_t) ( (_m->line_sensor_13 & (0x03U)) | ((_m->line_sensor_14 & (0x03U)) << 2U) | ((_m->line_sensor_15 & (0x03U)) << 4U) | ((_m->line_sensor_16 & (0x03U)) << 6U) );
  cframe->Data[4] |= (uint8_t) ( (_m->line_sensor_17 & (0x03U)) | ((_m->line_sensor_18 & (0x03U)) << 2U) | ((_m->line_sensor_19 & (0x03U)) << 4U) | ((_m->line_sensor_20 & (0x03U)) << 6U) );
  cframe->Data[5] |= (uint8_t) ( (_m->line_sensor_21 & (0x03U)) | ((_m->line_sensor_22 & (0x03U)) << 2U) | ((_m->line_sensor_23 & (0x03U)) << 4U) | ((_m->line_sensor_24 & (0x03U)) << 6U) );
  cframe->Data[6] |= (uint8_t) ( (_m->line_sensor_25 & (0x03U)) | ((_m->line_sensor_26 & (0x03U)) << 2U) | ((_m->line_sensor_27 & (0x03U)) << 4U) | ((_m->line_sensor_28 & (0x03U)) << 6U) );
  cframe->Data[7] |= (uint8_t) ( (_m->line_sensor_29 & (0x03U)) | ((_m->line_sensor_30 & (0x03U)) << 2U) | ((_m->line_sensor_31 & (0x03U)) << 4U) | ((_m->line_sensor_32 & (0x03U)) << 6U) );

  cframe->MsgId = (uint32_t) measurements_1_CANID;
  cframe->DLC = (uint8_t) measurements_1_DLC;
  cframe->IDE = (uint8_t) measurements_1_IDE;
  return measurements_1_CANID;
}

#else

uint32_t Pack_measurements_1_jlb(measurements_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(measurements_1_DLC); _d[i++] = JLB_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->line_sensor_1 & (0x03U)) | ((_m->line_sensor_2 & (0x03U)) << 2U) | ((_m->line_sensor_3 & (0x03U)) << 4U) | ((_m->line_sensor_4 & (0x03U)) << 6U) );
  _d[1] |= (uint8_t) ( (_m->line_sensor_5 & (0x03U)) | ((_m->line_sensor_6 & (0x03U)) << 2U) | ((_m->line_sensor_7 & (0x03U)) << 4U) | ((_m->line_sensor_8 & (0x03U)) << 6U) );
  _d[2] |= (uint8_t) ( (_m->line_sensor_9 & (0x03U)) | ((_m->line_sensor_10 & (0x03U)) << 2U) | ((_m->line_sensor_11 & (0x03U)) << 4U) | ((_m->line_sensor_12 & (0x03U)) << 6U) );
  _d[3] |= (uint8_t) ( (_m->line_sensor_13 & (0x03U)) | ((_m->line_sensor_14 & (0x03U)) << 2U) | ((_m->line_sensor_15 & (0x03U)) << 4U) | ((_m->line_sensor_16 & (0x03U)) << 6U) );
  _d[4] |= (uint8_t) ( (_m->line_sensor_17 & (0x03U)) | ((_m->line_sensor_18 & (0x03U)) << 2U) | ((_m->line_sensor_19 & (0x03U)) << 4U) | ((_m->line_sensor_20 & (0x03U)) << 6U) );
  _d[5] |= (uint8_t) ( (_m->line_sensor_21 & (0x03U)) | ((_m->line_sensor_22 & (0x03U)) << 2U) | ((_m->line_sensor_23 & (0x03U)) << 4U) | ((_m->line_sensor_24 & (0x03U)) << 6U) );
  _d[6] |= (uint8_t) ( (_m->line_sensor_25 & (0x03U)) | ((_m->line_sensor_26 & (0x03U)) << 2U) | ((_m->line_sensor_27 & (0x03U)) << 4U) | ((_m->line_sensor_28 & (0x03U)) << 6U) );
  _d[7] |= (uint8_t) ( (_m->line_sensor_29 & (0x03U)) | ((_m->line_sensor_30 & (0x03U)) << 2U) | ((_m->line_sensor_31 & (0x03U)) << 4U) | ((_m->line_sensor_32 & (0x03U)) << 6U) );

  *_len = (uint8_t) measurements_1_DLC;
  *_ide = (uint8_t) measurements_1_IDE;
  return measurements_1_CANID;
}

#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_measurements_2_jlb(measurements_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->line_sensor_1 = (uint8_t) ( (_d[0] & (0x03U)) );
  _m->line_sensor_2 = (uint8_t) ( ((_d[0] >> 2U) & (0x03U)) );
  _m->line_sensor_3 = (uint8_t) ( ((_d[0] >> 4U) & (0x03U)) );
  _m->line_sensor_4 = (uint8_t) ( ((_d[0] >> 6U) & (0x03U)) );
  _m->line_sensor_5 = (uint8_t) ( (_d[1] & (0x03U)) );
  _m->line_sensor_6 = (uint8_t) ( ((_d[1] >> 2U) & (0x03U)) );
  _m->line_sensor_7 = (uint8_t) ( ((_d[1] >> 4U) & (0x03U)) );
  _m->line_sensor_8 = (uint8_t) ( ((_d[1] >> 6U) & (0x03U)) );
  _m->line_sensor_9 = (uint8_t) ( (_d[2] & (0x03U)) );
  _m->line_sensor_10 = (uint8_t) ( ((_d[2] >> 2U) & (0x03U)) );
  _m->line_sensor_11 = (uint8_t) ( ((_d[2] >> 4U) & (0x03U)) );
  _m->line_sensor_12 = (uint8_t) ( ((_d[2] >> 6U) & (0x03U)) );
  _m->line_sensor_13 = (uint8_t) ( (_d[3] & (0x03U)) );
  _m->line_sensor_14 = (uint8_t) ( ((_d[3] >> 2U) & (0x03U)) );
  _m->line_sensor_15 = (uint8_t) ( ((_d[3] >> 4U) & (0x03U)) );
  _m->line_sensor_16 = (uint8_t) ( ((_d[3] >> 6U) & (0x03U)) );
  _m->line_sensor_17 = (uint8_t) ( (_d[4] & (0x03U)) );
  _m->line_sensor_18 = (uint8_t) ( ((_d[4] >> 2U) & (0x03U)) );
  _m->line_sensor_19 = (uint8_t) ( ((_d[4] >> 4U) & (0x03U)) );
  _m->line_sensor_20 = (uint8_t) ( ((_d[4] >> 6U) & (0x03U)) );
  _m->line_sensor_21 = (uint8_t) ( (_d[5] & (0x03U)) );
  _m->line_sensor_22 = (uint8_t) ( ((_d[5] >> 2U) & (0x03U)) );
  _m->line_sensor_23 = (uint8_t) ( ((_d[5] >> 4U) & (0x03U)) );
  _m->line_sensor_24 = (uint8_t) ( ((_d[5] >> 6U) & (0x03U)) );
  _m->line_sensor_25 = (uint8_t) ( (_d[6] & (0x03U)) );
  _m->line_sensor_26 = (uint8_t) ( ((_d[6] >> 2U) & (0x03U)) );
  _m->line_sensor_27 = (uint8_t) ( ((_d[6] >> 4U) & (0x03U)) );
  _m->line_sensor_28 = (uint8_t) ( ((_d[6] >> 6U) & (0x03U)) );
  _m->line_sensor_29 = (uint8_t) ( (_d[7] & (0x03U)) );
  _m->line_sensor_30 = (uint8_t) ( ((_d[7] >> 2U) & (0x03U)) );
  _m->line_sensor_31 = (uint8_t) ( ((_d[7] >> 4U) & (0x03U)) );
  _m->line_sensor_32 = (uint8_t) ( ((_d[7] >> 6U) & (0x03U)) );

#ifdef JLB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < measurements_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_measurements_2_jlb(&_m->mon1, measurements_2_CANID);
#endif // JLB_USE_DIAG_MONITORS

  return measurements_2_CANID;
}

#ifdef JLB_USE_CANSTRUCT

uint32_t Pack_measurements_2_jlb(measurements_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(measurements_2_DLC); cframe->Data[i++] = JLB_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->line_sensor_1 & (0x03U)) | ((_m->line_sensor_2 & (0x03U)) << 2U) | ((_m->line_sensor_3 & (0x03U)) << 4U) | ((_m->line_sensor_4 & (0x03U)) << 6U) );
  cframe->Data[1] |= (uint8_t) ( (_m->line_sensor_5 & (0x03U)) | ((_m->line_sensor_6 & (0x03U)) << 2U) | ((_m->line_sensor_7 & (0x03U)) << 4U) | ((_m->line_sensor_8 & (0x03U)) << 6U) );
  cframe->Data[2] |= (uint8_t) ( (_m->line_sensor_9 & (0x03U)) | ((_m->line_sensor_10 & (0x03U)) << 2U) | ((_m->line_sensor_11 & (0x03U)) << 4U) | ((_m->line_sensor_12 & (0x03U)) << 6U) );
  cframe->Data[3] |= (uint8_t) ( (_m->line_sensor_13 & (0x03U)) | ((_m->line_sensor_14 & (0x03U)) << 2U) | ((_m->line_sensor_15 & (0x03U)) << 4U) | ((_m->line_sensor_16 & (0x03U)) << 6U) );
  cframe->Data[4] |= (uint8_t) ( (_m->line_sensor_17 & (0x03U)) | ((_m->line_sensor_18 & (0x03U)) << 2U) | ((_m->line_sensor_19 & (0x03U)) << 4U) | ((_m->line_sensor_20 & (0x03U)) << 6U) );
  cframe->Data[5] |= (uint8_t) ( (_m->line_sensor_21 & (0x03U)) | ((_m->line_sensor_22 & (0x03U)) << 2U) | ((_m->line_sensor_23 & (0x03U)) << 4U) | ((_m->line_sensor_24 & (0x03U)) << 6U) );
  cframe->Data[6] |= (uint8_t) ( (_m->line_sensor_25 & (0x03U)) | ((_m->line_sensor_26 & (0x03U)) << 2U) | ((_m->line_sensor_27 & (0x03U)) << 4U) | ((_m->line_sensor_28 & (0x03U)) << 6U) );
  cframe->Data[7] |= (uint8_t) ( (_m->line_sensor_29 & (0x03U)) | ((_m->line_sensor_30 & (0x03U)) << 2U) | ((_m->line_sensor_31 & (0x03U)) << 4U) | ((_m->line_sensor_32 & (0x03U)) << 6U) );

  cframe->MsgId = (uint32_t) measurements_2_CANID;
  cframe->DLC = (uint8_t) measurements_2_DLC;
  cframe->IDE = (uint8_t) measurements_2_IDE;
  return measurements_2_CANID;
}

#else

uint32_t Pack_measurements_2_jlb(measurements_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(measurements_2_DLC); _d[i++] = JLB_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->line_sensor_1 & (0x03U)) | ((_m->line_sensor_2 & (0x03U)) << 2U) | ((_m->line_sensor_3 & (0x03U)) << 4U) | ((_m->line_sensor_4 & (0x03U)) << 6U) );
  _d[1] |= (uint8_t) ( (_m->line_sensor_5 & (0x03U)) | ((_m->line_sensor_6 & (0x03U)) << 2U) | ((_m->line_sensor_7 & (0x03U)) << 4U) | ((_m->line_sensor_8 & (0x03U)) << 6U) );
  _d[2] |= (uint8_t) ( (_m->line_sensor_9 & (0x03U)) | ((_m->line_sensor_10 & (0x03U)) << 2U) | ((_m->line_sensor_11 & (0x03U)) << 4U) | ((_m->line_sensor_12 & (0x03U)) << 6U) );
  _d[3] |= (uint8_t) ( (_m->line_sensor_13 & (0x03U)) | ((_m->line_sensor_14 & (0x03U)) << 2U) | ((_m->line_sensor_15 & (0x03U)) << 4U) | ((_m->line_sensor_16 & (0x03U)) << 6U) );
  _d[4] |= (uint8_t) ( (_m->line_sensor_17 & (0x03U)) | ((_m->line_sensor_18 & (0x03U)) << 2U) | ((_m->line_sensor_19 & (0x03U)) << 4U) | ((_m->line_sensor_20 & (0x03U)) << 6U) );
  _d[5] |= (uint8_t) ( (_m->line_sensor_21 & (0x03U)) | ((_m->line_sensor_22 & (0x03U)) << 2U) | ((_m->line_sensor_23 & (0x03U)) << 4U) | ((_m->line_sensor_24 & (0x03U)) << 6U) );
  _d[6] |= (uint8_t) ( (_m->line_sensor_25 & (0x03U)) | ((_m->line_sensor_26 & (0x03U)) << 2U) | ((_m->line_sensor_27 & (0x03U)) << 4U) | ((_m->line_sensor_28 & (0x03U)) << 6U) );
  _d[7] |= (uint8_t) ( (_m->line_sensor_29 & (0x03U)) | ((_m->line_sensor_30 & (0x03U)) << 2U) | ((_m->line_sensor_31 & (0x03U)) << 4U) | ((_m->line_sensor_32 & (0x03U)) << 6U) );

  *_len = (uint8_t) measurements_2_DLC;
  *_ide = (uint8_t) measurements_2_IDE;
  return measurements_2_CANID;
}

#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_measurements_3_jlb(measurements_3_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->angular_velocity_x_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->angular_velocity_x_phys = (sigfloat_t)(JLB_angular_velocity_x_ro_fromS(_m->angular_velocity_x_ro));
#endif // JLB_USE_SIGFLOAT

  _m->angular_velocity_y_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->angular_velocity_y_phys = (sigfloat_t)(JLB_angular_velocity_y_ro_fromS(_m->angular_velocity_y_ro));
#endif // JLB_USE_SIGFLOAT

  _m->angular_velocity_z_ro = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->angular_velocity_z_phys = (sigfloat_t)(JLB_angular_velocity_z_ro_fromS(_m->angular_velocity_z_ro));
#endif // JLB_USE_SIGFLOAT

#ifdef JLB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < measurements_3_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_measurements_3_jlb(&_m->mon1, measurements_3_CANID);
#endif // JLB_USE_DIAG_MONITORS

  return measurements_3_CANID;
}

#ifdef JLB_USE_CANSTRUCT

uint32_t Pack_measurements_3_jlb(measurements_3_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(measurements_3_DLC); cframe->Data[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->angular_velocity_x_ro = (uint16_t) JLB_angular_velocity_x_ro_toS(_m->angular_velocity_x_phys);
  _m->angular_velocity_y_ro = (uint16_t) JLB_angular_velocity_y_ro_toS(_m->angular_velocity_y_phys);
  _m->angular_velocity_z_ro = (uint16_t) JLB_angular_velocity_z_ro_toS(_m->angular_velocity_z_phys);
#endif // JLB_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->angular_velocity_x_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->angular_velocity_x_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->angular_velocity_y_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->angular_velocity_y_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->angular_velocity_z_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->angular_velocity_z_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) measurements_3_CANID;
  cframe->DLC = (uint8_t) measurements_3_DLC;
  cframe->IDE = (uint8_t) measurements_3_IDE;
  return measurements_3_CANID;
}

#else

uint32_t Pack_measurements_3_jlb(measurements_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(measurements_3_DLC); _d[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->angular_velocity_x_ro = (uint16_t) JLB_angular_velocity_x_ro_toS(_m->angular_velocity_x_phys);
  _m->angular_velocity_y_ro = (uint16_t) JLB_angular_velocity_y_ro_toS(_m->angular_velocity_y_phys);
  _m->angular_velocity_z_ro = (uint16_t) JLB_angular_velocity_z_ro_toS(_m->angular_velocity_z_phys);
#endif // JLB_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->angular_velocity_x_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->angular_velocity_x_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->angular_velocity_y_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->angular_velocity_y_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->angular_velocity_z_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->angular_velocity_z_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) measurements_3_DLC;
  *_ide = (uint8_t) measurements_3_IDE;
  return measurements_3_CANID;
}

#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_measurements_4_jlb(measurements_4_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->linear_acceleration_x_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->linear_acceleration_x_phys = (sigfloat_t)(JLB_linear_acceleration_x_ro_fromS(_m->linear_acceleration_x_ro));
#endif // JLB_USE_SIGFLOAT

  _m->linear_acceleration_y_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->linear_acceleration_y_phys = (sigfloat_t)(JLB_linear_acceleration_y_ro_fromS(_m->linear_acceleration_y_ro));
#endif // JLB_USE_SIGFLOAT

  _m->linear_acceleration_z_ro = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->linear_acceleration_z_phys = (sigfloat_t)(JLB_linear_acceleration_z_ro_fromS(_m->linear_acceleration_z_ro));
#endif // JLB_USE_SIGFLOAT

#ifdef JLB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < measurements_4_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_measurements_4_jlb(&_m->mon1, measurements_4_CANID);
#endif // JLB_USE_DIAG_MONITORS

  return measurements_4_CANID;
}

#ifdef JLB_USE_CANSTRUCT

uint32_t Pack_measurements_4_jlb(measurements_4_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(measurements_4_DLC); cframe->Data[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->linear_acceleration_x_ro = (uint16_t) JLB_linear_acceleration_x_ro_toS(_m->linear_acceleration_x_phys);
  _m->linear_acceleration_y_ro = (uint16_t) JLB_linear_acceleration_y_ro_toS(_m->linear_acceleration_y_phys);
  _m->linear_acceleration_z_ro = (uint16_t) JLB_linear_acceleration_z_ro_toS(_m->linear_acceleration_z_phys);
#endif // JLB_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->linear_acceleration_x_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->linear_acceleration_x_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->linear_acceleration_y_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->linear_acceleration_y_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->linear_acceleration_z_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->linear_acceleration_z_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) measurements_4_CANID;
  cframe->DLC = (uint8_t) measurements_4_DLC;
  cframe->IDE = (uint8_t) measurements_4_IDE;
  return measurements_4_CANID;
}

#else

uint32_t Pack_measurements_4_jlb(measurements_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(measurements_4_DLC); _d[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->linear_acceleration_x_ro = (uint16_t) JLB_linear_acceleration_x_ro_toS(_m->linear_acceleration_x_phys);
  _m->linear_acceleration_y_ro = (uint16_t) JLB_linear_acceleration_y_ro_toS(_m->linear_acceleration_y_phys);
  _m->linear_acceleration_z_ro = (uint16_t) JLB_linear_acceleration_z_ro_toS(_m->linear_acceleration_z_phys);
#endif // JLB_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->linear_acceleration_x_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->linear_acceleration_x_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->linear_acceleration_y_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->linear_acceleration_y_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->linear_acceleration_z_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->linear_acceleration_z_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) measurements_4_DLC;
  *_ide = (uint8_t) measurements_4_IDE;
  return measurements_4_CANID;
}

#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_measurements_5_jlb(measurements_5_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->motor_rpm_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->motor_rpm_phys = (sigfloat_t)(JLB_motor_rpm_ro_fromS(_m->motor_rpm_ro));
#endif // JLB_USE_SIGFLOAT

  _m->object_range_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->object_range_phys = (sigfloat_t)(JLB_object_range_ro_fromS(_m->object_range_ro));
#endif // JLB_USE_SIGFLOAT

#ifdef JLB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < measurements_5_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_measurements_5_jlb(&_m->mon1, measurements_5_CANID);
#endif // JLB_USE_DIAG_MONITORS

  return measurements_5_CANID;
}

#ifdef JLB_USE_CANSTRUCT

uint32_t Pack_measurements_5_jlb(measurements_5_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(measurements_5_DLC); cframe->Data[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->motor_rpm_ro = (uint16_t) JLB_motor_rpm_ro_toS(_m->motor_rpm_phys);
  _m->object_range_ro = (uint16_t) JLB_object_range_ro_toS(_m->object_range_phys);
#endif // JLB_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->motor_rpm_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->motor_rpm_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->object_range_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->object_range_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) measurements_5_CANID;
  cframe->DLC = (uint8_t) measurements_5_DLC;
  cframe->IDE = (uint8_t) measurements_5_IDE;
  return measurements_5_CANID;
}

#else

uint32_t Pack_measurements_5_jlb(measurements_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(measurements_5_DLC); _d[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->motor_rpm_ro = (uint16_t) JLB_motor_rpm_ro_toS(_m->motor_rpm_phys);
  _m->object_range_ro = (uint16_t) JLB_object_range_ro_toS(_m->object_range_phys);
#endif // JLB_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->motor_rpm_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->motor_rpm_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->object_range_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->object_range_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) measurements_5_DLC;
  *_ide = (uint8_t) measurements_5_IDE;
  return measurements_5_CANID;
}

#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_odometry_1_jlb(odometry_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->orientation_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->orientation_phys = (sigfloat_t)(JLB_orientation_ro_fromS(_m->orientation_ro));
#endif // JLB_USE_SIGFLOAT

  _m->position_x_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->position_x_phys = (sigfloat_t)(JLB_position_x_ro_fromS(_m->position_x_ro));
#endif // JLB_USE_SIGFLOAT

  _m->position_y_ro = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->position_y_phys = (sigfloat_t)(JLB_position_y_ro_fromS(_m->position_y_ro));
#endif // JLB_USE_SIGFLOAT

#ifdef JLB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < odometry_1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_odometry_1_jlb(&_m->mon1, odometry_1_CANID);
#endif // JLB_USE_DIAG_MONITORS

  return odometry_1_CANID;
}

#ifdef JLB_USE_CANSTRUCT

uint32_t Pack_odometry_1_jlb(odometry_1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(odometry_1_DLC); cframe->Data[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->orientation_ro = (uint16_t) JLB_orientation_ro_toS(_m->orientation_phys);
  _m->position_x_ro = (uint16_t) JLB_position_x_ro_toS(_m->position_x_phys);
  _m->position_y_ro = (uint16_t) JLB_position_y_ro_toS(_m->position_y_phys);
#endif // JLB_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->orientation_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->orientation_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->position_x_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->position_x_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->position_y_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->position_y_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) odometry_1_CANID;
  cframe->DLC = (uint8_t) odometry_1_DLC;
  cframe->IDE = (uint8_t) odometry_1_IDE;
  return odometry_1_CANID;
}

#else

uint32_t Pack_odometry_1_jlb(odometry_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(odometry_1_DLC); _d[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->orientation_ro = (uint16_t) JLB_orientation_ro_toS(_m->orientation_phys);
  _m->position_x_ro = (uint16_t) JLB_position_x_ro_toS(_m->position_x_phys);
  _m->position_y_ro = (uint16_t) JLB_position_y_ro_toS(_m->position_y_phys);
#endif // JLB_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->orientation_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->orientation_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->position_x_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->position_x_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->position_y_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->position_y_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) odometry_1_DLC;
  *_ide = (uint8_t) odometry_1_IDE;
  return odometry_1_CANID;
}

#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_odometry_2_jlb(odometry_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->angular_velocity_z_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->angular_velocity_z_phys = (sigfloat_t)(JLB_angular_velocity_z_ro_fromS(_m->angular_velocity_z_ro));
#endif // JLB_USE_SIGFLOAT

  _m->linear_velocity_x_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->linear_velocity_x_phys = (sigfloat_t)(JLB_linear_velocity_x_ro_fromS(_m->linear_velocity_x_ro));
#endif // JLB_USE_SIGFLOAT

#ifdef JLB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < odometry_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_odometry_2_jlb(&_m->mon1, odometry_2_CANID);
#endif // JLB_USE_DIAG_MONITORS

  return odometry_2_CANID;
}

#ifdef JLB_USE_CANSTRUCT

uint32_t Pack_odometry_2_jlb(odometry_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(odometry_2_DLC); cframe->Data[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->angular_velocity_z_ro = (uint16_t) JLB_angular_velocity_z_ro_toS(_m->angular_velocity_z_phys);
  _m->linear_velocity_x_ro = (uint16_t) JLB_linear_velocity_x_ro_toS(_m->linear_velocity_x_phys);
#endif // JLB_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->angular_velocity_z_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->angular_velocity_z_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->linear_velocity_x_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->linear_velocity_x_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) odometry_2_CANID;
  cframe->DLC = (uint8_t) odometry_2_DLC;
  cframe->IDE = (uint8_t) odometry_2_IDE;
  return odometry_2_CANID;
}

#else

uint32_t Pack_odometry_2_jlb(odometry_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(odometry_2_DLC); _d[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->angular_velocity_z_ro = (uint16_t) JLB_angular_velocity_z_ro_toS(_m->angular_velocity_z_phys);
  _m->linear_velocity_x_ro = (uint16_t) JLB_linear_velocity_x_ro_toS(_m->linear_velocity_x_phys);
#endif // JLB_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->angular_velocity_z_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->angular_velocity_z_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->linear_velocity_x_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->linear_velocity_x_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) odometry_2_DLC;
  *_ide = (uint8_t) odometry_2_IDE;
  return odometry_2_CANID;
}

#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_logic_1_jlb(logic_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->target_angle_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->target_angle_phys = (sigfloat_t)(JLB_target_angle_ro_fromS(_m->target_angle_ro));
#endif // JLB_USE_SIGFLOAT

  _m->target_speed_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->target_speed_phys = (sigfloat_t)(JLB_target_speed_ro_fromS(_m->target_speed_ro));
#endif // JLB_USE_SIGFLOAT

  _m->cross_track_error_ro = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->cross_track_error_phys = (sigfloat_t)(JLB_cross_track_error_ro_fromS(_m->cross_track_error_ro));
#endif // JLB_USE_SIGFLOAT

  _m->heading_error_ro = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );
#ifdef JLB_USE_SIGFLOAT
  _m->heading_error_phys = (sigfloat_t)(JLB_heading_error_ro_fromS(_m->heading_error_ro));
#endif // JLB_USE_SIGFLOAT

#ifdef JLB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < logic_1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_logic_1_jlb(&_m->mon1, logic_1_CANID);
#endif // JLB_USE_DIAG_MONITORS

  return logic_1_CANID;
}

#ifdef JLB_USE_CANSTRUCT

uint32_t Pack_logic_1_jlb(logic_1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(logic_1_DLC); cframe->Data[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->target_angle_ro = (uint16_t) JLB_target_angle_ro_toS(_m->target_angle_phys);
  _m->target_speed_ro = (uint16_t) JLB_target_speed_ro_toS(_m->target_speed_phys);
  _m->cross_track_error_ro = (uint16_t) JLB_cross_track_error_ro_toS(_m->cross_track_error_phys);
  _m->heading_error_ro = (uint16_t) JLB_heading_error_ro_toS(_m->heading_error_phys);
#endif // JLB_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->target_angle_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->target_angle_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->target_speed_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->target_speed_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->cross_track_error_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->cross_track_error_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->heading_error_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->heading_error_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) logic_1_CANID;
  cframe->DLC = (uint8_t) logic_1_DLC;
  cframe->IDE = (uint8_t) logic_1_IDE;
  return logic_1_CANID;
}

#else

uint32_t Pack_logic_1_jlb(logic_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(logic_1_DLC); _d[i++] = JLB_INITIAL_BYTE_VALUE);

#ifdef JLB_USE_SIGFLOAT
  _m->target_angle_ro = (uint16_t) JLB_target_angle_ro_toS(_m->target_angle_phys);
  _m->target_speed_ro = (uint16_t) JLB_target_speed_ro_toS(_m->target_speed_phys);
  _m->cross_track_error_ro = (uint16_t) JLB_cross_track_error_ro_toS(_m->cross_track_error_phys);
  _m->heading_error_ro = (uint16_t) JLB_heading_error_ro_toS(_m->heading_error_phys);
#endif // JLB_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->target_angle_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->target_angle_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->target_speed_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->target_speed_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->cross_track_error_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->cross_track_error_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->heading_error_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->heading_error_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) logic_1_DLC;
  *_ide = (uint8_t) logic_1_IDE;
  return logic_1_CANID;
}

#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_logic_2_jlb(logic_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->state = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->direction = (uint8_t) ( (_d[1] & (0xFFU)) );
  _m->previous_node = (uint8_t) ( (_d[2] & (0xFFU)) );
  _m->next_node = (uint8_t) ( (_d[3] & (0xFFU)) );
  _m->at_cross_section = (uint8_t) ( (_d[4] & (0x01U)) );
  _m->under_gate = (uint8_t) ( ((_d[4] >> 1U) & (0x01U)) );

#ifdef JLB_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < logic_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_logic_2_jlb(&_m->mon1, logic_2_CANID);
#endif // JLB_USE_DIAG_MONITORS

  return logic_2_CANID;
}

#ifdef JLB_USE_CANSTRUCT

uint32_t Pack_logic_2_jlb(logic_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(logic_2_DLC); cframe->Data[i++] = JLB_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->state & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->direction & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->previous_node & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->next_node & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->at_cross_section & (0x01U)) | ((_m->under_gate & (0x01U)) << 1U) );

  cframe->MsgId = (uint32_t) logic_2_CANID;
  cframe->DLC = (uint8_t) logic_2_DLC;
  cframe->IDE = (uint8_t) logic_2_IDE;
  return logic_2_CANID;
}

#else

uint32_t Pack_logic_2_jlb(logic_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < JLB_VALIDATE_DLC(logic_2_DLC); _d[i++] = JLB_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->state & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->direction & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->previous_node & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->next_node & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->at_cross_section & (0x01U)) | ((_m->under_gate & (0x01U)) << 1U) );

  *_len = (uint8_t) logic_2_DLC;
  *_ide = (uint8_t) logic_2_IDE;
  return logic_2_CANID;
}

#endif // JLB_USE_CANSTRUCT

