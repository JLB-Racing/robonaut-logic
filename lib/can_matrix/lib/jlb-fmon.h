#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// DBC file version
#define VER_JLB_MAJ_FMON (0U)
#define VER_JLB_MIN_FMON (0U)

#include "jlb-config.h"

#ifdef JLB_USE_DIAG_MONITORS

#include "canmonitorutil.h"
/*
This file contains the prototypes of all the functions that will be called
from each Unpack_*name* function to detect DBC related errors
It is the user responsibility to defined these functions in the
separated .c file. If it won't be done the linkage error will happen
*/

#ifdef JLB_USE_MONO_FMON

void _FMon_MONO_jlb(FrameMonitor_t* _mon, uint32_t msgid);

#define FMon_measurements_1_jlb(x, y) _FMon_MONO_jlb((x), (y))
#define FMon_measurements_2_jlb(x, y) _FMon_MONO_jlb((x), (y))
#define FMon_measurements_3_jlb(x, y) _FMon_MONO_jlb((x), (y))
#define FMon_measurements_4_jlb(x, y) _FMon_MONO_jlb((x), (y))
#define FMon_odometry_1_jlb(x, y) _FMon_MONO_jlb((x), (y))
#define FMon_odometry_2_jlb(x, y) _FMon_MONO_jlb((x), (y))
#define FMon_logic_1_jlb(x, y) _FMon_MONO_jlb((x), (y))

#else

void _FMon_measurements_1_jlb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_measurements_2_jlb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_measurements_3_jlb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_measurements_4_jlb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_odometry_1_jlb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_odometry_2_jlb(FrameMonitor_t* _mon, uint32_t msgid);
void _FMon_logic_1_jlb(FrameMonitor_t* _mon, uint32_t msgid);

#define FMon_measurements_1_jlb(x, y) _FMon_measurements_1_jlb((x), (y))
#define FMon_measurements_2_jlb(x, y) _FMon_measurements_2_jlb((x), (y))
#define FMon_measurements_3_jlb(x, y) _FMon_measurements_3_jlb((x), (y))
#define FMon_measurements_4_jlb(x, y) _FMon_measurements_4_jlb((x), (y))
#define FMon_odometry_1_jlb(x, y) _FMon_odometry_1_jlb((x), (y))
#define FMon_odometry_2_jlb(x, y) _FMon_odometry_2_jlb((x), (y))
#define FMon_logic_1_jlb(x, y) _FMon_logic_1_jlb((x), (y))

#endif

#endif // JLB_USE_DIAG_MONITORS

#ifdef __cplusplus
}
#endif
