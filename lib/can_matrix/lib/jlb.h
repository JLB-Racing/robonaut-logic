#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// DBC file version
#define VER_JLB_MAJ (0U)
#define VER_JLB_MIN (0U)

// include current dbc-driver compilation config
#include "jlb-config.h"

#ifdef JLB_USE_DIAG_MONITORS
// This file must define:
// base monitor struct
#include "canmonitorutil.h"

#endif // JLB_USE_DIAG_MONITORS


// DLC maximum value which is used as the limit for frame's data buffer size.
// Client can set its own value (not sure why) in driver-config
// or can test it on some limit specified by application
// e.g.: static_assert(TESTDB_MAX_DLC_VALUE <= APPLICATION_FRAME_DATA_SIZE, "Max DLC value in the driver is too big")
#ifndef JLB_MAX_DLC_VALUE
// The value which was found out by generator (real max value)
#define JLB_MAX_DLC_VALUE 8U
#endif

// The limit is used for setting frame's data bytes
#define JLB_VALIDATE_DLC(msgDlc) (((msgDlc) <= (JLB_MAX_DLC_VALUE)) ? (msgDlc) : (JLB_MAX_DLC_VALUE))

// Initial byte value to be filles in data bytes of the frame before pack signals
// User can define its own custom value in driver-config file
#ifndef JLB_INITIAL_BYTE_VALUE
#define JLB_INITIAL_BYTE_VALUE 0U
#endif


// def @measurements_1 CAN Message (0    0)
#define measurements_1_IDE (0U)
#define measurements_1_DLC (8U)
#define measurements_1_CANID (0U)
// signal: @angular_velocity_x_ro
#define JLB_angular_velocity_x_ro_CovFactor (0.0002)
#define JLB_angular_velocity_x_ro_toS(x) ( (uint16_t) (((x) - (-6.5535)) / (0.0002)) )
#define JLB_angular_velocity_x_ro_fromS(x) ( (((x) * (0.0002)) + (-6.5535)) )
// signal: @angular_velocity_y_ro
#define JLB_angular_velocity_y_ro_CovFactor (0.0002)
#define JLB_angular_velocity_y_ro_toS(x) ( (uint16_t) (((x) - (-6.5535)) / (0.0002)) )
#define JLB_angular_velocity_y_ro_fromS(x) ( (((x) * (0.0002)) + (-6.5535)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint8_t line_sensor_1 : 1;                 //      Bits= 1

  uint8_t line_sensor_2 : 1;                 //      Bits= 1

  uint8_t line_sensor_3 : 1;                 //      Bits= 1

  uint8_t line_sensor_4 : 1;                 //      Bits= 1

  uint8_t line_sensor_5 : 1;                 //      Bits= 1

  uint8_t line_sensor_6 : 1;                 //      Bits= 1

  uint8_t line_sensor_7 : 1;                 //      Bits= 1

  uint8_t line_sensor_8 : 1;                 //      Bits= 1

  uint8_t line_sensor_9 : 1;                 //      Bits= 1

  uint8_t line_sensor_10 : 1;                //      Bits= 1

  uint8_t line_sensor_11 : 1;                //      Bits= 1

  uint8_t line_sensor_12 : 1;                //      Bits= 1

  uint8_t line_sensor_13 : 1;                //      Bits= 1

  uint8_t line_sensor_14 : 1;                //      Bits= 1

  uint8_t line_sensor_15 : 1;                //      Bits= 1

  uint8_t line_sensor_16 : 1;                //      Bits= 1

  uint8_t line_sensor_17 : 1;                //      Bits= 1

  uint8_t line_sensor_18 : 1;                //      Bits= 1

  uint8_t line_sensor_19 : 1;                //      Bits= 1

  uint8_t line_sensor_20 : 1;                //      Bits= 1

  uint8_t line_sensor_21 : 1;                //      Bits= 1

  uint8_t line_sensor_22 : 1;                //      Bits= 1

  uint8_t line_sensor_23 : 1;                //      Bits= 1

  uint8_t line_sensor_24 : 1;                //      Bits= 1

  uint8_t line_sensor_25 : 1;                //      Bits= 1

  uint8_t line_sensor_26 : 1;                //      Bits= 1

  uint8_t line_sensor_27 : 1;                //      Bits= 1

  uint8_t line_sensor_28 : 1;                //      Bits= 1

  uint8_t line_sensor_29 : 1;                //      Bits= 1

  uint8_t line_sensor_30 : 1;                //      Bits= 1

  uint8_t line_sensor_31 : 1;                //      Bits= 1

  uint8_t line_sensor_32 : 1;                //      Bits= 1

  uint16_t angular_velocity_x_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_x_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t angular_velocity_y_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_y_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint8_t line_sensor_1;                     //      Bits= 1

  uint8_t line_sensor_2;                     //      Bits= 1

  uint8_t line_sensor_3;                     //      Bits= 1

  uint8_t line_sensor_4;                     //      Bits= 1

  uint8_t line_sensor_5;                     //      Bits= 1

  uint8_t line_sensor_6;                     //      Bits= 1

  uint8_t line_sensor_7;                     //      Bits= 1

  uint8_t line_sensor_8;                     //      Bits= 1

  uint8_t line_sensor_9;                     //      Bits= 1

  uint8_t line_sensor_10;                    //      Bits= 1

  uint8_t line_sensor_11;                    //      Bits= 1

  uint8_t line_sensor_12;                    //      Bits= 1

  uint8_t line_sensor_13;                    //      Bits= 1

  uint8_t line_sensor_14;                    //      Bits= 1

  uint8_t line_sensor_15;                    //      Bits= 1

  uint8_t line_sensor_16;                    //      Bits= 1

  uint8_t line_sensor_17;                    //      Bits= 1

  uint8_t line_sensor_18;                    //      Bits= 1

  uint8_t line_sensor_19;                    //      Bits= 1

  uint8_t line_sensor_20;                    //      Bits= 1

  uint8_t line_sensor_21;                    //      Bits= 1

  uint8_t line_sensor_22;                    //      Bits= 1

  uint8_t line_sensor_23;                    //      Bits= 1

  uint8_t line_sensor_24;                    //      Bits= 1

  uint8_t line_sensor_25;                    //      Bits= 1

  uint8_t line_sensor_26;                    //      Bits= 1

  uint8_t line_sensor_27;                    //      Bits= 1

  uint8_t line_sensor_28;                    //      Bits= 1

  uint8_t line_sensor_29;                    //      Bits= 1

  uint8_t line_sensor_30;                    //      Bits= 1

  uint8_t line_sensor_31;                    //      Bits= 1

  uint8_t line_sensor_32;                    //      Bits= 1

  uint16_t angular_velocity_x_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_x_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t angular_velocity_y_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_y_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} measurements_1_t;

// def @measurements_2 CAN Message (1    0x1)
#define measurements_2_IDE (0U)
#define measurements_2_DLC (8U)
#define measurements_2_CANID (0x1U)
// signal: @angular_velocity_z_ro
#define JLB_angular_velocity_z_ro_CovFactor (0.0002)
#define JLB_angular_velocity_z_ro_toS(x) ( (uint16_t) (((x) - (-6.5535)) / (0.0002)) )
#define JLB_angular_velocity_z_ro_fromS(x) ( (((x) * (0.0002)) + (-6.5535)) )
// signal: @linear_acceleration_x_ro
#define JLB_linear_acceleration_x_ro_CovFactor (0.001)
#define JLB_linear_acceleration_x_ro_toS(x) ( (uint16_t) (((x) - (-32.7675)) / (0.001)) )
#define JLB_linear_acceleration_x_ro_fromS(x) ( (((x) * (0.001)) + (-32.7675)) )
// signal: @linear_acceleration_y_ro
#define JLB_linear_acceleration_y_ro_CovFactor (0.001)
#define JLB_linear_acceleration_y_ro_toS(x) ( (uint16_t) (((x) - (-32.7675)) / (0.001)) )
#define JLB_linear_acceleration_y_ro_fromS(x) ( (((x) * (0.001)) + (-32.7675)) )
// signal: @linear_acceleration_z_ro
#define JLB_linear_acceleration_z_ro_CovFactor (0.001)
#define JLB_linear_acceleration_z_ro_toS(x) ( (uint16_t) (((x) - (-32.7675)) / (0.001)) )
#define JLB_linear_acceleration_z_ro_fromS(x) ( (((x) * (0.001)) + (-32.7675)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint16_t angular_velocity_z_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_z_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_acceleration_x_ro;         //      Bits=16 Offset= -32.7675           Factor= 0.001           Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_x_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_acceleration_y_ro;         //      Bits=16 Offset= -32.7675           Factor= 0.001           Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_y_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_acceleration_z_ro;         //      Bits=16 Offset= -32.7675           Factor= 0.001           Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_z_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint16_t angular_velocity_z_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_z_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_acceleration_x_ro;         //      Bits=16 Offset= -32.7675           Factor= 0.001           Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_x_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_acceleration_y_ro;         //      Bits=16 Offset= -32.7675           Factor= 0.001           Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_y_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_acceleration_z_ro;         //      Bits=16 Offset= -32.7675           Factor= 0.001           Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_z_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} measurements_2_t;

// def @measurements_3 CAN Message (2    0x2)
#define measurements_3_IDE (0U)
#define measurements_3_DLC (2U)
#define measurements_3_CANID (0x2U)
// signal: @motor_rpm_ro
#define JLB_motor_rpm_ro_CovFactor (0.1)
#define JLB_motor_rpm_ro_toS(x) ( (uint16_t) (((x) - (-3276.75)) / (0.1)) )
#define JLB_motor_rpm_ro_fromS(x) ( (((x) * (0.1)) + (-3276.75)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint16_t motor_rpm_ro;                     //      Bits=16 Offset= -3276.75           Factor= 0.1             Unit:'RPM'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t motor_rpm_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint16_t motor_rpm_ro;                     //      Bits=16 Offset= -3276.75           Factor= 0.1             Unit:'RPM'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t motor_rpm_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} measurements_3_t;

// Function signatures

uint32_t Unpack_measurements_1_jlb(measurements_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_measurements_1_jlb(measurements_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_measurements_1_jlb(measurements_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_measurements_2_jlb(measurements_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_measurements_2_jlb(measurements_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_measurements_2_jlb(measurements_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_measurements_3_jlb(measurements_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_measurements_3_jlb(measurements_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_measurements_3_jlb(measurements_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

#ifdef __cplusplus
}
#endif
