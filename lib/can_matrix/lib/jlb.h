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


// def @measurements_1 CAN Message (1    0x1)
#define measurements_1_IDE (0U)
#define measurements_1_DLC (8U)
#define measurements_1_CANID (0x1U)

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint8_t line_sensor_1 : 2;                 //      Bits= 2

  uint8_t line_sensor_2 : 2;                 //      Bits= 2

  uint8_t line_sensor_3 : 2;                 //      Bits= 2

  uint8_t line_sensor_4 : 2;                 //      Bits= 2

  uint8_t line_sensor_5 : 2;                 //      Bits= 2

  uint8_t line_sensor_6 : 2;                 //      Bits= 2

  uint8_t line_sensor_7 : 2;                 //      Bits= 2

  uint8_t line_sensor_8 : 2;                 //      Bits= 2

  uint8_t line_sensor_9 : 2;                 //      Bits= 2

  uint8_t line_sensor_10 : 2;                //      Bits= 2

  uint8_t line_sensor_11 : 2;                //      Bits= 2

  uint8_t line_sensor_12 : 2;                //      Bits= 2

  uint8_t line_sensor_13 : 2;                //      Bits= 2

  uint8_t line_sensor_14 : 2;                //      Bits= 2

  uint8_t line_sensor_15 : 2;                //      Bits= 2

  uint8_t line_sensor_16 : 2;                //      Bits= 2

  uint8_t line_sensor_17 : 2;                //      Bits= 2

  uint8_t line_sensor_18 : 2;                //      Bits= 2

  uint8_t line_sensor_19 : 2;                //      Bits= 2

  uint8_t line_sensor_20 : 2;                //      Bits= 2

  uint8_t line_sensor_21 : 2;                //      Bits= 2

  uint8_t line_sensor_22 : 2;                //      Bits= 2

  uint8_t line_sensor_23 : 2;                //      Bits= 2

  uint8_t line_sensor_24 : 2;                //      Bits= 2

  uint8_t line_sensor_25 : 2;                //      Bits= 2

  uint8_t line_sensor_26 : 2;                //      Bits= 2

  uint8_t line_sensor_27 : 2;                //      Bits= 2

  uint8_t line_sensor_28 : 2;                //      Bits= 2

  uint8_t line_sensor_29 : 2;                //      Bits= 2

  uint8_t line_sensor_30 : 2;                //      Bits= 2

  uint8_t line_sensor_31 : 2;                //      Bits= 2

  uint8_t line_sensor_32 : 2;                //      Bits= 2

#else

  uint8_t line_sensor_1;                     //      Bits= 2

  uint8_t line_sensor_2;                     //      Bits= 2

  uint8_t line_sensor_3;                     //      Bits= 2

  uint8_t line_sensor_4;                     //      Bits= 2

  uint8_t line_sensor_5;                     //      Bits= 2

  uint8_t line_sensor_6;                     //      Bits= 2

  uint8_t line_sensor_7;                     //      Bits= 2

  uint8_t line_sensor_8;                     //      Bits= 2

  uint8_t line_sensor_9;                     //      Bits= 2

  uint8_t line_sensor_10;                    //      Bits= 2

  uint8_t line_sensor_11;                    //      Bits= 2

  uint8_t line_sensor_12;                    //      Bits= 2

  uint8_t line_sensor_13;                    //      Bits= 2

  uint8_t line_sensor_14;                    //      Bits= 2

  uint8_t line_sensor_15;                    //      Bits= 2

  uint8_t line_sensor_16;                    //      Bits= 2

  uint8_t line_sensor_17;                    //      Bits= 2

  uint8_t line_sensor_18;                    //      Bits= 2

  uint8_t line_sensor_19;                    //      Bits= 2

  uint8_t line_sensor_20;                    //      Bits= 2

  uint8_t line_sensor_21;                    //      Bits= 2

  uint8_t line_sensor_22;                    //      Bits= 2

  uint8_t line_sensor_23;                    //      Bits= 2

  uint8_t line_sensor_24;                    //      Bits= 2

  uint8_t line_sensor_25;                    //      Bits= 2

  uint8_t line_sensor_26;                    //      Bits= 2

  uint8_t line_sensor_27;                    //      Bits= 2

  uint8_t line_sensor_28;                    //      Bits= 2

  uint8_t line_sensor_29;                    //      Bits= 2

  uint8_t line_sensor_30;                    //      Bits= 2

  uint8_t line_sensor_31;                    //      Bits= 2

  uint8_t line_sensor_32;                    //      Bits= 2

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} measurements_1_t;

// def @measurements_2 CAN Message (2    0x2)
#define measurements_2_IDE (0U)
#define measurements_2_DLC (8U)
#define measurements_2_CANID (0x2U)

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint8_t line_sensor_1 : 2;                 //      Bits= 2

  uint8_t line_sensor_2 : 2;                 //      Bits= 2

  uint8_t line_sensor_3 : 2;                 //      Bits= 2

  uint8_t line_sensor_4 : 2;                 //      Bits= 2

  uint8_t line_sensor_5 : 2;                 //      Bits= 2

  uint8_t line_sensor_6 : 2;                 //      Bits= 2

  uint8_t line_sensor_7 : 2;                 //      Bits= 2

  uint8_t line_sensor_8 : 2;                 //      Bits= 2

  uint8_t line_sensor_9 : 2;                 //      Bits= 2

  uint8_t line_sensor_10 : 2;                //      Bits= 2

  uint8_t line_sensor_11 : 2;                //      Bits= 2

  uint8_t line_sensor_12 : 2;                //      Bits= 2

  uint8_t line_sensor_13 : 2;                //      Bits= 2

  uint8_t line_sensor_14 : 2;                //      Bits= 2

  uint8_t line_sensor_15 : 2;                //      Bits= 2

  uint8_t line_sensor_16 : 2;                //      Bits= 2

  uint8_t line_sensor_17 : 2;                //      Bits= 2

  uint8_t line_sensor_18 : 2;                //      Bits= 2

  uint8_t line_sensor_19 : 2;                //      Bits= 2

  uint8_t line_sensor_20 : 2;                //      Bits= 2

  uint8_t line_sensor_21 : 2;                //      Bits= 2

  uint8_t line_sensor_22 : 2;                //      Bits= 2

  uint8_t line_sensor_23 : 2;                //      Bits= 2

  uint8_t line_sensor_24 : 2;                //      Bits= 2

  uint8_t line_sensor_25 : 2;                //      Bits= 2

  uint8_t line_sensor_26 : 2;                //      Bits= 2

  uint8_t line_sensor_27 : 2;                //      Bits= 2

  uint8_t line_sensor_28 : 2;                //      Bits= 2

  uint8_t line_sensor_29 : 2;                //      Bits= 2

  uint8_t line_sensor_30 : 2;                //      Bits= 2

  uint8_t line_sensor_31 : 2;                //      Bits= 2

  uint8_t line_sensor_32 : 2;                //      Bits= 2

#else

  uint8_t line_sensor_1;                     //      Bits= 2

  uint8_t line_sensor_2;                     //      Bits= 2

  uint8_t line_sensor_3;                     //      Bits= 2

  uint8_t line_sensor_4;                     //      Bits= 2

  uint8_t line_sensor_5;                     //      Bits= 2

  uint8_t line_sensor_6;                     //      Bits= 2

  uint8_t line_sensor_7;                     //      Bits= 2

  uint8_t line_sensor_8;                     //      Bits= 2

  uint8_t line_sensor_9;                     //      Bits= 2

  uint8_t line_sensor_10;                    //      Bits= 2

  uint8_t line_sensor_11;                    //      Bits= 2

  uint8_t line_sensor_12;                    //      Bits= 2

  uint8_t line_sensor_13;                    //      Bits= 2

  uint8_t line_sensor_14;                    //      Bits= 2

  uint8_t line_sensor_15;                    //      Bits= 2

  uint8_t line_sensor_16;                    //      Bits= 2

  uint8_t line_sensor_17;                    //      Bits= 2

  uint8_t line_sensor_18;                    //      Bits= 2

  uint8_t line_sensor_19;                    //      Bits= 2

  uint8_t line_sensor_20;                    //      Bits= 2

  uint8_t line_sensor_21;                    //      Bits= 2

  uint8_t line_sensor_22;                    //      Bits= 2

  uint8_t line_sensor_23;                    //      Bits= 2

  uint8_t line_sensor_24;                    //      Bits= 2

  uint8_t line_sensor_25;                    //      Bits= 2

  uint8_t line_sensor_26;                    //      Bits= 2

  uint8_t line_sensor_27;                    //      Bits= 2

  uint8_t line_sensor_28;                    //      Bits= 2

  uint8_t line_sensor_29;                    //      Bits= 2

  uint8_t line_sensor_30;                    //      Bits= 2

  uint8_t line_sensor_31;                    //      Bits= 2

  uint8_t line_sensor_32;                    //      Bits= 2

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} measurements_2_t;

// def @measurements_3 CAN Message (3    0x3)
#define measurements_3_IDE (0U)
#define measurements_3_DLC (8U)
#define measurements_3_CANID (0x3U)
// signal: @angular_velocity_x_ro
#define JLB_angular_velocity_x_ro_CovFactor (0.0002)
#define JLB_angular_velocity_x_ro_toS(x) ( (uint16_t) (((x) - (-6.5535)) / (0.0002)) )
#define JLB_angular_velocity_x_ro_fromS(x) ( (((x) * (0.0002)) + (-6.5535)) )
// signal: @angular_velocity_y_ro
#define JLB_angular_velocity_y_ro_CovFactor (0.0002)
#define JLB_angular_velocity_y_ro_toS(x) ( (uint16_t) (((x) - (-6.5535)) / (0.0002)) )
#define JLB_angular_velocity_y_ro_fromS(x) ( (((x) * (0.0002)) + (-6.5535)) )
// signal: @angular_velocity_z_ro
#define JLB_angular_velocity_z_ro_CovFactor (0.0002)
#define JLB_angular_velocity_z_ro_toS(x) ( (uint16_t) (((x) - (-6.5535)) / (0.0002)) )
#define JLB_angular_velocity_z_ro_fromS(x) ( (((x) * (0.0002)) + (-6.5535)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint16_t angular_velocity_x_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_x_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t angular_velocity_y_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_y_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t angular_velocity_z_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_z_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint16_t angular_velocity_x_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_x_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t angular_velocity_y_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_y_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t angular_velocity_z_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_z_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} measurements_3_t;

// def @measurements_4 CAN Message (4    0x4)
#define measurements_4_IDE (0U)
#define measurements_4_DLC (8U)
#define measurements_4_CANID (0x4U)
// signal: @linear_acceleration_x_ro
#define JLB_linear_acceleration_x_ro_CovFactor (0.0005)
#define JLB_linear_acceleration_x_ro_toS(x) ( (uint16_t) (((x) - (-16.38375)) / (0.0005)) )
#define JLB_linear_acceleration_x_ro_fromS(x) ( (((x) * (0.0005)) + (-16.38375)) )
// signal: @linear_acceleration_y_ro
#define JLB_linear_acceleration_y_ro_CovFactor (0.0005)
#define JLB_linear_acceleration_y_ro_toS(x) ( (uint16_t) (((x) - (-16.38375)) / (0.0005)) )
#define JLB_linear_acceleration_y_ro_fromS(x) ( (((x) * (0.0005)) + (-16.38375)) )
// signal: @linear_acceleration_z_ro
#define JLB_linear_acceleration_z_ro_CovFactor (0.0005)
#define JLB_linear_acceleration_z_ro_toS(x) ( (uint16_t) (((x) - (-16.38375)) / (0.0005)) )
#define JLB_linear_acceleration_z_ro_fromS(x) ( (((x) * (0.0005)) + (-16.38375)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint16_t linear_acceleration_x_ro;         //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_x_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_acceleration_y_ro;         //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_y_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_acceleration_z_ro;         //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_z_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint16_t linear_acceleration_x_ro;         //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_x_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_acceleration_y_ro;         //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_y_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_acceleration_z_ro;         //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m/s^2'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_acceleration_z_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} measurements_4_t;

// def @measurements_5 CAN Message (5    0x5)
#define measurements_5_IDE (0U)
#define measurements_5_DLC (8U)
#define measurements_5_CANID (0x5U)
// signal: @motor_rpm_ro
#define JLB_motor_rpm_ro_CovFactor (0.05)
#define JLB_motor_rpm_ro_toS(x) ( (uint16_t) (((x) - (-1638.375)) / (0.05)) )
#define JLB_motor_rpm_ro_fromS(x) ( (((x) * (0.05)) + (-1638.375)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint16_t motor_rpm_ro;                     //      Bits=16 Offset= -1638.375          Factor= 0.05            Unit:'RPM'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t motor_rpm_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint16_t motor_rpm_ro;                     //      Bits=16 Offset= -1638.375          Factor= 0.05            Unit:'RPM'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t motor_rpm_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} measurements_5_t;

// def @odometry_1 CAN Message (17   0x11)
#define odometry_1_IDE (0U)
#define odometry_1_DLC (8U)
#define odometry_1_CANID (0x11U)
// signal: @orientation_ro
#define JLB_orientation_ro_CovFactor (0.0002)
#define JLB_orientation_ro_toS(x) ( (uint16_t) (((x) - (-6.5535)) / (0.0002)) )
#define JLB_orientation_ro_fromS(x) ( (((x) * (0.0002)) + (-6.5535)) )
// signal: @position_x_ro
#define JLB_position_x_ro_CovFactor (0.0005)
#define JLB_position_x_ro_toS(x) ( (uint16_t) (((x) - (-16.38375)) / (0.0005)) )
#define JLB_position_x_ro_fromS(x) ( (((x) * (0.0005)) + (-16.38375)) )
// signal: @position_y_ro
#define JLB_position_y_ro_CovFactor (0.0005)
#define JLB_position_y_ro_toS(x) ( (uint16_t) (((x) - (-16.38375)) / (0.0005)) )
#define JLB_position_y_ro_fromS(x) ( (((x) * (0.0005)) + (-16.38375)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint16_t orientation_ro;                   //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t orientation_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t position_x_ro;                    //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t position_x_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t position_y_ro;                    //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t position_y_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint16_t orientation_ro;                   //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t orientation_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t position_x_ro;                    //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t position_x_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t position_y_ro;                    //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t position_y_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} odometry_1_t;

// def @odometry_2 CAN Message (18   0x12)
#define odometry_2_IDE (0U)
#define odometry_2_DLC (8U)
#define odometry_2_CANID (0x12U)
// signal: @linear_velocity_x_ro
#define JLB_linear_velocity_x_ro_CovFactor (0.0005)
#define JLB_linear_velocity_x_ro_toS(x) ( (uint16_t) (((x) - (-16.38375)) / (0.0005)) )
#define JLB_linear_velocity_x_ro_fromS(x) ( (((x) * (0.0005)) + (-16.38375)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint16_t angular_velocity_z_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_z_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_velocity_x_ro;             //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_velocity_x_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint16_t angular_velocity_z_ro;            //      Bits=16 Offset= -6.5535            Factor= 0.0002          Unit:'rad/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t angular_velocity_z_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t linear_velocity_x_ro;             //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t linear_velocity_x_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} odometry_2_t;

// def @logic_1 CAN Message (33   0x21)
#define logic_1_IDE (0U)
#define logic_1_DLC (8U)
#define logic_1_CANID (0x21U)
// signal: @target_angle_ro
#define JLB_target_angle_ro_CovFactor (0.0001)
#define JLB_target_angle_ro_toS(x) ( (uint16_t) (((x) - (-3.27675)) / (0.0001)) )
#define JLB_target_angle_ro_fromS(x) ( (((x) * (0.0001)) + (-3.27675)) )
// signal: @target_speed_ro
#define JLB_target_speed_ro_CovFactor (0.0005)
#define JLB_target_speed_ro_toS(x) ( (uint16_t) (((x) - (-16.38375)) / (0.0005)) )
#define JLB_target_speed_ro_fromS(x) ( (((x) * (0.0005)) + (-16.38375)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint16_t target_angle_ro;                  //      Bits=16 Offset= -3.27675           Factor= 0.0001          Unit:'rad'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t target_angle_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t target_speed_ro;                  //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t target_speed_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint16_t target_angle_ro;                  //      Bits=16 Offset= -3.27675           Factor= 0.0001          Unit:'rad'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t target_angle_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t target_speed_ro;                  //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m/s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t target_speed_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} logic_1_t;

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

uint32_t Unpack_measurements_4_jlb(measurements_4_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_measurements_4_jlb(measurements_4_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_measurements_4_jlb(measurements_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_measurements_5_jlb(measurements_5_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_measurements_5_jlb(measurements_5_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_measurements_5_jlb(measurements_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_odometry_1_jlb(odometry_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_odometry_1_jlb(odometry_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_odometry_1_jlb(odometry_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_odometry_2_jlb(odometry_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_odometry_2_jlb(odometry_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_odometry_2_jlb(odometry_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_logic_1_jlb(logic_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_logic_1_jlb(logic_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_logic_1_jlb(logic_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

#ifdef __cplusplus
}
#endif
