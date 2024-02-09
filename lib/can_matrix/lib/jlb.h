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
// signal: @wheel_rpm_ro
#define JLB_wheel_rpm_ro_CovFactor (0.1)
#define JLB_wheel_rpm_ro_toS(x) ( (uint16_t) (((x) - (-3276.75)) / (0.1)) )
#define JLB_wheel_rpm_ro_fromS(x) ( (((x) * (0.1)) + (-3276.75)) )
// signal: @object_range_ro
#define JLB_object_range_ro_CovFactor (0.0001)
#define JLB_object_range_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.0001)) )
#define JLB_object_range_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @motor_current_ro
#define JLB_motor_current_ro_CovFactor (0.001)
#define JLB_motor_current_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define JLB_motor_current_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @duty_cycle_ro
#define JLB_duty_cycle_ro_CovFactor (0.0000175)
#define JLB_duty_cycle_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.0000175)) )
#define JLB_duty_cycle_ro_fromS(x) ( (((x) * (0.0000175)) + (0.0)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint16_t wheel_rpm_ro;                     //      Bits=16 Offset= -3276.75           Factor= 0.1             Unit:'RPM'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t wheel_rpm_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t object_range_ro;                  //      Bits=16 Factor= 0.0001          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t object_range_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t motor_current_ro;                 //      Bits=16 Factor= 0.001           Unit:'A'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t motor_current_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t duty_cycle_ro;                    //      Bits=16 Factor= 0.0000175

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t duty_cycle_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint16_t wheel_rpm_ro;                     //      Bits=16 Offset= -3276.75           Factor= 0.1             Unit:'RPM'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t wheel_rpm_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t object_range_ro;                  //      Bits=16 Factor= 0.0001          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t object_range_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t motor_current_ro;                 //      Bits=16 Factor= 0.001           Unit:'A'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t motor_current_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t duty_cycle_ro;                    //      Bits=16 Factor= 0.0000175

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t duty_cycle_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} measurements_5_t;

// def @measurements_6 CAN Message (6    0x6)
#define measurements_6_IDE (0U)
#define measurements_6_DLC (8U)
#define measurements_6_CANID (0x6U)
// signal: @hv_battery_voltage_ro
#define JLB_hv_battery_voltage_ro_CovFactor (0.00025)
#define JLB_hv_battery_voltage_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.00025)) )
#define JLB_hv_battery_voltage_ro_fromS(x) ( (((x) * (0.00025)) + (0.0)) )
// signal: @lv_battery_voltage_ro
#define JLB_lv_battery_voltage_ro_CovFactor (0.00025)
#define JLB_lv_battery_voltage_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.00025)) )
#define JLB_lv_battery_voltage_ro_fromS(x) ( (((x) * (0.00025)) + (0.0)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint16_t hv_battery_voltage_ro;            //      Bits=16 Factor= 0.00025         Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t hv_battery_voltage_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t lv_battery_voltage_ro;            //      Bits=16 Factor= 0.00025         Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t lv_battery_voltage_phys;
#endif // JLB_USE_SIGFLOAT

  uint8_t deadman_switch : 1;                //      Bits= 1

#else

  uint16_t hv_battery_voltage_ro;            //      Bits=16 Factor= 0.00025         Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t hv_battery_voltage_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t lv_battery_voltage_ro;            //      Bits=16 Factor= 0.00025         Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t lv_battery_voltage_phys;
#endif // JLB_USE_SIGFLOAT

  uint8_t deadman_switch;                    //      Bits= 1

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} measurements_6_t;

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
// signal: @cross_track_error_ro
#define JLB_cross_track_error_ro_CovFactor (0.0001)
#define JLB_cross_track_error_ro_toS(x) ( (uint16_t) (((x) - (-3.27675)) / (0.0001)) )
#define JLB_cross_track_error_ro_fromS(x) ( (((x) * (0.0001)) + (-3.27675)) )
// signal: @heading_error_ro
#define JLB_heading_error_ro_CovFactor (0.00005)
#define JLB_heading_error_ro_toS(x) ( (uint16_t) (((x) - (-1.638375)) / (0.00005)) )
#define JLB_heading_error_ro_fromS(x) ( (((x) * (0.00005)) + (-1.638375)) )

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

  uint16_t cross_track_error_ro;             //      Bits=16 Offset= -3.27675           Factor= 0.0001          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t cross_track_error_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t heading_error_ro;                 //      Bits=16 Offset= -1.638375          Factor= 0.00005         Unit:'rad'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t heading_error_phys;
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

  uint16_t cross_track_error_ro;             //      Bits=16 Offset= -3.27675           Factor= 0.0001          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t cross_track_error_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t heading_error_ro;                 //      Bits=16 Offset= -1.638375          Factor= 0.00005         Unit:'rad'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t heading_error_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} logic_1_t;

// def @logic_2 CAN Message (34   0x22)
#define logic_2_IDE (0U)
#define logic_2_DLC (8U)
#define logic_2_CANID (0x22U)
// signal: @distance_traveled_ro
#define JLB_distance_traveled_ro_CovFactor (0.01)
#define JLB_distance_traveled_ro_toS(x) ( (uint16_t) (((x) - (-327.675)) / (0.01)) )
#define JLB_distance_traveled_ro_fromS(x) ( (((x) * (0.01)) + (-327.675)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint8_t direction;                         //      Bits= 8 Unit:'enum'

  uint8_t mission;                           //      Bits= 8 Unit:'enum'

  uint8_t fast_state;                        //      Bits= 8 Unit:'enum'

  uint8_t labyrinth_state;                   //      Bits= 8 Unit:'enum'

  uint8_t next_node;                         //      Bits= 8 Unit:'char'

  uint8_t previous_node;                     //      Bits= 8 Unit:'char'

  uint16_t distance_traveled_ro;             //      Bits=16 Offset= -327.675           Factor= 0.01            Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t distance_traveled_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint8_t direction;                         //      Bits= 8 Unit:'enum'

  uint8_t mission;                           //      Bits= 8 Unit:'enum'

  uint8_t fast_state;                        //      Bits= 8 Unit:'enum'

  uint8_t labyrinth_state;                   //      Bits= 8 Unit:'enum'

  uint8_t next_node;                         //      Bits= 8 Unit:'char'

  uint8_t previous_node;                     //      Bits= 8 Unit:'char'

  uint16_t distance_traveled_ro;             //      Bits=16 Offset= -327.675           Factor= 0.01            Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t distance_traveled_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} logic_2_t;

// def @logic_3 CAN Message (35   0x23)
#define logic_3_IDE (0U)
#define logic_3_DLC (8U)
#define logic_3_CANID (0x23U)
// signal: @ang_error_norm_ro
#define JLB_ang_error_norm_ro_CovFactor (0.004)
#define JLB_ang_error_norm_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.004)) )
#define JLB_ang_error_norm_ro_fromS(x) ( (((x) * (0.004)) + (0.0)) )
// signal: @dist_error_norm_ro
#define JLB_dist_error_norm_ro_CovFactor (0.004)
#define JLB_dist_error_norm_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.004)) )
#define JLB_dist_error_norm_ro_fromS(x) ( (((x) * (0.004)) + (0.0)) )
// signal: @line_position_front_ro
#define JLB_line_position_front_ro_CovFactor (0.0005)
#define JLB_line_position_front_ro_toS(x) ( (uint16_t) (((x) - (-16.38375)) / (0.0005)) )
#define JLB_line_position_front_ro_fromS(x) ( (((x) * (0.0005)) + (-16.38375)) )
// signal: @line_position_rear_ro
#define JLB_line_position_rear_ro_CovFactor (0.0005)
#define JLB_line_position_rear_ro_toS(x) ( (uint16_t) (((x) - (-16.38375)) / (0.0005)) )
#define JLB_line_position_rear_ro_fromS(x) ( (((x) * (0.0005)) + (-16.38375)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint8_t ang_error_norm_ro;                 //      Bits= 8 Factor= 0.004

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t ang_error_norm_phys;
#endif // JLB_USE_SIGFLOAT

  uint8_t dist_error_norm_ro;                //      Bits= 8 Factor= 0.004

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t dist_error_norm_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t line_position_front_ro;           //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t line_position_front_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t line_position_rear_ro;            //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t line_position_rear_phys;
#endif // JLB_USE_SIGFLOAT

  uint8_t at_cross_section : 1;              //      Bits= 1

  uint8_t under_gate : 1;                    //      Bits= 1

#else

  uint8_t ang_error_norm_ro;                 //      Bits= 8 Factor= 0.004

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t ang_error_norm_phys;
#endif // JLB_USE_SIGFLOAT

  uint8_t dist_error_norm_ro;                //      Bits= 8 Factor= 0.004

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t dist_error_norm_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t line_position_front_ro;           //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t line_position_front_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t line_position_rear_ro;            //      Bits=16 Offset= -16.38375          Factor= 0.0005          Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t line_position_rear_phys;
#endif // JLB_USE_SIGFLOAT

  uint8_t at_cross_section;                  //      Bits= 1

  uint8_t under_gate;                        //      Bits= 1

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} logic_3_t;

// def @logic_4 CAN Message (36   0x24)
#define logic_4_IDE (0U)
#define logic_4_DLC (8U)
#define logic_4_CANID (0x24U)
// signal: @best_laptime_ro
#define JLB_best_laptime_ro_CovFactor (0.01)
#define JLB_best_laptime_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define JLB_best_laptime_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @current_laptime_ro
#define JLB_current_laptime_ro_CovFactor (0.01)
#define JLB_current_laptime_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define JLB_current_laptime_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @target_distance_ro
#define JLB_target_distance_ro_CovFactor (0.01)
#define JLB_target_distance_ro_toS(x) ( (uint16_t) (((x) - (-327.675)) / (0.01)) )
#define JLB_target_distance_ro_fromS(x) ( (((x) * (0.01)) + (-327.675)) )
// signal: @last_laptime_ro
#define JLB_last_laptime_ro_CovFactor (0.01)
#define JLB_last_laptime_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define JLB_last_laptime_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint8_t best_laptime_ro;                   //      Bits= 8 Factor= 0.01            Unit:'s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t best_laptime_phys;
#endif // JLB_USE_SIGFLOAT

  uint8_t current_laptime_ro;                //      Bits= 8 Factor= 0.01            Unit:'s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t current_laptime_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t target_distance_ro;               //      Bits=16 Offset= -327.675           Factor= 0.01            Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t target_distance_phys;
#endif // JLB_USE_SIGFLOAT

  uint8_t mission_switch_state;              //      Bits= 8 Unit:'enum'

  uint8_t goal_node;                         //      Bits= 8 Unit:'char'

  uint8_t last_laptime_ro;                   //      Bits= 8 Factor= 0.01            Unit:'s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t last_laptime_phys;
#endif // JLB_USE_SIGFLOAT

#else

  uint8_t best_laptime_ro;                   //      Bits= 8 Factor= 0.01            Unit:'s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t best_laptime_phys;
#endif // JLB_USE_SIGFLOAT

  uint8_t current_laptime_ro;                //      Bits= 8 Factor= 0.01            Unit:'s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t current_laptime_phys;
#endif // JLB_USE_SIGFLOAT

  uint16_t target_distance_ro;               //      Bits=16 Offset= -327.675           Factor= 0.01            Unit:'m'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t target_distance_phys;
#endif // JLB_USE_SIGFLOAT

  uint8_t mission_switch_state;              //      Bits= 8 Unit:'enum'

  uint8_t goal_node;                         //      Bits= 8 Unit:'char'

  uint8_t last_laptime_ro;                   //      Bits= 8 Factor= 0.01            Unit:'s'

#ifdef JLB_USE_SIGFLOAT
  sigfloat_t last_laptime_phys;
#endif // JLB_USE_SIGFLOAT

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} logic_4_t;

// def @logic_5 CAN Message (37   0x25)
#define logic_5_IDE (0U)
#define logic_5_DLC (8U)
#define logic_5_CANID (0x25U)

typedef struct
{
#ifdef JLB_USE_BITS_SIGNAL

  uint8_t pirate_after_next;                 //      Bits= 8 Unit:'char'

  uint8_t pirate_next;                       //      Bits= 8 Unit:'char'

  uint8_t pirate_previous;                   //      Bits= 8 Unit:'char'

  uint8_t follow_car : 1;                    //      Bits= 1

  uint8_t flood : 1;                         //      Bits= 1

  uint8_t collected_valid_gates : 5;         //      Bits= 5

  uint8_t collected_gates : 5;               //      Bits= 5

  uint8_t controller_type;                   //      Bits= 8

#else

  uint8_t pirate_after_next;                 //      Bits= 8 Unit:'char'

  uint8_t pirate_next;                       //      Bits= 8 Unit:'char'

  uint8_t pirate_previous;                   //      Bits= 8 Unit:'char'

  uint8_t follow_car;                        //      Bits= 1

  uint8_t flood;                             //      Bits= 1

  uint8_t collected_valid_gates;             //      Bits= 5

  uint8_t collected_gates;                   //      Bits= 5

  uint8_t controller_type;                   //      Bits= 8

#endif // JLB_USE_BITS_SIGNAL

#ifdef JLB_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // JLB_USE_DIAG_MONITORS

} logic_5_t;

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

uint32_t Unpack_measurements_6_jlb(measurements_6_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_measurements_6_jlb(measurements_6_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_measurements_6_jlb(measurements_6_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
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

uint32_t Unpack_logic_2_jlb(logic_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_logic_2_jlb(logic_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_logic_2_jlb(logic_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_logic_3_jlb(logic_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_logic_3_jlb(logic_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_logic_3_jlb(logic_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_logic_4_jlb(logic_4_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_logic_4_jlb(logic_4_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_logic_4_jlb(logic_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

uint32_t Unpack_logic_5_jlb(logic_5_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef JLB_USE_CANSTRUCT
uint32_t Pack_logic_5_jlb(logic_5_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_logic_5_jlb(logic_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // JLB_USE_CANSTRUCT

#ifdef __cplusplus
}
#endif
