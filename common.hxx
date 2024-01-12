#ifndef COMMON_HXX
#define COMMON_HXX

#include "types.hxx"

///////////////////////////////////////////////////////////////////////////
//
//      DEFINES
//

#define SIMULATION

namespace jlb
{
#ifndef SIMULATION

#include "stm32l5xx_hal.h"

    ///////////////////////////////////////////////////////////////////////////
    //
    //      LOGIC
    //

    /* STATIC PARAMETERS OF THE TRACK */
    PARAM float    SQUARE_LENGTH = 0.6;  // m
    PARAM unsigned BITMAP_SIZE   = 64;   // px

    /* AS STATE MACHINE*/
    PARAM float STATE_TRANSITION_TIME_LIMIT = 0.0f;
    PARAM float STATE_MIN_TIME              = 0.5f;

    ///////////////////////////////////////////////////////////////////////////
    //
    //      ODOMETRY
    //

    /* STATIC PARAMETERS OF THE VEHICLE */
    PARAM float WHEEL_RADIUS_UNLOADED = 0.0525f;                       // m
    PARAM float WHEEL_DIAMETER        = 2.0f * WHEEL_RADIUS_UNLOADED;  // m
    PARAM int   SENSOR_COUNT          = 32;                            // -
    PARAM float SENSOR_BASE           = 0.5f;                          // m
    PARAM float SENSOR_WIDTH          = 0.2f;                          // m
    PARAM float MAX_WHEEL_ANGLE       = 22.5f;                         // deg
    PARAM float WHEEL_WIDTH           = 0.05f;                         // m
    PARAM float TRACK                 = 0.26f;                         // m
    PARAM float WHEELBASE             = 0.275f;                        // m

    /* DYNAMIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_VELOCITY = 12.5f;        // m/s
    PARAM float MAX_YAW_RATE = 1.5f * M_PI;  // rad/s

    /* ALGORITHM PARAMETERS */
    PARAM int VELOCITY_BUFFER_SIZE = 1;
    PARAM int IMU_BUFFER_SIZE      = 10;

    ///////////////////////////////////////////////////////////////////////////
    //
    //      CONTROLLER
    //

    /* OBJECT PID CONTROLLER PARAMETERS */
    namespace obj
    {
        PARAM float kP                      = 4.20f;
        PARAM float kI                      = 0.69f;
        PARAM float kD                      = 0.0f;
        PARAM float TAU                     = 0.05f;
        PARAM float T                       = 0.005f;
        PARAM float LIM_MIN                 = 0.0f;
        PARAM float LIM_MAX                 = 1.0f;
        PARAM float DEADBAND                = 0.05f;
        PARAM float DERIVATIVE_FILTER_ALPHA = 0.1f;
        PARAM float FOLLOW_DISTANCE         = 0.3f;
    }  // namespace obj

    /* LATERAL PID CONTROLLER PARAMETERS */
    namespace lat
    {
        PARAM float kP                      = 6.9f;
        PARAM float kI                      = 4.20f;
        PARAM float kD                      = 0.0f;
        PARAM float TAU                     = 0.05f;
        PARAM float T                       = 0.005f;
        PARAM float LIM_MIN                 = -MAX_WHEEL_ANGLE;
        PARAM float LIM_MAX                 = MAX_WHEEL_ANGLE;
        PARAM float DEADBAND                = 0.5f;
        PARAM float DERIVATIVE_FILTER_ALPHA = 0.0f;
    }  // namespace lat

    PARAM float OFFSET  = 0.25f;
    PARAM float SLOPE   = 1.0f;
    PARAM float DAMPING = 0.9f;
    PARAM float D5_MIN  = 1.5f;

    PARAM float MAX_ACCELERATION = 3.0f;  // m/s^2
    PARAM float MAX_DECELERATION = 4.0f;  // m/s^2

    PARAM float DIST_ERROR_MAX = 1.0f;   // m
    PARAM float ANG_ERROR_MAX  = 90.0f;  // deg

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED         = 1.0f;   // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = 0.5f;   // m/s
    PARAM float FAST_SPEED              = 4.5f;   // m/s
    PARAM float FAST_SPEED_TURN         = 1.5f;   // m/s
    PARAM float FAST_SPEED_OVERTAKE     = 1.0f;   // m/s
    PARAM float FAST_SPEED_SAFETY_CAR   = 1.0f;   // m/s
    PARAM float MIN_SPEED               = 0.25f;  // m/s

    ///////////////////////////////////////////////////////////////////////////
    //
    //      SIGNALS
    //

    PARAM int         RECEIVER_PORT    = 9750;
    PARAM const char *RECEIVER_ADDRESS = "0.0.0.0";

#else

    ///////////////////////////////////////////////////////////////////////////
    //
    //      LOGIC
    //

    /* STATIC PARAMETERS OF THE TRACK */
    PARAM float    SQUARE_LENGTH = 0.6;  // m
    PARAM unsigned BITMAP_SIZE   = 64;   // px

    /* AS STATE MACHINE*/
    PARAM float STATE_TRANSITION_TIME_LIMIT = 0.0f;  // s
    PARAM float STATE_MIN_TIME              = 0.1f;  // s
    PARAM float LOCALIZATION_INACCURACY     = 0.1f;  // m

    ///////////////////////////////////////////////////////////////////////////
    //
    //      ODOMETRY
    //

    /* STATIC PARAMETERS OF THE VEHICLE */
    PARAM float WHEEL_DIAMETER  = px_to_m(1.0f);          // m
    PARAM int   SENSOR_COUNT    = 16;                     // -
    PARAM float SENSOR_BASE     = px_to_m(16);            // m
    PARAM float SENSOR_WIDTH    = px_to_m(SENSOR_COUNT);  // m
    PARAM float MAX_WHEEL_ANGLE = 30.0f;                  // deg
    PARAM float WHEELBASE       = px_to_m(16.0f);         // m

    /* DYNAMIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_VELOCITY = 10.0f;        // m/s
    PARAM float MAX_YAW_RATE = 1.5f * M_PI;  // rad/s

    /* GEAR RATIOS */
    PARAM int   MAX_MOTOR_RPM             = 10000;
    PARAM float GEAR_RATIO_MOTOR_TO_WHEEL = static_cast<float>(3 / 2) * 1.0f;

    /* ALGORITHM PARAMETERS */
    PARAM int VELOCITY_BUFFER_SIZE = 10;
    PARAM int IMU_BUFFER_SIZE      = 10;

    ///////////////////////////////////////////////////////////////////////////
    //
    //      CONTROLLER
    //

    /* OBJECT PID CONTROLLER PARAMETERS */
    namespace obj
    {
        PARAM float kP                      = 4.20f;
        PARAM float kI                      = 0.69f;
        PARAM float kD                      = 0.0f;
        PARAM float TAU                     = 0.05f;
        PARAM float T                       = 0.005f;
        PARAM float LIM_MIN                 = 0.0f;
        PARAM float LIM_MAX                 = 1.0f;
        PARAM float DEADBAND                = 0.05f;
        PARAM float DERIVATIVE_FILTER_ALPHA = 0.1f;
        PARAM float FOLLOW_DISTANCE         = 0.3f;
    }  // namespace obj

    PARAM float OFFSET  = 0.4f;
    PARAM float SLOPE   = 0.5f;
    PARAM float DAMPING = 0.5f;

    PARAM float MAX_ACCELERATION = 1.0f;  // m/s^2
    PARAM float MAX_DECELERATION = 1.0f;  // m/s^2

    PARAM float DIST_ERROR_MAX = 1.0f;   // m
    PARAM float ANG_ERROR_MAX  = 90.0f;  // deg

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED         = 0.4f;   // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = 0.2f;   // m/s
    PARAM float FAST_SPEED              = 1.5f;   // m/s
    PARAM float FAST_SPEED_TURN         = 0.75f;  // m/s
    PARAM float FAST_SPEED_OVERTAKE     = 1.0f;   // m/s
    PARAM float FAST_SPEED_SAFETY_CAR   = 0.6f;   // m/s
    PARAM float MIN_SPEED               = 0.1f;   // m/s

    ///////////////////////////////////////////////////////////////////////////
    //
    //      SIGNALS
    //

    PARAM int         RECEIVER_PORT    = 9750;
    PARAM const char *RECEIVER_ADDRESS = "0.0.0.0";
    PARAM int         SENDER_PORT      = 9750;
    PARAM const char *SENDER_ADDRESS   = "255.255.255.255";

#endif

}  // namespace jlb

#endif  // COMMON_HXX
