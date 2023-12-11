#ifndef COMMON_HXX
#define COMMON_HXX

#include "types.hxx"

///////////////////////////////////////////////////////////////////////////
//
//      DEFINES
//

//#define SIMULATION

namespace jlb
{
#ifndef SIMULATION

    ///////////////////////////////////////////////////////////////////////////
    //
    //      LOGIC
    //

    /* STATIC PARAMETERS OF THE TRACK */
    PARAM float    SQUARE_LENGTH = 0.6;  // m
    PARAM unsigned BITMAP_SIZE   = 64;   // px

    /* AS STATE MACHINE*/
    PARAM float STATE_TRANSITION_TIME_LIMIT = 0.0f;
    PARAM float STATE_MIN_TIME              = 0.25f;

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
    PARAM float MAX_WHEEL_ANGLE       = 21.5f;                         // deg
    PARAM float WHEEL_WIDTH           = 0.05f;                         // m
    PARAM float TRACK                 = 0.26f;                         // m
    PARAM float WHEELBASE             = 0.275f;                        // m

    /* DYNAMIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_VELOCITY = 12.5f;        // m/s
    PARAM float MAX_YAW_RATE = 1.5f * M_PI;  // rad/s

    /* GEAR RATIOS */
    PARAM int   MAX_MOTOR_RPM             = 10000;
    PARAM int   SPUR_GEAR_TOOTH_COUNT     = 48;
    PARAM int   PINION_GEAR_TOOTH_COUNT   = 13;
    PARAM float INTERNAL_GEAR_RATIO       = 1.0f;
    PARAM float GEAR_RATIO_MOTOR_TO_WHEEL = static_cast<float>(SPUR_GEAR_TOOTH_COUNT) / static_cast<float>(PINION_GEAR_TOOTH_COUNT) * INTERNAL_GEAR_RATIO;

    /* ALGORITHM PARAMETERS */
    PARAM int VELOCITY_BUFFER_SIZE = 1;
    PARAM int IMU_BUFFER_SIZE      = 10;

    ///////////////////////////////////////////////////////////////////////////
    //
    //      CONTROLLER
    //

    /* PID CONTROLLER PARAMETERS */
    PARAM float kP              = 4.20f;
    PARAM float kI              = 0.69f;
    PARAM float kD              = 0.0f;
    PARAM float TAU             = 0.05f;
    PARAM float T               = 0.005f;
    PARAM float LIM_MIN         = 0.0f;
    PARAM float LIM_MAX         = 1.0f;
    PARAM float DEADBAND        = 0.00f;
    PARAM float FOLLOW_DISTANCE = 0.60f;

    /* STANLEY CONTROLLER PARAMETERS - FAST_SPEED */
    PARAM float kANG_FS  = 0.15f;
    PARAM float kDIST_FS = 1.0f;
    PARAM float kSOFT_FS = 0.35f;
    PARAM float kDAMP_FS = 0.0025f;

    /* STANLEY CONTROLLER PARAMETERS - FAST_SPEED_TURN */
    PARAM float kANG_FST  = 1.5f;
    PARAM float kDIST_FST = 1.0f;
    PARAM float kSOFT_FST = 0.15f;
    PARAM float kDAMP_FST = 0.0f;

    PARAM float PARAM_TRANSITION_TIME_ACCEL = 0.1f; //s
    PARAM float PARAM_TRANSITION_TIME_DECEL = 0.5f; //s
    PARAM float DIST_ERROR_MAX = 1.0f;   // m
    PARAM float ANG_ERROR_MAX  = 90.0f;  // deg

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED         = 1.0f;   // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = 0.5f;   // m/s
    PARAM float FAST_SPEED              = 8.0f;   // m/s
    PARAM float FAST_SPEED_TURN         = 5.0f;  // m/s
    PARAM float FAST_SPEED_OVERTAKE     = 5.0f;   // m/s
    PARAM float FAST_SPEED_SAFETY_CAR   = 3.0f;   // m/s
    PARAM float MIN_SPEED               = 1.0f;   // m/s

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
    PARAM float STATE_TRANSITION_TIME_LIMIT = 0.05f;
    PARAM float STATE_MIN_TIME              = 0.25f;

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

    /* PID CONTROLLER PARAMETERS */
    PARAM float kP              = 4.20f;
    PARAM float kI              = 0.69f;
    PARAM float kD              = 0.0f;
    PARAM float TAU             = 0.05f;
    PARAM float T               = 0.005f;
    PARAM float LIM_MIN         = 0.0f;
    PARAM float LIM_MAX         = 1.0f;
    PARAM float DEADBAND        = 0.00f;
    PARAM float FOLLOW_DISTANCE = 0.3f;

    /* STANLEY CONTROLLER PARAMETERS */
    PARAM float kAng           = 0.75f;
    PARAM float kDist          = 10.0f;
    PARAM float kSoft          = 1.0f;
    PARAM float kDamp          = 0.0f;
    PARAM float DIST_ERROR_MAX = 1.0f;   // m
    PARAM float ANG_ERROR_MAX  = 90.0f;  // deg

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED         = 0.4f;  // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = 0.2f;  // m/s
    PARAM float FAST_SPEED              = 1.2f;  // m/s
    PARAM float FAST_SPEED_TURN         = 0.6f;  // m/s
    PARAM float FAST_SPEED_OVERTAKE     = 1.0f;  // m/s
    PARAM float FAST_SPEED_SAFETY_CAR   = 0.6f;  // m/s
    PARAM float MIN_SPEED               = 0.1f;  // m/s

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
