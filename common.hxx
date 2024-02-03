#ifndef COMMON_HXX
#define COMMON_HXX

#include "types.hxx"
#include "map_common.hxx"

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

    /* AS STATE MACHINE*/
    PARAM float STATE_TRANSITION_TIME_LIMIT = 0.0f;
    PARAM float STATE_MIN_TIME              = 0.5f;
    PARAM float LOCALIZATION_INACCURACY     = 0.2f;  // m
    PARAM float WEIGHT_PENALTY              = 1000.0f;
    PARAM float SAFETY_MARGIN               = 1.0f;

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
        PARAM float kP                      = 4.0f;
        PARAM float kI                      = 1.69f;
        PARAM float kD                      = 0.0f;
        PARAM float TAU                     = 0.01f;
        PARAM float T                       = 0.01f;
        PARAM float LIM_MIN                 = 0.0f;
        PARAM float LIM_MAX                 = 1.0f;
        PARAM float DEADBAND                = 0.1f;
        PARAM float DERIVATIVE_FILTER_ALPHA = 0.0f;
        PARAM float FOLLOW_DISTANCE         = 0.40f;
    }  // namespace obj

    /* LATERAL PID CONTROLLER PARAMETERS */
    namespace lat
    {
        PARAM float kP                      = 5.12f;
        PARAM float kI                      = 0.25f;
        PARAM float kD                      = 4.0f;
        PARAM float TAU                     = 0.05f;
        PARAM float T                       = 0.005f;
        PARAM float LIM_MIN                 = -MAX_WHEEL_ANGLE;
        PARAM float LIM_MAX                 = MAX_WHEEL_ANGLE;
        PARAM float DEADBAND                = 0.5f;
        PARAM float DERIVATIVE_FILTER_ALPHA = 0.0f;
    }  // namespace lat

    PARAM float OFFSET             = -4.625f;
    PARAM float SLOPE              = 3.5f;
    PARAM float DAMPING            = 0.95f;
    PARAM float DAMPING_TURN       = 0.85f;
    PARAM float D5_MIN             = 1.50f;
    PARAM float OFFSET_EXP1        = 4.4f;
    PARAM float OFFSET_EXP2        = -1.38f;
    PARAM float D5_REVERSE         = 0.15f;
    PARAM float DAMPING_REVERSE    = 0.85f;
    PARAM float MULTIPLIER_REVERSE = 1.0f;

    PARAM float MAX_ACCELERATION = 7.5f;  // m/s^2
    PARAM float MAX_DECELERATION = 6.5f;  // m/s^2

    PARAM float DIST_ERROR_MAX = 1.0f;   // m
    PARAM float ANG_ERROR_MAX  = 90.0f;  // deg

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED         = 1.20f;  // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = 0.5f;   // m/s
    PARAM float BALANCER_SPEED          = 0.5f;   // m/s
    PARAM float MISSION_SWITCH_SPEED    = 0.5f;   // m/s
    // PARAM float FAST_SPEED              = 5.0f;  // m/s
    // PARAM float FAST_SPEED_TURN         = 1.20f;  // m/s
    PARAM float FAST_SPEED            = 1.5f;   // m/s
    PARAM float FAST_SPEED_TURN       = 1.5f;   // m/s
    PARAM float FAST_SPEED_OVERTAKE   = 1.0f;   // m/s
    PARAM float FAST_SPEED_SAFETY_CAR = 1.0f;   // m/s
    PARAM float LOW_SPEED_EPSILON     = 0.20f;  // m/s

    PARAM float SPEED_SAFETY_CAR_FOLLOW = 1.3f;   // m/s
    PARAM float SAFETY_CAR_THRESHOLD    = 1.50f;  // m

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

    /* AS STATE MACHINE*/
    PARAM float STATE_TRANSITION_TIME_LIMIT = 0.0f;
    PARAM float STATE_MIN_TIME              = 0.25f;
    PARAM float LOCALIZATION_INACCURACY     = 0.2f;  // m
    PARAM float WEIGHT_PENALTY              = 1000.0f;
    PARAM float SAFETY_MARGIN               = 1.0f;

    ///////////////////////////////////////////////////////////////////////////
    //
    //      ODOMETRY
    //

    /* STATIC PARAMETERS OF THE VEHICLE */
    PARAM float WHEEL_DIAMETER  = px_to_m(1.0f);          // m
    PARAM int   SENSOR_COUNT    = 24;                     // -
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

    namespace lat
    {
        PARAM float kP                      = 5.12f;
        PARAM float kI                      = 0.25f;
        PARAM float kD                      = 4.0f;
        PARAM float TAU                     = 0.05f;
        PARAM float T                       = 0.005f;
        PARAM float LIM_MIN                 = -MAX_WHEEL_ANGLE;
        PARAM float LIM_MAX                 = MAX_WHEEL_ANGLE;
        PARAM float DEADBAND                = 0.5f;
        PARAM float DERIVATIVE_FILTER_ALPHA = 0.0f;
    }  // namespace lat

    PARAM float OFFSET             = 0.0f;
    PARAM float SLOPE              = 0.001f;
    PARAM float DAMPING            = 0.001f;
    PARAM float D5_MIN             = 0.0f;
    PARAM float OFFSET_EXP1        = 4.5f;
    PARAM float OFFSET_EXP2        = -1.38f;
    PARAM float D5_REVERSE         = 0.15f;
    PARAM float DAMPING_REVERSE    = 0.85f;
    PARAM float MULTIPLIER_REVERSE = 1.0f;

    PARAM float MAX_ACCELERATION = 5.0f;  // m/s^2
    PARAM float MAX_DECELERATION = 7.5f;  // m/s^2

    PARAM float DIST_ERROR_MAX = 1.0f;   // m
    PARAM float ANG_ERROR_MAX  = 90.0f;  // deg

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED         = 2.0f;   // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = 1.0f;   // m/s
    PARAM float BALANCER_SPEED          = 1.0f;   // m/s
    PARAM float MISSION_SWITCH_SPEED    = 0.5f;   // m/s
    PARAM float FAST_SPEED              = 1.5f;   // m/s
    PARAM float FAST_SPEED_TURN         = 0.75f;  // m/s
    PARAM float FAST_SPEED_OVERTAKE     = 1.0f;   // m/s
    PARAM float FAST_SPEED_SAFETY_CAR   = 0.6f;   // m/s

    PARAM float SPEED_SAFETY_CAR_FOLLOW = 1.3f;   // m/s
    PARAM float SAFETY_CAR_THRESHOLD    = 1.50f;  // m

    ///////////////////////////////////////////////////////////////////////////
    //
    //      SIGNALS
    //

    PARAM int         RECEIVER_PORT    = 9750;
    PARAM const char *RECEIVER_ADDRESS = "0.0.0.0";
    PARAM int         SENDER_PORT      = 9750;
    PARAM const char *SENDER_ADDRESS   = "255.255.255.255";

    std::ostream &operator<<(std::ostream &os, const Direction &direction)
    {
        switch (direction)
        {
            case Direction::LEFT:
                os << "LEFT";
                break;
            case Direction::RIGHT:
                os << "RIGHT";
                break;
            case Direction::STRAIGHT:
                os << "STRAIGHT";
                break;
            default:
                os << "UNKNOWN";
                break;
        }
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const Mission &mission)
    {
        switch (mission)
        {
            case Mission::STANDBY:
                os << "STANDBY";
                break;
            case Mission::LABYRINTH:
                os << "LABYRINTH";
                break;
            case Mission::FAST:
                os << "FAST";
                break;
            default:
                os << "UNKNOWN";
                break;
        }
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const LabyrinthState &state)
    {
        switch (state)
        {
            case LabyrinthState::START:
                os << "START";
                break;
            case LabyrinthState::EXPLORING:
                os << "EXPLORING";
                break;
            case LabyrinthState::STANDBY:
                os << "STANDBY";
                break;
            case LabyrinthState::ESCAPE:
                os << "ESCAPE";
                break;
            case LabyrinthState::REVERSE_ESCAPE:
                os << "REVERSE_ESCAPE";
                break;
            case LabyrinthState::FLOOD_TO_BALANCER:
                os << "FLOOD_TO_BALANCER";
                break;
            case LabyrinthState::FLOOD_SOLVING:
                os << "FLOOD_SOLVING";
                break;
            case LabyrinthState::FLOOD_TO_LABYRINTH:
                os << "FLOOD_TO_LABYRINTH";
                break;
            case LabyrinthState::FINISHED:
                os << "FINISHED";
                break;
            case LabyrinthState::MISSION_SWITCH:
                os << "MISSION_SWITCH";
                break;
            case LabyrinthState::ERROR:
                os << "ERROR";
                break;
            default:
                os << "UNKNOWN";
                break;
        }
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const MissionSwitchState &state)
    {
        switch (state)
        {
            case MissionSwitchState::STANDBY:
                os << "STANDBY";
                break;
            case MissionSwitchState::FIRST_FORWARD:
                os << "FIRST_FORWARD";
                break;
            case MissionSwitchState::FIRST_TURN:
                os << "FIRST_TURN";
                break;
            case MissionSwitchState::SECOND_TURN:
                os << "SECOND_TURN";
                break;
            case MissionSwitchState::SECOND_FORWARD:
                os << "SECOND_FORWARD";
                break;
            default:
                os << "UNKNOWN";
                break;
        }
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const FastState &state)
    {
        switch (state)
        {
            case FastState::FOLLOW_SAFETY_CAR:
                os << "FOLLOW_SAFETY_CAR";
                break;
            case FastState::OVERTAKE_SAFETY_CAR_START:
                os << "OVERTAKE_SAFETY_CAR_START";
                break;
            case FastState::OVERTAKE_SAFETY_CAR_END:
                os << "OVERTAKE_SAFETY_CAR_END";
                break;
            case FastState::IN_ACCEL_ZONE:
                os << "IN_ACCEL_ZONE";
                break;
            case FastState::OUT_ACCEL_ZONE:
                os << "OUT_ACCEL_ZONE";
                break;
            case FastState::IN_BRAKE_ZONE:
                os << "IN_BRAKE_ZONE";
                break;
            case FastState::OUT_BRAKE_ZONE:
                os << "OUT_BRAKE_ZONE";
                break;
            default:
                os << "UNKNOWN";
                break;
        }
        return os;
    }

#endif

}  // namespace jlb

#endif  // COMMON_HXX
