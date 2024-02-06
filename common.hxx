#ifndef COMMON_HXX
#define COMMON_HXX

#include "defines.hxx"
#include "types.hxx"
#include "map_common.hxx"

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
    PARAM float LOCALIZATION_INACCURACY     = 0.1f;  // m
    PARAM float LOCALIZATION_FALLBACK       = 0.0f;  // m
    PARAM float WEIGHT_PENALTY              = 1000.0f;
    PARAM float SAFETY_MARGIN               = 1.0f;
    PARAM float FAST_DISCOUNT               = 0.6f;

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
        PARAM float kP                      = 4.275f;  // 5.12f;
        PARAM float kI                      = 0.0f;    // 0.25f;
        PARAM float kD                      = 4.75f;   // 4.0f;
        PARAM float TAU                     = 0.05f;
        PARAM float T                       = 0.005f;
        PARAM float LIM_MIN                 = -MAX_WHEEL_ANGLE;
        PARAM float LIM_MAX                 = MAX_WHEEL_ANGLE;
        PARAM float DEADBAND                = 0.0f;
        PARAM float DERIVATIVE_FILTER_ALPHA = 0.0f;
    }  // namespace lat

    PARAM float OFFSET             = -4.625f;
    PARAM float SLOPE              = 3.5f;
    PARAM float DAMPING            = 0.95f;
    PARAM float DAMPING_TURN       = 0.85f;
    PARAM float D5_MIN             = 1.50f;
    PARAM float OFFSET_EXP1        = 4.4f;
    PARAM float OFFSET_EXP2        = -1.38f;
    PARAM float D5_REVERSE         = 0.02f;
    PARAM float DAMPING_REVERSE    = 0.75f;
    PARAM float MULTIPLIER_REVERSE = 1.55f;

    PARAM float MAX_ACCELERATION = 7.5f;  // m/s^2
    PARAM float MAX_DECELERATION = 6.5f;  // m/s^2

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED         = 1.0f;   // m/s
    PARAM float LABYRINTH_SPEED_FAST    = 1.5f;   // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = 0.75f;  // m/s
    PARAM float BALANCER_SPEED          = 1.5f;   // m/s
    PARAM float MISSION_SWITCH_SPEED    = 0.75f;  // m/s

#ifdef FAST_V0
    PARAM float FAST_SPEED      = 5.0f;  // m/s
    PARAM float FAST_SPEED_TURN = 1.2f;  // m/s
#else
    PARAM float FAST_SPEED[6]      = {5.0f, 5.0f, 5.0f, 5.5f, 6.0f, 6.5f};  // m/s
    PARAM float FAST_SPEED_TURN[6] = {1.0f, 1.0f, 1.0f, 1.1f, 1.2f, 1.3f};  // m/s
#endif
    PARAM float FAST_SPEED_OVERTAKE      = 2.0f;   // m/s
    PARAM float FAST_SPEED_OVERTAKE_TURN = 1.5f;   // m/s
    PARAM float FAST_SPEED_SAFETY_CAR    = 1.5f;   // m/s
    PARAM float LOW_SPEED_EPSILON        = 0.20f;  // m/s

    PARAM float SAFETY_CAR_THRESHOLD         = 1.50f;  // m
    PARAM float SAFETY_CAR_TIMEOUT           = 2.0f;   // s
    PARAM float CROSS_SECTION_THRESHOLD      = 0.20f;  // %/100
    PARAM float OVERTAKE_FIRST_FORWARD_TIME  = 2.0f;   // s
    PARAM float OVERTAKE_FIRST_LEFT_TIME     = 1.0f;   // s
    PARAM float OVERTAKE_FIRST_RIGHT_TIME    = 1.0f;   // s
    PARAM float OVERTAKE_SECOND_FORWARD_TIME = 2.0f;   // s
    PARAM float OVERTAKE_SECOND_LEFT_TIME    = 1.0f;   // s
    PARAM float OVERTAKE_SECOND_RIGHT_TIME   = 1.0f;   // s
    PARAM float OVERTAKE_STEERING_ANGLE      = 12.5f;  // deg

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
    PARAM float LOCALIZATION_FALLBACK       = 0.0f;  // m
    PARAM float WEIGHT_PENALTY              = 1000.0f;
    PARAM float SAFETY_MARGIN               = 1.0f;
    PARAM float FAST_DISCOUNT               = 0.6f;

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
        PARAM float kP                      = 100.0f;
        PARAM float kI                      = 0.0f;
        PARAM float kD                      = 0.0f;
        PARAM float TAU                     = 0.05f;
        PARAM float T                       = 0.005f;
        PARAM float LIM_MIN                 = 0.0f;
        PARAM float LIM_MAX                 = 1.0f;
        PARAM float DEADBAND                = 0.0f;
        PARAM float DERIVATIVE_FILTER_ALPHA = 0.0f;
        PARAM float FOLLOW_DISTANCE         = 0.4f;
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

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED         = 2.0f;  // m/s
    PARAM float LABYRINTH_SPEED_FAST    = 2.0f;  // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = 1.0f;  // m/s
    PARAM float BALANCER_SPEED          = 1.5f;  // m/s
    PARAM float MISSION_SWITCH_SPEED    = 0.5f;  // m/s
#ifdef FAST_V0
    PARAM float FAST_SPEED      = 3.5f;  // m/s
    PARAM float FAST_SPEED_TURN = 3.0f;  // m/s
#else
    PARAM float FAST_SPEED[6]      = {6.5f, 6.5f, 6.5f, 6.75f, 7.0f, 7.25f};  // m/s
    PARAM float FAST_SPEED_TURN[6] = {3.0f, 3.0f, 3.0f, 3.25f, 3.5f, 3.75f};  // m/s
#endif
    PARAM float FAST_SPEED_OVERTAKE      = 4.0f;  // m/s
    PARAM float FAST_SPEED_OVERTAKE_TURN = 2.5f;  // m/s
    PARAM float FAST_SPEED_SAFETY_CAR    = 2.5f;  // m/s

    PARAM float SAFETY_CAR_THRESHOLD         = 1.50f;  // m
    PARAM float SAFETY_CAR_TIMEOUT           = 2.0f;   // s
    PARAM float OVERTAKE_FIRST_FORWARD_TIME  = 0.5f;   // s
    PARAM float OVERTAKE_FIRST_LEFT_TIME     = 0.35f;  // s
    PARAM float OVERTAKE_FIRST_RIGHT_TIME    = 0.35f;  // s
    PARAM float OVERTAKE_SECOND_FORWARD_TIME = 0.75f;  // s
    PARAM float OVERTAKE_SECOND_LEFT_TIME    = 0.35f;  // s
    PARAM float OVERTAKE_SECOND_RIGHT_TIME   = 0.35f;  // s
    PARAM float OVERTAKE_STEERING_ANGLE      = 12.5f;  // deg

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
            case FastState::FIRST_FAST:
                os << "FIRST_FAST";
                break;
            case FastState::FIRST_SLOW:
                os << "FIRST_SLOW";
                break;
            case FastState::SECOND_FAST:
                os << "SECOND_FAST";
                break;
            case FastState::SECOND_SLOW:
                os << "SECOND_SLOW";
                break;
            case FastState::THIRD_FAST:
                os << "THIRD_FAST";
                break;
            case FastState::THIRD_SLOW:
                os << "THIRD_SLOW";
                break;
            case FastState::FOURTH_FAST:
                os << "FOURTH_FAST";
                break;
            case FastState::FOURTH_SLOW:
                os << "FOURTH_SLOW";
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
