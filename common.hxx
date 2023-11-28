#ifndef COMMON_HXX
#define COMMON_HXX

#include <cmath>

///////////////////////////////////////////////////////////////////////////
//
//      DEFINES
//

// #define SIMULATION
// #define STM32

#define PARAM static constexpr

#ifndef px_to_m
#define px_to_m(px) (px * jlb::SQUARE_LENGTH / jlb::BITMAP_SIZE)
#endif
#ifndef m_to_px
#define m_to_px(m) (m * jlb::BITMAP_SIZE / jlb::SQUARE_LENGTH)
#endif
//
//      END DEFINES
//
///////////////////////////////////////////////////////////////////////////

namespace jlb
{
    ///////////////////////////////////////////////////////////////////////////
    //
    //      ENUMS
    //

    enum class Direction
    {
        LEFT,
        RIGHT,
        STRAIGHT,
        REVERSE_LEFT,
        REVERSE_RIGHT,
        REVERSE_STRAIGHT
    };

    enum class Mission
    {
        LABYRINTH,
        FAST,
        FAST_TURN,
        FAST_OVERTAKE
    };

    //
    //      END ENUMS
    //
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //
    //      LOGIC
    //

    /* STATIC PARAMETERS OF THE TRACK */
    PARAM float SQUARE_LENGTH = 0.6; // m
    PARAM unsigned BITMAP_SIZE = 64; // px

    //
    //      END LOGIC
    //
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //
    //      ODOMETRY
    //

#ifndef SIMULATION
    /* STATIC PARAMETERS OF THE VEHICLE */
    [[maybe_unused]] PARAM float WHEEL_RADIUS_UNLOADED = 0.0525f; // m
    PARAM float WHEEL_DIAMETER = 2.0f * WHEEL_RADIUS_UNLOADED;    // m
    [[maybe_unused]] PARAM float WHEEL_WIDTH = 0.05f;             // m
    [[maybe_unused]] PARAM float TRACK = 0.26f;                   // m
    [[maybe_unused]] PARAM float WHEELBASE = 0.275f;              // m
    PARAM int SENSOR_COUNT = 32;                                  // -
    PARAM float SENSOR_BASE = 0.5f;                               // m
    PARAM float SENSOR_WIDTH = 0.2f;                              // m

    /* DYNAMIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_VELOCITY = 12.5f;       // m/s
    PARAM float MAX_YAW_RATE = 1.5f * M_PI; // rad/s

    /* GEAR RATIOS */
    PARAM int MAX_MOTOR_RPM = 10000;
    PARAM int SPUR_GEAR_TOOTH_COUNT = 48;
    PARAM int PINION_GEAR_TOOTH_COUNT = 13;
    PARAM float INTERNAL_GEAR_RATIO = 1.0f;
    PARAM float GEAR_RATIO_MOTOR_TO_WHEEL = static_cast<float>(SPUR_GEAR_TOOTH_COUNT) / static_cast<float>(PINION_GEAR_TOOTH_COUNT) * INTERNAL_GEAR_RATIO;

    /* ALGORITHM PARAMETERS */
    PARAM int VELOCITY_BUFFER_SIZE = 3;
    PARAM int IMU_BUFFER_SIZE = 3;
#else
    /* STATIC PARAMETERS OF THE VEHICLE */
    PARAM float WHEEL_DIAMETER = px_to_m(1.0f);       // m
    PARAM float WHEELBASE = px_to_m(16.0f);           // m
    PARAM int SENSOR_COUNT = 16;                      // -
    PARAM float SENSOR_BASE = px_to_m(16);            // m
    PARAM float SENSOR_WIDTH = px_to_m(SENSOR_COUNT); // m

    /* DYNAMIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_VELOCITY = px_to_m(500.0f); // m/s
    PARAM float MAX_YAW_RATE = 1.5f * M_PI;     // rad/s

    /* GEAR RATIOS */
    PARAM int MAX_MOTOR_RPM = 10000;
    PARAM float GEAR_RATIO_MOTOR_TO_WHEEL = static_cast<float>(3 / 2) * 1.0f;

    /* ALGORITHM PARAMETERS */
    PARAM int VELOCITY_BUFFER_SIZE = 10;
    PARAM int IMU_BUFFER_SIZE = 10;
#endif

    //
    //      END ODOMETRY
    //
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //
    //      CONTROLLER
    //

#ifndef SIMULATION
    /* STATIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_WHEEL_ANGLE = 1.0f; // rad

    /* PID CONTROLLER PARAMETERS */
    PARAM float kP = 1.0f;
    PARAM float kI = 0.0f;
    PARAM float kD = 0.0f;
    PARAM float TAU = 0.05f;
    PARAM float T = 0.005f;
    PARAM float LIM_MIN = 0.0f;
    PARAM float LIM_MAX = 1.0f;
    PARAM float FOLLOW_DISTANCE = 0.25f;

    /* STANLEY CONTROLLER PARAMETERS */
    PARAM float kAng = 0.5f;
    PARAM float kDist = 0.5f;
    PARAM float kSoft = 1.0f;
    PARAM float kDamp = 1.0f;

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED = 10.0f;        // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = 5.0f; // m/s
    PARAM float FAST_SPEED = 30.0f;             // m/s
    PARAM float FAST_SPEED_TURN = 10.0f;        // m/s
    PARAM float FAST_SPEED_OVERTAKE = 20.0f;    // m/s
#else
    /* STATIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_WHEEL_ANGLE = 1.0f; // rad

    /* PID CONTROLLER PARAMETERS */
    PARAM float kP = 5.0f;
    PARAM float kI = 0.0f;
    PARAM float kD = 0.0f;
    PARAM float TAU = 0.05f;
    PARAM float T = 0.005f;
    PARAM float LIM_MIN = 0.0f;
    PARAM float LIM_MAX = 1.0f;
    PARAM float FOLLOW_DISTANCE = 0.35f;

    /* STANLEY CONTROLLER PARAMETERS */
    PARAM float kAng = 0.75f;
    PARAM float kDist = 15.0f;
    PARAM float kSoft = 1.0f;
    PARAM float kDamp = 0.0f;

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED = px_to_m(40.0f);         // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = px_to_m(20.0f); // m/s
    PARAM float FAST_SPEED = px_to_m(80.0f);              // m/s
    PARAM float FAST_SPEED_TURN = px_to_m(40.0f);         // m/s
    PARAM float FAST_SPEED_OVERTAKE = px_to_m(60.0f);     // m/s
#endif

    //
    //      END CONTROLLER
    //
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //
    //      SIGNALS
    //

#ifndef SIMULATION
    PARAM int SERVER_PORT = 8998;
    PARAM const char *SERVER_ADDRESS = "localhost";
#else
    PARAM int SERVER_PORT = 8998;
    PARAM const char *SERVER_ADDRESS = "localhost";
#endif

    //
    //      END SIGNALS
    //
    ///////////////////////////////////////////////////////////////////////////

} // namespace jlb

#endif // COMMON_HXX
