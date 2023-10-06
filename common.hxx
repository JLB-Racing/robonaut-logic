#ifndef COMMON_HXX
#define COMMON_HXX

#include <cmath>

///////////////////////////////////////////////////////////////////////////
//
//      DEFINES
//

#define SIMULATION
// #define STM32

#define PARAM static constexpr

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
    //      ODOMETRY
    //

#ifndef SIMULATION
    /* STATIC PARAMETERS OF THE VEHICLE */
    [[maybe_unused]] PARAM float WHEEL_RADIUS_UNLOADED = 0.0525f; // m
    PARAM float WHEEL_DIAMETER = 2.0f * WHEEL_RADIUS_UNLOADED;    // m
    [[maybe_unused]] PARAM float WHEEL_WIDTH = 0.05f;             // m
    [[maybe_unused]] PARAM float TRACK = 0.26f;                   // m
    [[maybe_unused]] PARAM float WHEELBASE = 0.275f;              // m
    PARAM int SENSOR_WIDTH = 32;                                  // -

    /* DYNAMIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_VELOCITY = 12.5f;       // m/s
    PARAM float MAX_YAW_RATE = 1.5f * M_PI; // rad/s

    /* GEAR RATIOS */
    PARAM int SPUR_GEAR_TOOTH_COUNT = 48;
    PARAM int PINION_GEAR_TOOTH_COUNT = 13;
    PARAM float INTERNAL_GEAR_RATIO = 1.0f;
    PARAM float GEAR_RATIO_MOTOR_TO_WHEEL = static_cast<float>(SPUR_GEAR_TOOTH_COUNT) / static_cast<float>(PINION_GEAR_TOOTH_COUNT) * INTERNAL_GEAR_RATIO;

    /* ALGORITHM PARAMETERS */
    PARAM int VELOCITY_BUFFER_SIZE = 3;
    PARAM int IMU_BUFFER_SIZE = 3;
#else
    /* STATIC PARAMETERS OF THE VEHICLE */
    PARAM float WHEEL_DIAMETER = 1.0f; // px
    PARAM float WHEELBASE = 16.0f;     // px
    PARAM int SENSOR_WIDTH = 16;       // -

    /* DYNAMIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_VELOCITY = 500.0f;      // px/s
    PARAM float MAX_YAW_RATE = 1.5f * M_PI; // rad/s

    /* GEAR RATIOS */
    PARAM float GEAR_RATIO_MOTOR_TO_WHEEL = static_cast<float>(3 / 2) * 1.0f;

    /* ALGORITHM PARAMETERS */
    PARAM int VELOCITY_BUFFER_SIZE = 1;
    PARAM int IMU_BUFFER_SIZE = 1;
#endif

    //
    //      END ODOMETRY
    //
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //
    //      LOGIC
    //

#ifndef SIMULATION
    /* STATIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_WHEEL_ANGLE = 1.0f; // rad

    /* LONGITUDINAL CONTROLLER PARAMETERS */
    PARAM float Kp = 0.6f;
    PARAM float Ki = 0.01f;
    PARAM float Kd = 0.4f;

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED = 10.0f;        // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = 5.0f; // m/s
    PARAM float FAST_SPEED = 30.0f;             // m/s
    PARAM float FAST_SPEED_TURN = 10.0f;        // m/s
    PARAM float FAST_SPEED_OVERTAKE = 20.0f;    // m/s
#else
    /* STATIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_WHEEL_ANGLE = 1.0f; // rad

    /* LONGITUDINAL CONTROLLER PARAMETERS */
    PARAM float Kp = 0.6f;
    PARAM float Ki = 0.001f;
    PARAM float Kd = 0.1f;

    /* LATERAL CONTROLLER PARAMETERS */
    PARAM float LABYRINTH_SPEED = 40.0f;         // px/s
    PARAM float LABYRINTH_SPEED_REVERSE = 20.0f; // px/s
    PARAM float FAST_SPEED = 80.0f;              // px/s
    PARAM float FAST_SPEED_TURN = 40.0f;         // px/s
    PARAM float FAST_SPEED_OVERTAKE = 60.0f;     // px/s
#endif

    //
    //      END LOGIC
    //
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //
    //      SIGNALS
    //

    PARAM int SERVER_PORT = 5000;
    PARAM const char *SERVER_ADDRESS = "localhost";
    PARAM unsigned long SIGNALS_SIZE = 2;

    PARAM uint8_t TARGET_ANGLE_ID = 0;
    PARAM uint8_t TARGET_SPEED_ID = 1;

    //
    //      END SIGNALS
    //
    ///////////////////////////////////////////////////////////////////////////

} // namespace jlb

#endif // COMMON_HXX