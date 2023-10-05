#ifndef COMMON_HXX
#define COMMON_HXX

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
    //      ODOMETRY
    //

#ifndef SIMULATION
    /* STATIC PARAMETERS OF THE VEHICLE */
    [[maybe_unused]] PARAM float WHEEL_RADIUS_UNLOADED = 0.0525f; // m
    PARAM float WHEEL_DIAMETER = 2.0f * WHEEL_RADIUS_UNLOADED;    // m
    [[maybe_unused]] PARAM float WHEEL_WIDTH = 0.05f;             // m
    [[maybe_unused]] PARAM float TRACK = 0.26f;                   // m
    [[maybe_unused]] PARAM float WHEELBASE = 0.275f;              // m

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
    PARAM float WHEEL_DIAMETER = 1.0f;        // px
    static constexpr float WHEELBASE = 16.0f; // px

    /* DYNAMIC PARAMETERS OF THE VEHICLE */
    PARAM float MAX_VELOCITY = 50.0f;       // px/s
    PARAM float MAX_YAW_RATE = 1.5f * M_PI; // rad/s

    /* GEAR RATIOS */
    PARAM float GEAR_RATIO_MOTOR_TO_WHEEL = static_cast<float>(3 / 2) * 1.0f;

    /* ALGORITHM PARAMETERS */
    PARAM int VELOCITY_BUFFER_SIZE = 5;
    PARAM int IMU_BUFFER_SIZE = 5;
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
    PARAM float Kp = 0.6f;
    PARAM float Ki = 0.01f;
    PARAM float Kd = 0.4f;

    PARAM float LABYRINTH_SPEED = 10.0f;        // m/s
    PARAM float LABYRINTH_SPEED_REVERSE = 5.0f; // m/s
    PARAM float FAST_SPEED = 30.0f;             // m/s
    PARAM float FAST_SPEED_TURN = 10.0f;        // m/s
    PARAM float FAST_SPEED_OVERTAKE = 20.0f;    // m/s
#else
    PARAM float Kp = 0.6f;
    PARAM float Ki = 0.01f;
    PARAM float Kd = 0.4f;

    PARAM float LABYRINTH_SPEED = 50.0f;         // px/s
    PARAM float LABYRINTH_SPEED_REVERSE = 25.0f; // px/s
    PARAM float FAST_SPEED = 300.0f;             // px/s
    PARAM float FAST_SPEED_TURN = 100.0f;        // px/s
    PARAM float FAST_SPEED_OVERTAKE = 200.0f;    // px/s
#endif

    //
    //      END LOGIC
    //
    ///////////////////////////////////////////////////////////////////////////

} // namespace jlb

#endif // COMMON_HXX