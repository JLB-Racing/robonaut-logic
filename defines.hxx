#ifndef DEFINES_HXX
#define DEFINES_HXX

///////////////////////////////////////////////////////////////////////////
//
//      LOGIC DEFINES
//

// #define Q2
#define SIMULATION
// #define OVERTAKE
// #define TEST_REVERSE
// #define TEST_MISSION_SWITCH
// #define FAST_V0

///////////////////////////////////////////////////////////////////////////
//
//      OTHER DEFINES
//

#define PARAM static constexpr

#ifndef px_to_m
#define px_to_m(px) (px * (jlb::SQUARE_LENGTH * 2.0f) / jlb::BITMAP_SIZE)
#endif
#ifndef m_to_px
#define m_to_px(m) (m * jlb::BITMAP_SIZE / (jlb::SQUARE_LENGTH * 2.0f))
#endif

#ifndef rad2deg
#define rad2deg(rad) (rad * 180.0f / static_cast<float>(M_PI))
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * static_cast<float>(M_PI) / 180.0f)
#endif

#endif  // DEFINES_HXX
