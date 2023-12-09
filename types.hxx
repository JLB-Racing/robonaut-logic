#ifndef TYPES_HXX
#define TYPES_HXX

#include <cmath>

#define PARAM static constexpr

#ifndef px_to_m
#define px_to_m(px) (px * jlb::SQUARE_LENGTH / jlb::BITMAP_SIZE)
#endif
#ifndef m_to_px
#define m_to_px(m) (m * jlb::BITMAP_SIZE / jlb::SQUARE_LENGTH)
#endif

#ifndef rad2deg
#define rad2deg(rad) (rad * 180.0f / static_cast<float>(M_PI))
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * static_cast<float>(M_PI) / 180.0f)
#endif

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
    };

    enum class Mission
    {
        LABYRINTH,
        FAST,
    };

    enum class LabyrinthState
    {
        // TODO: this is just placeholder
        START,
        TURN_LEFT,
        TURN_RIGHT,
        STRAIGHT,
        END,
    };

    enum class FastState
    {
        FOLLOW_SAFETY_CAR,
        OVERTAKE_SAFETY_CAR_START,
        OVERTAKE_SAFETY_CAR_END,
        IN_ACCEL_ZONE,
        OUT_ACCEL_ZONE,
        IN_BRAKE_ZONE,
        OUT_BRAKE_ZONE,
    };

    //
    //      END ENUMS
    //
    ///////////////////////////////////////////////////////////////////////////

}  // namespace jlb

#endif  // TYPES_HXX///////////////////////////////////////////////////////////////////////////
        //
//      LOGIC
//

/* STATIC PARAMETERS OF THE TRACK */
PARAM float    SQUARE_LENGTH = 0.6;  // m
PARAM unsigned BITMAP_SIZE   = 64;   // px

//
//      END LOGIC
//
///////////////////////////////////////////////////////////////////////////