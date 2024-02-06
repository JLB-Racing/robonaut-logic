#ifndef MAP_COMMON_HXX
#define MAP_COMMON_HXX

#include "defines.hxx"
#include "types.hxx"

namespace jlb
{

#ifndef Q2
    /* STATIC PARAMETERS OF THE TRACK */
    PARAM float    SQUARE_LENGTH                                                              = 0.6f;                                           // m
    PARAM unsigned BITMAP_SIZE                                                                = 64;                                             // px
    PARAM char     MISSION_SWITCH_NODE                                                        = 'V';                                            // -
    PARAM char     MISSION_SWITCH_NEXT_NODE                                                   = 'W';                                            // -
    PARAM int      NUMBER_OF_MISSION_SWITCH_PROHIBITED_EDGES                                  = 3;                                              // -
    PARAM pcc      MISSION_SWITCH_PROHIBITED_EDGES[NUMBER_OF_MISSION_SWITCH_PROHIBITED_EDGES] = {pcc('S', 'V'), pcc('W', 'V'), pcc('R', 'Q')};  // -
    PARAM int      NUMBER_OF_MISSION_SWITCH_PREV_NODES                                        = 1;                                              // -
    PARAM char     MISSION_SWITCH_PREV_NODES[NUMBER_OF_MISSION_SWITCH_PREV_NODES]             = {'Q'};
    PARAM char     BALANCER_PREV_NODE                                                         = 'Q';              // -
    PARAM char     BALANCER_START_NODE                                                        = 'X';              // -
    PARAM char     BALANCER_END_NODE                                                          = 'Y';              // -
    PARAM int      NUMBER_OF_BALANCER_PROHIBITED_EDGES                                        = 1;                // -
    PARAM pcc      BALANCER_PROHIBITED_EDGES[NUMBER_OF_BALANCER_PROHIBITED_EDGES]             = {pcc('V', 'Q')};  // -

    PARAM int   NUMBER_OF_NODES                          = 25;  // -
    PARAM int   NUMBER_OF_GATES                          = 17;  // -
    PARAM char  GATE_NAMES[NUMBER_OF_GATES]              = {'M', 'H', 'C', 'R', 'K', 'F', 'A', 'N', 'I', 'D', 'T', 'L', 'G', 'B', 'O', 'J', 'E'};
    PARAM int   NUMBER_OF_CROSS_SECTIONS                 = 3;
    PARAM cross CROSS_SECTIONS[NUMBER_OF_CROSS_SECTIONS] = {
        cross(pcc('K', 'L'), pcc('N', 'I')), cross(pcc('F', 'G'), pcc('I', 'D')), cross(pcc('T', 'U'), pcc('W', 'O'))};

    PARAM float     START_X           = px_to_m(320.0f);
    PARAM float     START_Y           = px_to_m(724.0f);
    PARAM float     START_ORIENTATION = -M_PI / 2.0f;
    PARAM char      START_GATE        = 'U';
    PARAM char      START_NEXT_GATE   = 'O';
    PARAM Direction START_DIRECTION   = Direction::RIGHT;

#ifndef SIMULATION
    PARAM float     MISSION_SWITCH_FIRST_FORWARD_DIST  = 0.3f;   // m
    PARAM float     MISSION_SWITCH_SECOND_FORWARD_DIST = 0.3f;   // m
    PARAM float     MISSION_SWITCH_STEERING_ANGLE      = 22.5f;  // deg
    PARAM float     MISSION_SWITCH_LATERAL_DIST        = 0.5f;   // m
    PARAM Direction MISSION_SWITCH_DIRECTION           = Direction::RIGHT;

    PARAM float FIRST_FAST_DIST  = 100.0f;  // m
    PARAM float FIRST_SLOW_DIST  = 100.0f;  // m
    PARAM float SECOND_FAST_DIST = 100.0f;  // m
    PARAM float SECOND_SLOW_DIST = 100.0f;  // m
    PARAM float THIRD_FAST_DIST  = 100.0f;  // m
    PARAM float THIRD_SLOW_DIST  = 100.0f;  // m
    PARAM float FOURTH_FAST_DIST = 100.0f;  // m
    PARAM float FOURTH_SLOW_DIST = 100.0f;  // m

#else
    PARAM float     MISSION_SWITCH_FIRST_FORWARD_DIST  = 0.6f;    // m
    PARAM float     MISSION_SWITCH_SECOND_FORWARD_DIST = 0.3f;    // m
    PARAM float     MISSION_SWITCH_STEERING_ANGLE      = 22.5f;   // deg
    PARAM float     MISSION_SWITCH_LATERAL_DIST        = 0.425f;  // m
    PARAM Direction MISSION_SWITCH_DIRECTION           = Direction::RIGHT;

    PARAM float FIRST_FAST_DIST  = px_to_m(290.0f);  // m
    PARAM float FIRST_SLOW_DIST  = 4.6f;             // m
    PARAM float SECOND_FAST_DIST = px_to_m(355.0f);  // m
    PARAM float SECOND_SLOW_DIST = 9.0f;             // m
    PARAM float THIRD_FAST_DIST  = px_to_m(542.0f);  // m
    PARAM float THIRD_SLOW_DIST  = 9.0f;             // m
    PARAM float FOURTH_FAST_DIST = px_to_m(355.0f);  // m
    PARAM float FOURTH_SLOW_DIST = 4.6f;             // m
#endif

#else
    /* STATIC PARAMETERS OF THE TRACK */
    PARAM float    SQUARE_LENGTH                                                              = 0.6f;  // m
    PARAM unsigned BITMAP_SIZE                                                                = 64;    // px
    PARAM char     MISSION_SWITCH_NODE                                                        = 'N';   // -
    PARAM char     MISSION_SWITCH_NEXT_NODE                                                   = 'Q';   // -
    PARAM int      NUMBER_OF_MISSION_SWITCH_PROHIBITED_EDGES                                  = 5;     // -
    PARAM pcc      MISSION_SWITCH_PROHIBITED_EDGES[NUMBER_OF_MISSION_SWITCH_PROHIBITED_EDGES] = {
        pcc('Q', 'N'), pcc('S', 'Q'), pcc('R', 'Q'), pcc('O', 'N'), pcc('M', 'L')};  // -
    PARAM int  NUMBER_OF_MISSION_SWITCH_PREV_NODES                            = 2;        // -
    PARAM char MISSION_SWITCH_PREV_NODES[NUMBER_OF_MISSION_SWITCH_PREV_NODES] = {'K', 'L'};
    PARAM char BALANCER_PREV_NODE                                             = 'X';              // -
    PARAM char BALANCER_START_NODE                                            = 'Z';              // -
    PARAM char BALANCER_END_NODE                                              = '[';              // -
    PARAM int  NUMBER_OF_BALANCER_PROHIBITED_EDGES                            = 1;                // -
    PARAM pcc  BALANCER_PROHIBITED_EDGES[NUMBER_OF_BALANCER_PROHIBITED_EDGES] = {pcc('W', 'X')};  // -

    PARAM int   NUMBER_OF_NODES                          = 27;  // -
    PARAM int   NUMBER_OF_GATES                          = 17;  // -
    PARAM char  GATE_NAMES[NUMBER_OF_GATES]              = {'B', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 'M', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V'};
    PARAM int   NUMBER_OF_CROSS_SECTIONS                 = 3;
    PARAM cross CROSS_SECTIONS[NUMBER_OF_CROSS_SECTIONS] = {
        cross(pcc('F', 'I'), pcc('H', 'G')), cross(pcc('K', 'N'), pcc('M', 'L')), cross(pcc('P', 'S'), pcc('R', 'Q'))};

    PARAM float START_X           = px_to_m(1812.0f);
    PARAM float START_Y           = px_to_m(128.0f);
    PARAM float START_ORIENTATION = M_PI;
    // PARAM char      START_GATE        = 'S';
    // PARAM char      START_NEXT_GATE   = 'V';
    PARAM char      START_GATE      = 'Y';
    PARAM char      START_NEXT_GATE = 'W';
    PARAM Direction START_DIRECTION = Direction::RIGHT;

#ifndef SIMULATION
    PARAM float     MISSION_SWITCH_FIRST_FORWARD_DIST  = 1.9f;   // m
    PARAM float     MISSION_SWITCH_SECOND_FORWARD_DIST = 0.9f;   // m
    PARAM float     MISSION_SWITCH_STEERING_ANGLE      = 22.5f;  // deg
    PARAM float     MISSION_SWITCH_LATERAL_DIST        = 0.8f;   // m
    PARAM Direction MISSION_SWITCH_DIRECTION           = Direction::LEFT;

    PARAM float FIRST_FAST_DIST  = 100.0f;  // m
    PARAM float FIRST_SLOW_DIST  = 100.0f;  // m
    PARAM float SECOND_FAST_DIST = 100.0f;  // m
    PARAM float SECOND_SLOW_DIST = 100.0f;  // m
    PARAM float THIRD_FAST_DIST  = 100.0f;  // m
    PARAM float THIRD_SLOW_DIST  = 100.0f;  // m
    PARAM float FOURTH_FAST_DIST = 100.0f;  // m
    PARAM float FOURTH_SLOW_DIST = 100.0f;  // m
#else
    PARAM float     MISSION_SWITCH_FIRST_FORWARD_DIST  = 1.9f;   // m
    PARAM float     MISSION_SWITCH_SECOND_FORWARD_DIST = 0.9f;   // m
    PARAM float     MISSION_SWITCH_STEERING_ANGLE      = 22.5f;  // deg
    PARAM float     MISSION_SWITCH_LATERAL_DIST        = 0.8f;   // m
    PARAM Direction MISSION_SWITCH_DIRECTION           = Direction::LEFT;

    PARAM float FIRST_FAST_DIST  = 100.0f;  // m
    PARAM float FIRST_SLOW_DIST  = 100.0f;  // m
    PARAM float SECOND_FAST_DIST = 100.0f;  // m
    PARAM float SECOND_SLOW_DIST = 100.0f;  // m
    PARAM float THIRD_FAST_DIST  = 100.0f;  // m
    PARAM float THIRD_SLOW_DIST  = 100.0f;  // m
    PARAM float FOURTH_FAST_DIST = 100.0f;  // m
    PARAM float FOURTH_SLOW_DIST = 100.0f;  // m
#endif
#endif

}  // namespace jlb

#endif  // MAP_COMMON_JXX
