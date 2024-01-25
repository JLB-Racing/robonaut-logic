#ifndef MAP_COMMON_HXX
#define MAP_COMMON_HXX

#include "types.hxx"

///////////////////////////////////////////////////////////////////////////
//
//      DEFINES
//

// #define Q2

namespace jlb
{

#ifndef Q2
    /* STATIC PARAMETERS OF THE TRACK */
    PARAM float    SQUARE_LENGTH                                                  = 0.6f;             // m
    PARAM unsigned BITMAP_SIZE                                                    = 64;               // px
    PARAM char     MISSION_SWITCH_NODE                                            = 'V';              // -
    PARAM int      NUMER_OF_PREV_GATES                                            = 1;                // -
    PARAM char     MISSION_SWITCH_PREV_NODES[NUMER_OF_PREV_GATES]                 = {'Q'};            // -
    PARAM char     BALANCER_PREV_NODE                                             = 'Q';              // -
    PARAM char     BALANCER_START_NODE                                            = 'X';              // -
    PARAM char     BALANCER_END_NODE                                              = 'Y';              // -
    PARAM int      NUMBER_OF_BALANCER_PROHIBITED_EDGES                            = 1;                // -
    PARAM pcc      BALANCER_PROHIBITED_EDGES[NUMBER_OF_BALANCER_PROHIBITED_EDGES] = {pcc('V', 'Q')};  // -

    PARAM int   NUMBER_OF_GATES                          = 17;  // -
    PARAM char  GATE_NAMES[NUMBER_OF_GATES]              = {'M', 'H', 'C', 'R', 'K', 'F', 'A', 'N', 'I', 'D', 'T', 'L', 'G', 'B', 'O', 'J', 'E'};
    PARAM int   NUMBER_OF_CROSS_SECTIONS                 = 3;
    PARAM cross CROSS_SECTIONS[NUMBER_OF_CROSS_SECTIONS] = {
        cross(pcc('K', 'L'), pcc('N', 'I')), cross(pcc('F', 'G'), pcc('I', 'D')), cross(pcc('T', 'U'), pcc('W', 'O'))};
#else
    /* STATIC PARAMETERS OF THE TRACK */
    PARAM float    SQUARE_LENGTH                                                  = 0.3f;             // m
    PARAM unsigned BITMAP_SIZE                                                    = 64;               // px
    PARAM char     MISSION_SWITCH_NODE                                            = 'N';              // -
    PARAM int      NUMER_OF_PREV_GATES                                            = 2;                // -
    PARAM char     MISSION_SWITCH_PREV_NODES[NUMER_OF_PREV_GATES]                 = {'K', 'L'};       // -
    PARAM char     BALANCER_PREV_NODE                                             = 'X';              // -
    PARAM char     BALANCER_START_NODE                                            = 'Z';              // -
    PARAM char     BALANCER_END_NODE                                              = '[';              // -
    PARAM int      NUMBER_OF_BALANCER_PROHIBITED_EDGES                            = 1;                // -
    PARAM pcc      BALANCER_PROHIBITED_EDGES[NUMBER_OF_BALANCER_PROHIBITED_EDGES] = {pcc('W', 'X')};  // -

    PARAM int   NUMBER_OF_GATES                          = 17;  // -
    PARAM char  GATE_NAMES[NUMBER_OF_GATES]              = {'B', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 'M', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V'};
    PARAM int   NUMBER_OF_CROSS_SECTIONS                 = 3;
    PARAM cross CROSS_SECTIONS[NUMBER_OF_CROSS_SECTIONS] = {
        cross(pcc('F', 'I'), pcc('H', 'G')), cross(pcc('K', 'N'), pcc('M', 'L')), cross(pcc('P', 'S'), pcc('R', 'Q'))};
#endif

}  // namespace jlb

#endif  // MAP_COMMON_JXX