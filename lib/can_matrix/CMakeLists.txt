cmake_minimum_required(VERSION 3.1...3.26)

project(
    can_matrix
    VERSION 1.0
    LANGUAGES C
)

include_directories(
    ${PROJECT_SOURCE_DIR}/butl
    ${PROJECT_SOURCE_DIR}/conf
    ${PROJECT_SOURCE_DIR}/lib
)

add_library(can_matrix STATIC butl/jlb-binutil.c lib/jlb.c)
target_include_directories(can_matrix PUBLIC
    ${PROJECT_SOURCE_DIR}/butl
    ${PROJECT_SOURCE_DIR}/conf
    ${PROJECT_SOURCE_DIR}/lib
)

get_target_property(TARGET_COMPILE_OPTIONS can_matrix COMPILE_OPTIONS)
list(REMOVE_ITEM TARGET_COMPILE_OPTIONS -Werror -Wunused-function)
set_target_properties(can_matrix PROPERTIES COMPILE_OPTIONS "${TARGET_COMPILE_OPTIONS}")

