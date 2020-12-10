cmake_minimum_required(VERSION 2.8)

project(modul_cpu_models C)

set(SRC_FILES   ${CMAKE_SOURCE_DIR}/libs/ins/fsnav_ins_alignment.c
                ${CMAKE_SOURCE_DIR}/libs/ins/fsnav_ins_attitude.c
                ${CMAKE_SOURCE_DIR}/libs/ins/fsnav_ins_gravity.c
                ${CMAKE_SOURCE_DIR}/libs/ins/fsnav_ins_motion.c
                ${CMAKE_SOURCE_DIR}/libs/fsnav.c
                ${CMAKE_SOURCE_DIR}/source/fsnav_ins/fsnav_ins.c)

#include_directories(${CMAKE_SOURCE_DIR}/src/include)


add_executable(run_ins ${SRC_FILES})

target_link_libraries(run_ins m)