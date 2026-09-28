cmake_minimum_required(VERSION 3.13)

# ESP-IDF picks this branch up when the file is used as a component.
if(DEFINED ENV{IDF_PATH} AND COMMAND idf_component_register)
  idf_component_register(SRCS src/ad_wire.c src/ad_params.c src/ad_mission.c
                              src/ad_cal.c src/ardudeck.c
                         INCLUDE_DIRS include
                         PRIV_INCLUDE_DIRS src)
  return()
endif()

project(ardudeck_vehicle_sdk C)

add_library(ardudeck_sdk STATIC
  src/ad_wire.c
  src/ad_params.c
  src/ad_mission.c
  src/ad_cal.c
  src/ardudeck.c
)
add_library(ardudeck::sdk ALIAS ardudeck_sdk)

target_include_directories(ardudeck_sdk
  PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)
set_target_properties(ardudeck_sdk PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
target_compile_options(ardudeck_sdk PRIVATE -Wall -Wextra)
