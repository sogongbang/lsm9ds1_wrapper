#
# Copyright (c) 2022 Sung Ho Park and CSOS
#
# SPDX-License-Identifier: Apache-2.0
#

# ubinos_config_info {"name_base": "lsm9ds1_self_test", "build_type": "cmake_ubinos", "app": true}

set_cache(UBINOS__UBIK__TICK_TYPE "RTC" STRING)

include(${PROJECT_UBINOS_DIR}/config/ubinos_nucleof207zg.cmake)
include(${PROJECT_LIBRARY_DIR}/seggerrtt_wrapper/config/seggerrtt.cmake)
include(${PROJECT_LIBRARY_DIR}/stm32cubef2_wrapper/config/stm32cubef2.cmake)
include(${PROJECT_LIBRARY_DIR}/stm32cubef2_extension/config/stm32cubef2_extension.cmake)
include(${PROJECT_LIBRARY_DIR}/lsm9ds1_wrapper/config/lsm9ds1.cmake)

####

set(INCLUDE__APP TRUE)
set(APP__NAME "lsm9ds1_self_test")

get_filename_component(_tmp_source_dir "${CMAKE_CURRENT_LIST_DIR}/${APP__NAME}" ABSOLUTE)
string(TOLOWER ${UBINOS__BSP__BOARD_VARIATION_NAME} _temp_board_model)

include_directories(${_tmp_source_dir}/arch/arm/cortexm/${_temp_board_model}/Inc)
include_directories(${_tmp_source_dir})

file(GLOB_RECURSE _tmp_sources
    "${_tmp_source_dir}/*.c"
    "${_tmp_source_dir}/*.cpp"
    "${_tmp_source_dir}/*.cc"
    "${_tmp_source_dir}/*.S")

set(PROJECT_APP_SOURCES ${PROJECT_APP_SOURCES} ${_tmp_sources})

