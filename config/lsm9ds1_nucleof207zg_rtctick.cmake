#
# Copyright (c) 2022 Sung Ho Park and CSOS
#
# SPDX-License-Identifier: Apache-2.0
#

# ubinos_config_info {"name_base": "lsm9ds1", "build_type": "cmake_ubinos"}

set_cache(UBINOS__UBIK__TICK_TYPE "RTC" STRING)

include(${PROJECT_UBINOS_DIR}/config/ubinos_nucleof207zg.cmake)
include(${PROJECT_LIBRARY_DIR}/seggerrtt_wrapper/config/seggerrtt.cmake)
include(${PROJECT_LIBRARY_DIR}/stm32cubef2_wrapper/config/stm32cubef2.cmake)
include(${PROJECT_LIBRARY_DIR}/stm32cubef2_extension/config/stm32cubef2_extension.cmake)

include(${CMAKE_CURRENT_LIST_DIR}/lsm9ds1.cmake)

####

string(TOLOWER ${UBINOS__BSP__BOARD_VARIATION_NAME} _tmp_board_model)

get_filename_component(_tmp_source_dir "${PROJECT_LIBRARY_DIR}/ArduinoCore-API_wrapper/source/arduinocore_api_ubinos/arch/arm/cortexm/${_tmp_board_model}" ABSOLUTE)

include_directories(${_tmp_source_dir}/Inc)

