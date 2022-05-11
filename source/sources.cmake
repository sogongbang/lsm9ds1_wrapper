#
# Copyright (c) 2021 Sung Ho Park and CSOS
#
# SPDX-License-Identifier: Apache-2.0
#

if(INCLUDE__LSM9DS1)

get_filename_component(_tmp_source_dir "${LSM9DS1__BASE_DIR}" ABSOLUTE)

include_directories(${_tmp_source_dir})

set(PROJECT_SOURCES ${PROJECT_SOURCES} ${_tmp_source_dir}/lsm9ds1_reg.c)

endif(INCLUDE__LSM9DS1)

