#
# Copyright (c) 2021 Sung Ho Park and CSOS
#
# SPDX-License-Identifier: Apache-2.0
#

set(INCLUDE__LSM9DS1 TRUE)
set(PROJECT_UBINOS_LIBRARIES ${PROJECT_UBINOS_LIBRARIES} lsm9ds1_wrapper)

set_cache_default(LSM9DS1__BASE_DIR "${PROJECT_LIBRARY_DIR}/lsm9ds1" STRING "LSM9DS1 project base dir")

