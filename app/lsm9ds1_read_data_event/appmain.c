/*
 * Copyright (c) 2021 Sung Ho Park and CSOS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ubinos.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "main.h"

static void root_func(void * arg);

int appmain(int argc, char * argv[])
{
    int r;
    (void) r;

    r = task_create(NULL, root_func, NULL, task_getlowestpriority(), 0, "root_func");
    ubi_assert(r == 0);

    ubik_comp_start();

    return 0;
}

static void root_func(void * arg)
{
    printf("\n\n\n");
    printf("================================================================================\n");
    printf("lsm9ds1_read_data_event (build time: %s %s)\n", __TIME__, __DATE__);
    printf("================================================================================\n");
    printf("\n");

    lsm9ds1_read_data_event();
}

