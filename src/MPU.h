/*  Header file for MPU9250 class library
 *
 *  Copyright 2017 Tlera Corporation
 *  
 *  Created by Kris Winer
 *
 *  Adapted by Simon D. 2018
 *  
 *  Library may be used freely and without limit with attribution.
 */

#pragma once

#include <stdint.h>

typedef enum {

    AFS_2G,
    AFS_4G,  
    AFS_8G,  
    AFS_16G 

} Ascale_t;

typedef enum {

    GFS_250DPS,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS

} Gscale_t;

typedef enum {

    MPU_ERROR_NONE,
    MPU_ERROR_CONNECT,
    MPU_ERROR_IMU_ID,
    MPU_ERROR_MAG_ID,
    MPU_ERROR_SELFTEST

} MPU_Error_t;

