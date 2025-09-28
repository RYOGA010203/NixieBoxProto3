/* ==== gps.h ========================================================== */
#pragma once
#include "main.h"
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint8_t  fix;       /* 1=有効(A), 0=無効(V) */
    uint8_t  hh, mm, ss;/* UTC時刻 */
    double   lat_deg;   /* 緯度[deg], 北+ 南- */
    double   lon_deg;   /* 経度[deg], 東+ 西- */
} gps_rmc_t;

void    gps_init(UART_HandleTypeDef *huart);
uint8_t gps_poll_line(gps_rmc_t *out);   /* 1行分(RMC)が読めたら1 */
