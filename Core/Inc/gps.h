#pragma once
#include "main.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== UTC 時刻 ===== */
extern volatile int g_UTC_hh;  /* 0..23 or -1 */
extern volatile int g_UTC_mm;  /* 0..59 or -1 */
extern volatile int g_UTC_ss;  /* 0..59 or -1 */

/* ===== UTC 日付（RMCのddmmyy → 2000+yy） ===== */
extern volatile int g_UTC_YYYY; /* 4桁 or -1 */
extern volatile int g_UTC_MM;   /* 1..12 or -1 */
extern volatile int g_UTC_DD;   /* 1..31 or -1 */

/* ===== 現地時間（分・秒はUTC、時のみ経度で補正） ===== */
extern volatile int g_LCL_hh;   /* 0..23 or -1 */
extern volatile int g_LCL_mm;   /* = g_UTC_mm */
extern volatile int g_LCL_ss;   /* = g_UTC_ss */

/* ===== 現地日付（UTC日付＋経度由来の時差で日跨ぎ補正） ===== */
extern volatile int g_LCL_YYYY; /* 4桁 or -1 */
extern volatile int g_LCL_MM;   /* 1..12 or -1 */
extern volatile int g_LCL_DD;   /* 1..31 or -1 */

/* ===== 位置・高度・速度（float 32bit） ===== */
extern volatile float g_LTT;    /* 緯度[deg], 北+ 南- */
extern volatile float g_LGT;    /* 経度[deg], 東+ 西- */
extern volatile float g_ALT;    /* 高度[m] (GGA) */
extern volatile float g_SPD;    /* 速度[m/s] (RMC knots→m/s) */

/* ===== API ===== */
void    gps_init(UART_HandleTypeDef *huart);
/* NMEAを1行ずつ解析。RMC/GGAで上記グローバルを更新。
   返値: 0=更新なし, 1=RMC更新, 2=GGA更新, 3=両方 */
uint8_t gps_poll_line(void);

/* ===== デバッグ指標 ===== */
extern volatile uint32_t gps_rx_bytes;
extern volatile uint32_t gps_rx_lines;
extern volatile uint32_t gps_rmc_ok, gps_rmc_bad;
extern volatile uint32_t gps_gga_ok, gps_gga_bad;

#define GPS_LAST_SENTENCE_MAX 82
extern volatile char     gps_last_sentence[GPS_LAST_SENTENCE_MAX];

#ifdef __cplusplus
}
#endif
