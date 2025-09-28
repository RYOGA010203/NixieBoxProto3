#include "main.h"
#include "gps.h"
#include "nixie.h"
#include <stdlib.h>

/* ===== シャッフル効果パラメータ ===== */
#ifndef SHUF_START_DIV
#define SHUF_START_DIV 3
#endif
#ifndef SHUF_PEAK_DIV
#define SHUF_PEAK_DIV  20
#endif
#ifndef SHUF_END_DIV
#define SHUF_END_DIV   10
#endif

/* ===== シャッフル制御フラグ ===== */
static volatile uint8_t  g_shuffle_req  = 0U;
static volatile uint8_t  g_shuffle_busy = 0U;

/* ====== 表示モードと時刻カウンタ ====== */
typedef enum { DISP_LOCAL = 0, DISP_UTC = 1 } disp_mode_t;
static volatile disp_mode_t g_disp_mode = DISP_LOCAL;
static volatile uint32_t    g_utc_btn_last_tick = 0;   /* 150msデバウンス */

static volatile int disp_hh = -1, disp_mm = -1, disp_ss = -1;

/* ===== GPS→表示カウンタ同期 ===== */
static void sync_display_time_from_gps(void)
{
    if (g_disp_mode == DISP_UTC) {
        if (g_UTC_hh >= 0 && g_UTC_mm >= 0 && g_UTC_ss >= 0) {
            disp_hh = g_UTC_hh; disp_mm = g_UTC_mm; disp_ss = g_UTC_ss;
        }
    } else {
        if (g_LCL_hh >= 0 && g_LCL_mm >= 0 && g_LCL_ss >= 0) {
            disp_hh = g_LCL_hh; disp_mm = g_LCL_mm; disp_ss = g_LCL_ss;
        }
    }
}

/* ===== 1秒進める ===== */
static inline void tick_display_1s(void)
{
    if (disp_hh < 0) return;
    if (++disp_ss >= 60) { disp_ss = 0; if (++disp_mm >= 60) { disp_mm = 0; if (++disp_hh >= 24) disp_hh = 0; } }
}

/* ===== EXTI（ボタン） ===== */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == SW_EX_Pin){            /* シャッフル要求 */
        if(!g_shuffle_req) g_shuffle_req = 1U;
        return;
    }
    if (GPIO_Pin == SW_UTC_Pin) {         /* UTC↔現地 切替 */
        uint32_t now = HAL_GetTick();
        if ((now - g_utc_btn_last_tick) >= 150U) {
            g_utc_btn_last_tick = now;
            g_disp_mode = (g_disp_mode == DISP_UTC) ? DISP_LOCAL : DISP_UTC;
            sync_display_time_from_gps();
        }
        return;
    }
}

/* ===== シャッフル実装 ===== */
static void shuffle_effect(void)
{
    g_shuffle_busy = 1U;

    uint8_t d[8];
    for (int i=0;i<8;i++) d[i]=(uint8_t)(rand()%10U);
    nixie_show_digits_lr(d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);

    for (uint8_t div=SHUF_START_DIV; div<=SHUF_PEAK_DIV; ++div){
        HAL_Delay(1000U/div);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        for (int i=0;i<8;i++) d[i]=(uint8_t)(rand()%10U);
        nixie_show_digits_lr(d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);
    }
    for (int div=SHUF_PEAK_DIV-1; div>=SHUF_END_DIV; --div){
        HAL_Delay(1000U/(uint32_t)div);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        for (int i=0;i<8;i++) d[i]=(uint8_t)(rand()%10U);
        nixie_show_digits_lr(d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);
    }

    for (int i=0;i<8;i++) d[i]=(uint8_t)(rand()%10U);
    nixie_show_digits_lr(d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);

    g_shuffle_busy = 0U;
}

/* ===== エントリ =====
   main() の while ループ直前で呼ぶ */
void user_main(void)
{
    srand((unsigned)HAL_GetTick());

    nixie_init();
    nixie_set_enable_mask(0xFF);   /* 全桁有効 */

    sync_display_time_from_gps();  /* 取れている側に初期同期 */

    uint32_t t_prev = HAL_GetTick();

    while (1) {
        if (g_shuffle_req) {
            shuffle_effect();
            t_prev = HAL_GetTick();    /* タイマ補正 */
            g_shuffle_req = 0U;
        }

        uint32_t now = HAL_GetTick();
        if ((now - t_prev) >= 1000U && !g_shuffle_busy) {
            t_prev += 1000U;

            if (disp_hh < 0) sync_display_time_from_gps();
            else             tick_display_1s();

            if (disp_hh >= 0)
                nixie_show_time_hms((uint8_t)disp_hh,(uint8_t)disp_mm,(uint8_t)disp_ss);

            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }
    }
}
