#pragma once
#include <stdint.h>
#include "main.h"  // SRx_* ピン定義/ポート定義を参照

#ifdef __cplusplus
extern "C" {
#endif

/* --- アクティブLow駆動なら有効化（必要ならビルド設定で -DNIXIE_ACTIVE_LOW ）--- */
/* // #define NIXIE_ACTIVE_LOW */

void nixie_init(void);                         /* 内部状態の初期化（有効マスク=0xFF） */
void nixie_set_enable_mask(uint8_t mask);      /* 1=表示許可(桁単位) 左→右でbit7..bit0 */

void nixie_show_integer8_str(const char *s8);  /* '0'..'9' と先頭 '-' を解釈（'-'=左右ドット1桁） */
void nixie_show_decimal_str(const char *s8);   /* '0'..'9' と '.' を解釈（'.'=左ドット1桁） */
void nixie_show_time_hms(uint8_t hh, uint8_t mm, uint8_t ss); /* "HH.MM.SS" 形式表示 */

/* 直接8桁の数字（0-9）を左→右で指定して表示（演出用） */
void nixie_show_digits_lr(uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,
                          uint8_t d4,uint8_t d5,uint8_t d6,uint8_t d7);

#ifdef __cplusplus
}
#endif
