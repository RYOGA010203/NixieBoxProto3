#include "main.h"
#include "gps.h"
#include <stdint.h>
#include <stdlib.h>

 gps_rmc_t r;

/* ===== アクティブLow駆動なら有効化 ===== */
// #define NIXIE_ACTIVE_LOW

/* ===== IN-14 実機ビット割り当て =====
   bit0..8 = 1..9, bit9 = 左ドット, bit10 = 右ドット, bit11 = 0
   → 数字0..9を下表へ正規化（0→bit11, 1→bit0, …, 9→bit8） */
static const uint8_t NIXIE_BIT_OF_DIG[10] = {
    11, /* 0 */
     0, /* 1 */
     1, /* 2 */
     2, /* 3 */
     3, /* 4 */
     4, /* 5 */
     5, /* 6 */
     6, /* 7 */
     7, /* 8 */
     8  /* 9 */
};

/* ===== ドット位置定義 ===== */
#define DOT_L_BIT   9
#define DOT_R_BIT  10
#define BIT1(n)    ((uint16_t)(1U<<(n)))

/* ===== 数字＋左右ドット→12bit（Active-High/Low 両対応） ===== */
static inline uint16_t NIXIE_CODE(uint8_t digit, uint8_t dotL, uint8_t dotR)
{
    uint16_t raw = 0U;
    if (digit < 10U) raw |= BIT1(NIXIE_BIT_OF_DIG[digit]); /* 数字 */
    if (dotL)        raw |= BIT1(DOT_L_BIT);               /* 左ドット */
    if (dotR)        raw |= BIT1(DOT_R_BIT);               /* 右ドット */

#ifndef NIXIE_ACTIVE_LOW
    return raw;                         /* Highで点灯 */
#else
    return (uint16_t)(~raw) & 0x0FFFU;  /* Lowで点灯（12bitのみ有効） */
#endif
}

#ifndef NIXIE_ACTIVE_LOW
  #define BLANK_12      ((uint16_t)0x0000)        /* 全消灯 */
#else
  #define BLANK_12      ((uint16_t)0x0FFF)
#endif

// CubeMXが生成したUSART1ハンドルを使う場合
extern UART_HandleTypeDef huart1;

// 既存：ニキシー表示API
//void nixie_show_time_hms(uint8_t hh, uint8_t mm, uint8_t ss);

static void utc_to_jst(uint8_t uh, uint8_t um, uint8_t us,
                       uint8_t *jh, uint8_t *jm, uint8_t *js)
{
    int h = (int)uh + 9;  // JST = UTC + 9h
    int m = (int)um;
    int s = (int)us;
    if(h >= 24) h -= 24;
    *jh = (uint8_t)h; *jm = (uint8_t)m; *js = (uint8_t)s;
}

/* ===== 各SER独立I/O ===== */
static inline void SR0_set(uint8_t b){ HAL_GPIO_WritePin(SR0_GPIO_Port, SR0_Pin, b?GPIO_PIN_SET:GPIO_PIN_RESET); }
static inline void SR1_set(uint8_t b){ HAL_GPIO_WritePin(SR1_GPIO_Port, SR1_Pin, b?GPIO_PIN_SET:GPIO_PIN_RESET); }
static inline void SR2_set(uint8_t b){ HAL_GPIO_WritePin(SR2_GPIO_Port, SR2_Pin, b?GPIO_PIN_SET:GPIO_PIN_RESET); }
static inline void SR3_set(uint8_t b){ HAL_GPIO_WritePin(SR3_GPIO_Port, SR3_Pin, b?GPIO_PIN_SET:GPIO_PIN_RESET); }
static inline void SR4_set(uint8_t b){ HAL_GPIO_WritePin(SR4_GPIO_Port, SR4_Pin, b?GPIO_PIN_SET:GPIO_PIN_RESET); }
static inline void SR5_set(uint8_t b){ HAL_GPIO_WritePin(SR5_GPIO_Port, SR5_Pin, b?GPIO_PIN_SET:GPIO_PIN_RESET); }
static inline void SR6_set(uint8_t b){ HAL_GPIO_WritePin(SR6_GPIO_Port, SR6_Pin, b?GPIO_PIN_SET:GPIO_PIN_RESET); }
static inline void SR7_set(uint8_t b){ HAL_GPIO_WritePin(SR7_GPIO_Port, SR7_Pin, b?GPIO_PIN_SET:GPIO_PIN_RESET); }

static inline void SHCP_pulse(void){ HAL_GPIO_WritePin(SHCP_GPIO_Port, SHCP_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(SHCP_GPIO_Port, SHCP_Pin, GPIO_PIN_RESET); }
static inline void STCP_latch(void){ HAL_GPIO_WritePin(STCP_GPIO_Port, STCP_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(STCP_GPIO_Port, STCP_Pin, GPIO_PIN_RESET); }

/* ===== 8本SERの各ビット出力（同期ビットバン用） ===== */
static inline void SR0_set_bit(uint16_t v,int8_t i){ SR0_set((uint8_t)((v>>i)&1U)); }
static inline void SR1_set_bit(uint16_t v,int8_t i){ SR1_set((uint8_t)((v>>i)&1U)); }
static inline void SR2_set_bit(uint16_t v,int8_t i){ SR2_set((uint8_t)((v>>i)&1U)); }
static inline void SR3_set_bit(uint16_t v,int8_t i){ SR3_set((uint8_t)((v>>i)&1U)); }
static inline void SR4_set_bit(uint16_t v,int8_t i){ SR4_set((uint8_t)((v>>i)&1U)); }
static inline void SR5_set_bit(uint16_t v,int8_t i){ SR5_set((uint8_t)((v>>i)&1U)); }
static inline void SR6_set_bit(uint16_t v,int8_t i){ SR6_set((uint8_t)((v>>i)&1U)); }
static inline void SR7_set_bit(uint16_t v,int8_t i){ SR7_set((uint8_t)((v>>i)&1U)); }

/* ===== 8桁 enable マスク（1=有効） ===== */
static volatile uint8_t g_sr_enable = 0xFFU;
static inline void SR_enable_mask_set(uint8_t m){ g_sr_enable = m; }

/* ===== 同期シフト：引数は SR7,SR6,...,SR0 の順 ===== */
static void shift12_sync_masked(uint16_t v7,uint16_t v6,uint16_t v5,uint16_t v4,
                                uint16_t v3,uint16_t v2,uint16_t v1,uint16_t v0)
{
    if(!(g_sr_enable & (1U<<7))) v7 = BLANK_12;
    if(!(g_sr_enable & (1U<<6))) v6 = BLANK_12;
    if(!(g_sr_enable & (1U<<5))) v5 = BLANK_12;
    if(!(g_sr_enable & (1U<<4))) v4 = BLANK_12;
    if(!(g_sr_enable & (1U<<3))) v3 = BLANK_12;
    if(!(g_sr_enable & (1U<<2))) v2 = BLANK_12;
    if(!(g_sr_enable & (1U<<1))) v1 = BLANK_12;
    if(!(g_sr_enable & (1U<<0))) v0 = BLANK_12;

    for(int8_t i=11;i>=0;--i){
        SR7_set_bit(v7,i); SR6_set_bit(v6,i); SR5_set_bit(v5,i); SR4_set_bit(v4,i);
        SR3_set_bit(v3,i); SR2_set_bit(v2,i); SR1_set_bit(v1,i); SR0_set_bit(v0,i);
        SHCP_pulse();
    }
}

/* ===== 8桁を“見た目の左→右”で指定するAPI（方向矯正） =====
   l0..l7 は左端→右端。実ハードの (SR7..SR0) に合わせて反転割当。 */
static inline void display8_codes_lr(uint16_t l0,uint16_t l1,uint16_t l2,uint16_t l3,
                                     uint16_t l4,uint16_t l5,uint16_t l6,uint16_t l7)
{
    /* 左→右の並び(l0..l7)を SR7..SR0 へ割当（左右逆転を解消） */
    shift12_sync_masked(l7,l6,l5,l4,l3,l2,l1,l0);
    STCP_latch();
}

/* ===== 短縮ヘルパ ===== */
static inline uint16_t code_digit(uint8_t d){ return NIXIE_CODE(d,0,0); }
static inline uint16_t code_dot_only(void){ return NIXIE_CODE(0xFF,1,0); }   /* 左ドットのみ（1桁） */
static inline uint16_t code_sign_bothdots(void){ return NIXIE_CODE(0xFF,1,1); } /* 負号＝左右ドット */
static inline uint16_t code_blank(void){ return BLANK_12; }

/* ===== 表示ユーティリティ（基本：左→右で指定） ===== */
static inline void display8_digits_lr(uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,
                                      uint8_t d4,uint8_t d5,uint8_t d6,uint8_t d7)
{
    display8_codes_lr(code_digit(d0),code_digit(d1),code_digit(d2),code_digit(d3),
                      code_digit(d4),code_digit(d5),code_digit(d6),code_digit(d7));
}

static inline void display8_same_digit(uint8_t d)
{
    d %= 10U;
    display8_digits_lr(d,d,d,d,d,d,d,d);
}

/* ===== テキスト8桁：'0'〜'9' と '.' と 先頭 '-'（負号） =====
   - s8 の先頭8文字を左→右にそのまま配置
   - '0'〜'9' : 数字
   - '.'     : 左ドットのみ（1桁）
   - 先頭 '-' : 左右ドットのみ（1桁）→負号
   - その他  : 消灯
*/
static void nixie_show_text8_core(const char *s8, uint8_t accept_leading_minus)
{
    uint16_t v[8];
    for(int i=0;i<8;i++) v[i] = code_blank();

    int pos = 0;
    const char* p = s8 ? s8 : "";

    /* 先頭負号処理（1桁消費） */
    if(accept_leading_minus && *p=='-' && pos<8){
        v[pos++] = code_sign_bothdots();
        ++p;
    }

    /* 残り文字を流し込み */
    while(pos < 8 && *p){
        char c = *p++;
        if(c>='0' && c<='9'){
            v[pos++] = code_digit((uint8_t)(c-'0'));
        }else if(c=='.'){
            v[pos++] = code_dot_only();     /* 左ドットのみ（1桁） */
        }else{
            v[pos++] = code_blank();        /* その他は消灯 */
        }
    }

    display8_codes_lr(v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7]);
}

/* ===== 公開API：整数/小数/時刻 ===== */
/* 整数（例：01234567／-1234567 など） */
void nixie_show_integer8_str(const char *s8)
{
    nixie_show_text8_core(s8, /*accept_leading_minus=*/1U);
}

/* 小数（例：135.4567／-35.4567 など。'.'は1桁消費、左ドットのみ） */
void nixie_show_decimal_str(const char *s8)
{
    nixie_show_text8_core(s8, /*accept_leading_minus=*/1U);
}

/* 時刻（例：12.53.20）— “.” は左ドットの1桁として入れる */
void nixie_show_time_hms(uint8_t hh, uint8_t mm, uint8_t ss)
{
    char buf[9];
    buf[0] = (char)('0' + ((hh/10)%10));
    buf[1] = (char)('0' + (hh%10));
    buf[2] = '.';
    buf[3] = (char)('0' + ((mm/10)%10));
    buf[4] = (char)('0' + (mm%10));
    buf[5] = '.';
    buf[6] = (char)('0' + ((ss/10)%10));
    buf[7] = (char)('0' + (ss%10));
    buf[8] = '\0';
    nixie_show_text8_core(buf, /*accept_leading_minus=*/0U);
}

/* ===== シャッフル効果: start=1/3s → peak=1/20s → end=1/10s ===== */
#ifndef SHUF_START_DIV
#define SHUF_START_DIV 3    /* 1/3 秒 */
#endif
#ifndef SHUF_PEAK_DIV
#define SHUF_PEAK_DIV  20   /* 1/20 秒 */
#endif
#ifndef SHUF_END_DIV
#define SHUF_END_DIV   10   /* 1/10 秒 */
#endif

static volatile uint8_t  g_shuffle_req  = 0U;
static volatile uint8_t  g_shuffle_busy = 0U;

/* SW_EX の EXTIコールバック（アクティブLow） */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == SW_EX_Pin){
        if(g_shuffle_req == 0){
            g_shuffle_req = 1U;
        }
    }
}

static void shuffle_effect(void)
{
    g_shuffle_busy = 1U;

    /* ランダム8桁（数字のみ） */
    uint8_t d[8];
    for(int i=0;i<8;i++) d[i] = (uint8_t)(rand()%10U);
    display8_digits_lr(d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);

    /* 1/3 → … → 1/20 */
    for (uint8_t div = SHUF_START_DIV; div <= SHUF_PEAK_DIV; ++div) {
        HAL_Delay(1000U / div);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        for(int i=0;i<8;i++) d[i] = (uint8_t)(rand()%10U);
        display8_digits_lr(d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);
    }
    /* 1/19 → … → 1/10 */
    for (int div = SHUF_PEAK_DIV - 1; div >= SHUF_END_DIV; --div) {
        HAL_Delay(1000U / (uint32_t)div);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        for(int i=0;i<8;i++) d[i] = (uint8_t)(rand()%10U);
        display8_digits_lr(d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);
    }

    /* 余韻 */
    for(int i=0;i<8;i++) d[i] = (uint8_t)(rand()%10U);
    display8_digits_lr(d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);
    g_shuffle_busy = 0U;
}

/* ===================================================================== */
/* ============================= user_main ============================== */
/* ===================================================================== */
/* user_main() は通常の main() の while ループの手前で呼び出してください。 */
void user_main(void)
{
    srand((unsigned)(HAL_GetTick()));
    SR_enable_mask_set(0xFFU);

    gps_init(&huart1);  // ← これだけで受信割込みが走り始める

    /* 動作確認（左右順・ドット表現・負号） */
    nixie_show_integer8_str("01234567");  /* 左→右で 0..7 */
    HAL_Delay(1000);
    nixie_show_decimal_str("135.4567");   /* “.”は左ドット1桁 */
    HAL_Delay(1000);
    nixie_show_time_hms(12,53,20);        /* 12.53.20 */
    HAL_Delay(1000);

    /* 動作確認（左右順・ドット表現・負号） */
    nixie_show_integer8_str("-1234");  /* 左→右で 0..7 */
    HAL_Delay(1000);
    nixie_show_decimal_str("-135.45");   /* “.”は左ドット1桁 */
    HAL_Delay(1000);
    nixie_show_time_hms(12,53,20);        /* 12.53.20 */
    HAL_Delay(1000);

    /* 動作確認（左右順・ドット表現・負号） */
    nixie_show_integer8_str("-0123456789");  /* 左→右で 0..7 */
    HAL_Delay(1000);
    nixie_show_decimal_str("-135.456789");   /* “.”は左ドット1桁 */
    HAL_Delay(1000);
    nixie_show_time_hms(12,53,20);        /* 12.53.20 */
    HAL_Delay(1000);

    uint8_t  dig = 0U;
    uint32_t t_prev = HAL_GetTick();

    while(1){
        if(g_shuffle_req){
            shuffle_effect();
            t_prev = HAL_GetTick();
            g_shuffle_req = 0U;
        }

        uint32_t now = HAL_GetTick();
        if((now - t_prev) >= 1000U && !g_shuffle_busy){
            t_prev += 1000U;

            /* 1秒ごとに全桁同一数字（インジケータ） */
            display8_same_digit(dig);  /* 00000000 → 11111111 → ... */
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            dig = (uint8_t)((dig + 1U) % 10U);

            /* 任意表示に差し替え可：
               nixie_show_integer8_str("-1234567");
               nixie_show_decimal_str("-135.4567");
               nixie_show_time_hms(23, 59, 59);
            */

            //gps_rmc_t r;
             if (gps_poll_line(&r)) {           // RMC 1行取れたら1
                 if (r.fix) {
                     uint8_t hh, mm, ss;
                     utc_to_jst(r.hh, r.mm, r.ss, &hh, &mm, &ss);
                     nixie_show_time_hms(hh, mm, ss);
                 } else {
                     // 未Fix時の演出（例：12.00.00を点滅、または "  .  .  " 的な表示）
                     // nixie_show_time_hms(12, 0, 0);
                 }
             }
        }
    }
}
