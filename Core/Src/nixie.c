#include "nixie.h"
#include <stdlib.h>

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

#define DOT_L_BIT   9
#define DOT_R_BIT  10
#define BIT1(n)    ((uint16_t)(1U<<(n)))

#ifndef NIXIE_ACTIVE_LOW
  #define BLANK_12   ((uint16_t)0x0000)
#else
  #define BLANK_12   ((uint16_t)0x0FFF)
#endif

static inline uint16_t NIXIE_CODE(uint8_t digit, uint8_t dotL, uint8_t dotR)
{
    uint16_t raw = 0U;
    if (digit < 10U) raw |= BIT1(NIXIE_BIT_OF_DIG[digit]);
    if (dotL)        raw |= BIT1(DOT_L_BIT);
    if (dotR)        raw |= BIT1(DOT_R_BIT);
#ifndef NIXIE_ACTIVE_LOW
    return raw;
#else
    return (uint16_t)(~raw) & 0x0FFFU;
#endif
}

/* ===== 低レベルGPIO（8本のシリアル入力線） ===== */
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

void nixie_set_enable_mask(uint8_t mask){ g_sr_enable = mask; }

void nixie_init(void)
{
    g_sr_enable = 0xFFU;
}

/* ===== 内部：同期シフト  SR引数は SR7..SR0 の順 ===== */
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

/* ===== 左→右(l0..l7) を SR7..SR0 へ（左右逆転の吸収） ===== */
static inline void display8_codes_lr(uint16_t l0,uint16_t l1,uint16_t l2,uint16_t l3,
                                     uint16_t l4,uint16_t l5,uint16_t l6,uint16_t l7)
{
    shift12_sync_masked(l7,l6,l5,l4,l3,l2,l1,l0);
    STCP_latch();
}

/* ===== ヘルパ ===== */
static inline uint16_t code_digit(uint8_t d){ return NIXIE_CODE(d,0,0); }
static inline uint16_t code_dot_only(void){ return NIXIE_CODE(0xFF,1,0); }          /* 左ドット1桁 */
static inline uint16_t code_sign_bothdots(void){ return NIXIE_CODE(0xFF,1,1); }     /* 負号（左右ドット） */
static inline uint16_t code_blank(void){ return BLANK_12; }

/* -- 公開：数字8桁（左→右） -- */
void nixie_show_digits_lr(uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,
                          uint8_t d4,uint8_t d5,uint8_t d6,uint8_t d7)
{
    display8_codes_lr(code_digit(d0),code_digit(d1),code_digit(d2),code_digit(d3),
                      code_digit(d4),code_digit(d5),code_digit(d6),code_digit(d7));
}

/* -- 内部：文字列8桁 → 表示コード -- */
static void nixie_show_text8_core(const char *s8, uint8_t accept_leading_minus)
{
    uint16_t v[8];
    for(int i=0;i<8;i++) v[i] = code_blank();

    int pos = 0;
    const char* p = s8 ? s8 : "";

    if(accept_leading_minus && *p=='-' && pos<8){
        v[pos++] = code_sign_bothdots();
        ++p;
    }
    while(pos < 8 && *p){
        char c = *p++;
        if(c>='0' && c<='9'){
            v[pos++] = code_digit((uint8_t)(c-'0'));
        }else if(c=='.'){
            v[pos++] = code_dot_only();    /* '.' は左ドット1桁 */
        }else{
            v[pos++] = code_blank();
        }
    }
    display8_codes_lr(v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7]);
}

/* -- 公開API -- */
void nixie_show_integer8_str(const char *s8){ nixie_show_text8_core(s8,1U); }
void nixie_show_decimal_str(const char *s8){ nixie_show_text8_core(s8,1U); }

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
    nixie_show_text8_core(buf,0U);
}
