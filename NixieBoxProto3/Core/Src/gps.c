/* ==== gps.c ========================================================== */
#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* ---- 内部バッファ：IDLE不要の1バイト再始動方式 ---- */
#ifndef GPS_RX_BUF_SZ
#define GPS_RX_BUF_SZ 256
#endif
static UART_HandleTypeDef *s_hu = NULL;
static volatile uint8_t  s_rx_byte;
static volatile uint8_t  s_ring[GPS_RX_BUF_SZ];
static volatile uint16_t s_w = 0, s_r = 0;

/* ---- 前方宣言 ---- */
static void rx_restart(void);
static int  ring_avail(void){ return (int)((uint16_t)(s_w - s_r)); }
static int  ring_get(void){ if(s_r==s_w) return -1; uint8_t b=s_ring[s_r++ & (GPS_RX_BUF_SZ-1)]; return (int)b;}

/* ---- API ---- */
void gps_init(UART_HandleTypeDef *huart)
{
    s_hu = huart;
    s_w = s_r = 0;
    rx_restart();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart==s_hu){
        s_ring[s_w++ & (GPS_RX_BUF_SZ-1)] = s_rx_byte;
        rx_restart();
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart==s_hu){ rx_restart(); }
}
static void rx_restart(void)
{
    HAL_UART_Receive_IT(s_hu, (uint8_t*)&s_rx_byte, 1);
}

/* ---- ユーティリティ：NMEAチェックサム & 角度変換 ---- */
static int nmea_ck_ok(const char *p, size_t n)
{
    /* p～'*'直前までをXOR、'*'後の2桁HEXと一致でOK */
    uint8_t sum=0; size_t i=0;
    for(; i<n && p[i] && p[i] != '*'; ++i) sum ^= (uint8_t)p[i];
    if(i>=n || p[i] != '*') return 0;
    if(i+2 >= n) return 0;
    char h1=p[i+1], h2=p[i+2];
    uint8_t chk = ((uint8_t)(isdigit(h1)?h1-'0':(toupper(h1)-'A'+10))<<4) |
                  (uint8_t)(isdigit(h2)?h2-'0':(toupper(h2)-'A'+10));
    return sum==chk;
}

static double dm_to_deg(const char *s, int lon)
{
    /* ddmm.mmmm or dddmm.mmmm */
    if(!s || !*s) return 0.0;
    double v = atof(s);
    int d = (int)(v/100.0);
    double m = v - d*100.0;
    double deg = d + m/60.0;
    (void)lon;
    return deg;
}

/* ---- 1行取り出し＆RMCだけ解析 ---- */
uint8_t gps_poll_line(gps_rmc_t *out)
{
    static char line[GPS_RX_BUF_SZ];
    static uint16_t L=0;

    /* 1行組み立て */
    while(ring_avail()>0){
        int c = ring_get();
        if(c < 0) break;
        char ch = (char)c;
        if(ch == '\r') continue;
        if(ch == '\n'){
            line[L] = '\0';
            /* $GPRMC/$GNRMCのみ扱う */
            if(L>=6 && (strncmp(line,"$GPRMC",6)==0 || strncmp(line,"$GNRMC",6)==0) && nmea_ck_ok(line, L)){
                /* フィールド分割 */
                /* 0:$G?RMC,1:hhmmss.sss,2:A/V,3:lat,4:N/S,5:lon,6:E/W,... */
                char *fld[20]={0}; int nf=0;
                for(char *p=line; *p && nf<20; ++p){
                    if(*p==',' || *p=='*'){ *p='\0'; }
                }
                /* もう一度走査してポインタ集め */
                nf=0; fld[nf++]=line;
                for(uint16_t i=1;i<L && nf<20;i++){
                    if(line[i-1]=='\0' && line[i]!='\0') fld[nf++]=&line[i];
                }
                if(nf>=7){
                    const char *t=fld[1], *st=fld[2], *lat=fld[3], *ns=fld[4], *lon=fld[5], *ew=fld[6];
                    gps_rmc_t r={0};
                    r.fix = (st && *st=='A') ? 1:0;
                    /* 時刻 */
                    if(t && strlen(t)>=6){
                        r.hh = (uint8_t)((t[0]-'0')*10 + (t[1]-'0'));
                        r.mm = (uint8_t)((t[2]-'0')*10 + (t[3]-'0'));
                        r.ss = (uint8_t)((t[4]-'0')*10 + (t[5]-'0'));
                    }
                    /* 度変換 */
                    r.lat_deg = dm_to_deg(lat, 0);
                    r.lon_deg = dm_to_deg(lon, 1);
                    if(ns && *ns=='S') r.lat_deg = -r.lat_deg;
                    if(ew && *ew=='W') r.lon_deg = -r.lon_deg;

                    if(out) *out = r;
                    L=0; return 1;
                }
            }
            L=0; /* どのみちリセット */
        }else{
            if(L < GPS_RX_BUF_SZ-1) line[L++] = ch;
            else L=0; /* オーバーフロー時は捨てて再同期 */
        }
    }
    return 0;
}
