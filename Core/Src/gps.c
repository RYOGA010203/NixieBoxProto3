#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

/* ==== グローバル定義 ================================================== */
/* UTC 時刻 */
volatile int g_UTC_hh = -1, g_UTC_mm = -1, g_UTC_ss = -1;
/* UTC 日付（RMCのddmmyy→2000+yy） */
volatile int g_UTC_YYYY = -1, g_UTC_MM = -1, g_UTC_DD = -1;

/* 現地時間（分・秒はUTC、時は経度で補正） */
volatile int g_LCL_hh = -1, g_LCL_mm = -1, g_LCL_ss = -1;
/* 現地日付（UTC日付＋時差で繰上げ/繰下げ） */
volatile int g_LCL_YYYY = -1, g_LCL_MM = -1, g_LCL_DD = -1;

/* 位置・高度・速度（float 32bit） */
volatile float g_LTT = NAN;   /* 緯度 */
volatile float g_LGT = NAN;   /* 経度 */
volatile float g_ALT = NAN;   /* 高度[m] */
volatile float g_SPD = NAN;   /* 速度[m/s] */

/* デバッグ指標 */
volatile uint32_t gps_rx_bytes = 0;
volatile uint32_t gps_rx_lines = 0;
volatile uint32_t gps_rmc_ok   = 0, gps_rmc_bad = 0;
volatile uint32_t gps_gga_ok   = 0, gps_gga_bad = 0;

volatile char     gps_last_sentence[GPS_LAST_SENTENCE_MAX] = {0};

/* ==== 受信リング（2の冪サイズ） ===================================== */
#ifndef GPS_RX_BUF_SZ
#define GPS_RX_BUF_SZ 256
#endif
#if (GPS_RX_BUF_SZ & (GPS_RX_BUF_SZ-1)) != 0
#error "GPS_RX_BUF_SZ は 2の冪(256/512/1024等)にしてください"
#endif

static UART_HandleTypeDef *s_hu = NULL;
static volatile uint8_t  s_rx_byte;
static volatile uint8_t  s_ring[GPS_RX_BUF_SZ];
static volatile uint16_t s_w = 0, s_r = 0;

/* ==== 内部プロトタイプ =============================================== */
static void   rx_restart(void);
static inline int  ring_avail(void){ return (int)((uint16_t)(s_w - s_r)); }
static inline int  ring_get(void){ if(s_r==s_w) return -1; uint8_t b=s_ring[s_r++&(GPS_RX_BUF_SZ-1)]; return (int)b; }
static int    nmea_ck_ok(const char *p, size_t n);
static int    hexval(char c);
static float  dm_to_deg(const char *s);
static int    wrap24(int h){ int x=h%24; if(x<0)x+=24; return x; }
static int    tz_from_longitude(float lon_deg);
static int    is_leap(int y);
static int    dim(int y,int m);
static void   inc_day(int *y,int *m,int *d);
static void   dec_day(int *y,int *m,int *d);

/* ==== API ============================================================ */
void gps_init(UART_HandleTypeDef *huart)
{
    s_hu = huart;
    s_w = s_r = 0;
    gps_rx_bytes = gps_rx_lines = gps_rmc_ok = gps_rmc_bad = 0;
    gps_gga_ok = gps_gga_bad = 0;
    gps_last_sentence[0] = '\0';
    rx_restart();
}

/* HALコールバック（多重定義に注意） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == s_hu){
        s_ring[s_w++ & (GPS_RX_BUF_SZ-1)] = s_rx_byte;
        gps_rx_bytes++;
        rx_restart();
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == s_hu){ rx_restart(); }
}

/* ==== 内部実装 ======================================================= */
static void rx_restart(void)
{
    (void)HAL_UART_Receive_IT(s_hu, (uint8_t*)&s_rx_byte, 1);
}

/* '$' と '*' を除外して XOR、'*'後2桁HEXと一致でOK */
static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)toupper((unsigned char)c);
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}
static int nmea_ck_ok(const char *p, size_t n)
{
    if (!p || n < 4) return 0;
    size_t i = (p[0] == '$') ? 1 : 0;
    uint8_t sum = 0; size_t k=i;
    for(; k<n && p[k] && p[k] != '*'; ++k) sum ^= (uint8_t)p[k];
    if (k>=n || p[k] != '*') return 0;
    if (k+2 >= n) return 0;
    int hi = hexval(p[k+1]), lo = hexval(p[k+2]);
    if (hi<0 || lo<0) return 0;
    return (sum == (uint8_t)((hi<<4)|lo));
}

/* ddmm.mmmm / dddmm.mmmm → 度（float） */
static float dm_to_deg(const char *s)
{
    if(!s || !*s) return NAN;
    char *endp = NULL;
    float v = strtof(s, &endp);
    if (endp == s) return NAN;
    int d = (int)(v / 100.0f);
    float m = v - d * 100.0f;
    return d + m / 60.0f;
}

/* 経度→時差(時間)。15度=1時間、中央7.5°で丸め（float版） */
static int tz_from_longitude(float lon_deg)
{
    if (isnan(lon_deg)) return 0;
    float tzf = floorf((lon_deg + 7.5f) / 15.0f);
    if (tzf < -12.0f) tzf = -12.0f;
    if (tzf >  14.0f) tzf =  14.0f;
    return (int)tzf;
}

/* 日付ユーティリティ */
static int is_leap(int y){ return ( (y%4==0 && y%100!=0) || (y%400==0) ); }
static int dim(int y,int m){
    static const int d[12]={31,28,31,30,31,30,31,31,30,31,30,31};
    if(m==2) return d[m-1] + (is_leap(y)?1:0);
    return d[m-1];
}
static void inc_day(int *y,int *m,int *d){
    if(*y<0||*m<1||*d<1) return;
    (*d)++;
    int md = dim(*y,*m);
    if(*d>md){ *d=1; (*m)++; if(*m>12){ *m=1; (*y)++; } }
}
static void dec_day(int *y,int *m,int *d){
    if(*y<0||*m<1||*d<1) return;
    (*d)--;
    if(*d<1){ (*m)--; if(*m<1){ *m=12; (*y)--; } *d = dim(*y,*m); }
}

/* ==== RMC/GNRMC と GGA を解析してグローバル更新 ===================== */
uint8_t gps_poll_line(void)
{
    static char     line[GPS_RX_BUF_SZ];
    static uint16_t L = 0;
    uint8_t updated = 0;

    while(ring_avail() > 0){
        int ci = ring_get();
        if(ci < 0) break;
        char ch = (char)ci;
        if(ch == '\r') continue;

        if(ch == '\n'){
            line[L] = '\0';
            gps_rx_lines++;
            size_t len = (size_t)L;

            /* ---- RMC ---- */
            if(L>=6 &&
               (strncmp(line,"$GPRMC",6)==0 || strncmp(line,"$GNRMC",6)==0) &&
               nmea_ck_ok(line,len))
            {
                size_t n = strlen(line);
                if(n >= GPS_LAST_SENTENCE_MAX) n = GPS_LAST_SENTENCE_MAX-1;
                for(size_t i=0;i<n;i++) gps_last_sentence[i]=line[i];
                gps_last_sentence[n]='\0';

                /* トークン化 */
                char tmp[GPS_RX_BUF_SZ];
                strncpy(tmp,line,sizeof(tmp)-1); tmp[sizeof(tmp)-1]='\0';
                char *fld[20]={0}; int nf=0; fld[nf++]=tmp;
                for(char *p=tmp; *p && nf<20; ++p){
                    if(*p==',' || *p=='*'){ *p='\0'; if(*(p+1)) fld[nf++]=p+1; }
                }
                /* RMC: 0:$G?RMC,1:time,2:A/V,3:lat,4:N/S,5:lon,6:E/W,7:knots,8:cog,9:date(ddmmyy),... */
                if(nf>=10){
                    const char *t   = fld[1]; /* hhmmss.sss */
                    const char *lat = fld[3]; const char *ns = fld[4];
                    const char *lon = fld[5]; const char *ew = fld[6];
                    const char *spk = fld[7]; /* knots */
                    const char *dmy = fld[9]; /* ddmmyy */

                    /* UTC時刻 */
                    if(t && strlen(t)>=6 && isdigit((unsigned char)t[0])){
                        g_UTC_hh = (t[0]-'0')*10 + (t[1]-'0');
                        g_UTC_mm = (t[2]-'0')*10 + (t[3]-'0');
                        g_UTC_ss = (t[4]-'0')*10 + (t[5]-'0');
                    }

                    /* UTC日付（2000+yy） */
                    if(dmy && strlen(dmy)==6 &&
                       isdigit((unsigned char)dmy[0]) && isdigit((unsigned char)dmy[1]) &&
                       isdigit((unsigned char)dmy[2]) && isdigit((unsigned char)dmy[3]) &&
                       isdigit((unsigned char)dmy[4]) && isdigit((unsigned char)dmy[5]))
                    {
                        g_UTC_DD  = (dmy[0]-'0')*10 + (dmy[1]-'0');
                        g_UTC_MM  = (dmy[2]-'0')*10 + (dmy[3]-'0');
                        int yy    = (dmy[4]-'0')*10 + (dmy[5]-'0');
                        g_UTC_YYYY = 2000 + yy;
                    }

                    /* 位置（float） */
                    float latd = dm_to_deg(lat);
                    float lond = dm_to_deg(lon);
                    if(!isnan(latd)){ if(ns && *ns=='S') latd = -latd; g_LTT = latd; }
                    if(!isnan(lond)){ if(ew && *ew=='W') lond = -lond; g_LGT = lond; }

                    /* 速度：knots→m/s（float） */
                    if(spk && *spk){
                        char *ep=NULL; float kn = strtof(spk,&ep);
                        if(ep!=spk) g_SPD = kn * 0.514444f;
                    }

                    /* === 現地時間・現地日付（UTCとUTC日付が揃っていれば） ==== */
                    if(g_UTC_hh>=0 && g_UTC_mm>=0 && g_UTC_ss>=0){
                        /* 分・秒はUTCのまま */
                        g_LCL_mm = g_UTC_mm;
                        g_LCL_ss = g_UTC_ss;

                        int tz  = tz_from_longitude(g_LGT);
                        int lhh = g_UTC_hh + tz;

                        /* UTC日付が未取得なら時だけ正規化（現地日付は据え置き） */
                        if(g_UTC_YYYY<0 || g_UTC_MM<1 || g_UTC_DD<1){
                            g_LCL_hh = wrap24(lhh);
                        }else{
                            int y=g_UTC_YYYY, m=g_UTC_MM, d=g_UTC_DD;
                            while(lhh < 0){ lhh += 24; dec_day(&y,&m,&d); }
                            while(lhh >= 24){ lhh -= 24; inc_day(&y,&m,&d); }
                            g_LCL_hh = lhh;
                            g_LCL_YYYY = y; g_LCL_MM = m; g_LCL_DD = d;  /* ← 現地日付確定 */
                        }
                    }

                    gps_rmc_ok++; updated |= 1;
                }else{
                    gps_rmc_bad++;
                }
            }
            /* ---- GGA ---- */
            else if(L>=6 &&
                    (strncmp(line,"$GPGGA",6)==0 || strncmp(line,"$GNGGA",6)==0) &&
                    nmea_ck_ok(line,len))
            {
                size_t n = strlen(line);
                if(n >= GPS_LAST_SENTENCE_MAX) n = GPS_LAST_SENTENCE_MAX-1;
                for(size_t i=0;i<n;i++) gps_last_sentence[i]=line[i];
                gps_last_sentence[n]='\0';

                char tmp[GPS_RX_BUF_SZ];
                strncpy(tmp,line,sizeof(tmp)-1); tmp[sizeof(tmp)-1]='\0';
                char *fld[20]={0}; int nf=0; fld[nf++]=tmp;
                for(char *p=tmp; *p && nf<20; ++p){
                    if(*p==',' || *p=='*'){ *p='\0'; if(*(p+1)) fld[nf++]=p+1; }
                }
                /* 9: alt(m) */
                if(nf>=11){
                    const char *alt = fld[9];
                    if(alt && *alt){
                        char *ep=NULL; float a = strtof(alt,&ep);
                        if(ep!=alt) g_ALT = a;
                    }
                    gps_gga_ok++; updated |= 2;
                }else{
                    gps_gga_bad++;
                }
            }else{
                if(L>=6 && (strncmp(line,"$GPRMC",6)==0 || strncmp(line,"$GNRMC",6)==0)) gps_rmc_bad++;
                else if(L>=6 && (strncmp(line,"$GPGGA",6)==0 || strncmp(line,"$GNGGA",6)==0)) gps_gga_bad++;
            }

            L = 0;
        }else{
            if(L < GPS_RX_BUF_SZ-1) line[L++] = ch;
            else L = 0;
        }
    }

    return updated;
}
