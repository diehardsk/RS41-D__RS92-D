
/*
 *  RS41D
 *  sync header: correlation/matched filter
 *  files: RS41D.c nav_gps_vel.c bch_ecc_mod.c bch_ecc_mod.h demod_mod.c demod_mod.h
 *  compile:
 *  (a)
 *      gcc -c demod_mod.c
 *      gcc -DINCLUDESTATIC RS41D.c demod_mod.o -lm -o RS41D
 *  (b)
 *      gcc -c demod_mod.c
 *      gcc -c bch_ecc_mod.c
 *      gcc RS41D.c demod_mod.o bch_ecc_mod.o -lm -o RS41D
 *  CYGWIN
 *      gcc -DCYGWIN RS41D.c demod_mod.o bch_ecc_mod.o -lm -O3 -o RS41D  
 *
 *  author: zilog80 code modified to RS41-D by diehardsk
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <time.h>

#ifdef CYGWIN
  #include <fcntl.h>  // cygwin: _setmode()
  #include <io.h>
#endif

// optional JSON "version"
//  (a) set global
//      gcc -DVERSION_JSN [-I<inc_dir>] ...
#ifdef VERSION_JSN
  #include "version_jsn.h"
#endif
// or
//  (b) set local compiler option, e.g.
//      gcc -DVER_JSN_STR=\"0.0.2\" ...

#include "demod_mod.h"

//#define  INCLUDESTATIC 1
#ifdef INCLUDESTATIC
    #include "bch_ecc_mod.c"
#else
    #include "bch_ecc_mod.h"
#endif


typedef struct {
    i8_t sub;  // subframe printing
    i8_t raw;  // raw frames
    i8_t ecn;  // Reed-Solomon ECC Error Count
    i8_t ptu;  // PTU: measurements
    i8_t dwp;  // PTU derived: dew point
    i8_t inv;  // inversion
    i8_t aut;
    i8_t jsn;  // JSON output
    i8_t slt;  // silent (only raw/json)    
    i8_t dbg;  // debug
    i8_t udp;  // UDP output
    i8_t arx;  // auto_rx
    i8_t dat;  // date-time
} option_t;
static option_t option;
static RS_t RS;

typedef struct {
    char *host;
    char *port;
    int   sfd;
    time_t ip_time;
} hostcfg_t;
static hostcfg_t hostcfg;

typedef struct {
    int msglen;
    int msgpos;
    int parpos;
} rscfg_t;
static rscfg_t cfg_rs41 = { 240-6-24, 6+24, 6 };
static char *key[10], *val[10];
static char extracnt;

#define BITS (1+8+1)  // 10

#define FRAMESTART  6
#define FRAME_LEN 240

#define crc_FRAME      (1<<0)
#define pck_FRAME      0x7928
#define pos_FRAME        0x1E
#define pos_FrameNb      0x20  // 2 byte
#define pos_SondeID      0x22  // 8 byte
#define pos_BattVolts    0x2A  // 2 byte
#define pos_BitField0D   0x2D  // 1 byte
#define burst_det      (1<<1)
#define pos_CalData      0x37  // 1 byte, counter 0x00..0x32
#define pos_Calfreq      0x3A  // 2 byte, calfr 0x00
#define pos_Calburst     0x43  // 1 byte, calfr 0x02
// ? #define pos_Caltimer  0x03F  // 2 byte, calfr 0x02 ?
#define pos_CalRSTyp     0x40  // 8 byte, calfr 0x21 (+2 byte in 0x22?)
        // weitere chars in calfr 0x22/0x23; weitere ID

#define crc_PTU        (1<<1)
#define pck_PTU        0x7A2A  // PTU
#define pos_PTU          0x4A


#define crc_ZERO       (1<<2)  // LEN variable
#define pck_ZERO         0x76
#define pos_ZEROstd      0x78

static char rs41d_rawheader[] = //"10010110010110010101" //DB 8N1 Manchester encoded
                                //"10010110010110010101" //DB
                                  "10010110010110010101" //DB
                                  "10010110010110010101" //DB
                                  "1001011001011001010110101001101001011001"; //DB 64

static ui8_t RS41D_header_bytes[6] = { 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0x64};

static ui8_t RS41D_padding[120] = {
0x76, 0x74, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c,
0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c,
0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c,
0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c,
0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c,
0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c,
0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c,
0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x67, 0x4a };

typedef struct {
    float frm_bytescore[FRAME_LEN+8];
    float ts;
    float last_frnb_ts;
    float last_calfrm_ts;
    ui16_t last_frnb;
    ui8_t  last_calfrm;
    int sort_idx1[FRAME_LEN]; // ui8_t[] sort_cw1_idx
    int sort_idx2[FRAME_LEN]; // ui8_t[] sort_cw2_idx
} ecdat_t;

typedef struct {
    int out;
    int frnr;
    char id[9];
    float T; float RH; float TH;
    float P;
    i32_t alt;
    ui32_t crc;
    ui8_t frame[FRAME_LEN];
    ui8_t calibytes[51*16];
    ui8_t calfrchk[51];
    float ptu_Rf1;      // ref-resistor f1 (750 Ohm)
    float ptu_Rf2;      // ref-resistor f2 (1100 Ohm)
    float ptu_co1[3];   // { -243.911 , 0.187654 , 8.2e-06 }
    float ptu_calT1[3]; // calibration T1
    float ptu_co2[3];   // { -243.911 , 0.187654 , 8.2e-06 }
    float ptu_calT2[3]; // calibration T2-Hum
    float ptu_calH[2];  // calibration Hum
    float ptu_mtxH[42];
    float ptu_corHp[3];
    float ptu_corHt[12];
    float ptu_Cf1;
    float ptu_Cf2;
    float ptu_calP[25];
    ui32_t freq;    // freq/kHz (RS41)
    int jsn_freq;   // freq/kHz (SDR)
    float batt;     // battery voltage (V)
    ui16_t conf_fw; // firmware
    i16_t conf_kt; // kill timer (sec)
    i16_t conf_bt; // burst timer (sec)
    i16_t conf_cd; // kill countdown (sec) (kt or bt)
    ui8_t  conf_bk; // burst kill
    ui8_t  burst;   // burst detected (by sonde)
    char rstyp[9];  // RS41-SG, RS41-SGP
    ecdat_t ecdat;
} gpx_t;
static gpx_t *gpx, *gpx1_pointer, *gpx2_pointer;

/* ------------------------------------------------------------------------------------ */

#define BIT_RATE 4800

/* ------------------------------------------------------------------------------------ */

// manchester1 1->10,0->01: 1.bit
// manchester2 0->10,1->01: 2.bit
// 8N1 manchester2
static int bits2byte(char bits[]) {
    int i, byteval=0, d=1;
    for (i = 1; i <= 8; i++) {   // little endian
    /* for (i = 8; i > 1; i--) { // big endian */
        if      (bits[i] == 1)  byteval += d;
        else if (bits[i] == 0)  byteval += 0;
        d <<= 1;
    }
    return byteval;
}

/* ------------------------------------------------------------------------------------ */

static ui32_t u4(ui8_t *bytes) {  // 32bit unsigned int
    ui32_t val = 0;
    memcpy(&val, bytes, 4);
    return val;
}

static ui32_t u3(ui8_t *bytes) {  // 24bit unsigned int
    int val24 = 0;
    val24 = bytes[0] | (bytes[1]<<8) | (bytes[2]<<16);
    // = memcpy(&val, bytes, 3), val &= 0x00FFFFFF;
    return val24;
}

static int i3(ui8_t *bytes) {  // 24bit signed int
    int val = 0,
        val24 = 0;
    val = bytes[0] | (bytes[1]<<8) | (bytes[2]<<16);
    val24 = val & 0xFFFFFF; if (val24 & 0x800000) val24 = val24 - 0x1000000;
    return val24;
}

static ui32_t u2(ui8_t *bytes) {  // 16bit unsigned int
    return  bytes[0] | (bytes[1]<<8);
}

static int i2(ui8_t *bytes) { // 16bit signed int
    //return (i16_t)u2(bytes);
    int val = bytes[0] | (bytes[1]<<8);
    if (val & 0x8000) val -= 0x10000;
    return val;
}

static int send_UDP(char *message) {
    struct addrinfo hints;
    struct addrinfo *result, *rp;
    int s;
    size_t len;
    double elap_minutes;
    time_t time_now = time(NULL);

    elap_minutes = difftime(time_now, hostcfg.ip_time)/60;
    if (hostcfg.sfd == -1 || elap_minutes > 60) {  //max once per hour
        fprintf(stderr, "resolving IP.. ");
        if (hostcfg.sfd != -1)
             close(hostcfg.sfd); 
        memset(&hints, 0, sizeof(struct addrinfo));
        hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
        hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
        hints.ai_flags = 0;
        hints.ai_protocol = 0;          /* Any protocol */

        s = getaddrinfo(hostcfg.host, hostcfg.port, &hints, &result);
        if (s != 0) {
            fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
            freeaddrinfo(result);
            return -1;
        }
        for (rp = result; rp != NULL; rp = rp->ai_next) {
            hostcfg.sfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
            if (hostcfg.sfd == -1)
                continue;
            if (connect(hostcfg.sfd, rp->ai_addr, rp->ai_addrlen) != -1)
                break;                  /* Success */
            close(hostcfg.sfd);
            hostcfg.sfd = -1;
        }
        if (rp == NULL) {               /* No address succeeded */
            fprintf(stderr, "could not connect to host\n");
            freeaddrinfo(result);
            return -2;
        }
        freeaddrinfo(result);
        hostcfg.ip_time = time(NULL);
        fprintf(stderr, "success\n");
    }
        
    len = strlen(message);
    if (write(hostcfg.sfd, message, len) != len) {
        fprintf(stderr, "Unable to send UDP message\n");
        return -3;
    }
}    

static int crc16(int start, int len) {
    int crc16poly = 0x1021;
    int rem = 0xFFFF, i, j;
    int byte;

    if (start+len+2 > FRAME_LEN) return -1;

    for (i = 0; i < len; i++) {
        byte = gpx->frame[start+i];
        rem = rem ^ (byte << 8);
        for (j = 0; j < 8; j++) {
            if (rem & 0x8000) {
                rem = (rem << 1) ^ crc16poly;
            }
            else {
                rem = (rem << 1);
            }
            rem &= 0xFFFF;
        }
    }
    return rem;
}

static int check_CRC(ui32_t pos, ui32_t pck) {
    ui32_t crclen = 0,
           crcdat = 0;
    if (((pck>>8) & 0xFF) != gpx->frame[pos]) return -1;
    crclen = gpx->frame[pos+1];
    if (pos + crclen + 4 > FRAME_LEN) return -1;
    crcdat = u2(gpx->frame+pos+2+crclen);
    if ( crcdat != crc16(pos+2, crclen) ) {
        return 1;  // CRC NO
    }
    else return 0; // CRC OK
}


static int get_FrameNb(int crc, int ofs) {
    int i;
    unsigned byte;
    ui8_t frnr_bytes[2];
    int frnr;

    for (i = 0; i < 2; i++) {
        byte = gpx->frame[pos_FrameNb+ofs + i];
        frnr_bytes[i] = byte;
    }

    frnr = frnr_bytes[0] + (frnr_bytes[1] << 8);
    gpx->frnr = frnr;

    // crc check
    if (crc == 0) {
        gpx->ecdat.last_frnb = frnr;
        gpx->ecdat.last_frnb_ts = gpx->ecdat.ts;
    }
    return 0;
}

static int get_BattVolts(int ofs) {
    int i;
    unsigned byte;
    ui8_t batt_bytes[2];
    ui16_t batt_volts; // signed voltage?

    for (i = 0; i < 2; i++) {
        byte = gpx->frame[pos_BattVolts+ofs + i];
        batt_bytes[i] = byte;
    }
                                // 2 bytes? V > 25.5 ?
    batt_volts = batt_bytes[0]; // + (batt_bytes[1] << 8);
    gpx->batt = batt_volts/10.0;

    return 0;
}


static int get_SondeID(int crc, int ofs) {
    int i;
    unsigned byte;
    char sondeid_bytes[9];

    if (crc == 0) {
        for (i = 0; i < 8; i++) {
            byte = gpx->frame[pos_SondeID+ofs + i];
            //if ((byte < 0x20) || (byte > 0x7E)) return -1;
            sondeid_bytes[i] = byte;
        }
        sondeid_bytes[8] = '\0';
        if ( strncmp(gpx->id, sondeid_bytes, 8) != 0 ) { //id doesn't match
            //let's swap pointer between gpx1 <-> gpx2 datasets
            gpx_t *original_pointer = gpx;   
            if (gpx == gpx1_pointer)
                gpx = gpx2_pointer;
            else gpx = gpx1_pointer;
            //copy new data frame
            memcpy(gpx->frame, original_pointer->frame, FRAME_LEN);

            if ( strncmp(gpx->id, sondeid_bytes, 8) != 0 ) { //ID doesn't match the other dataset
                                                               //reset data (of not recently used dataset)
                memset(gpx->calfrchk, 0, 51); // 0x00..0x32
                // reset conf data
                memset(gpx->rstyp, 0, 9);
                gpx->freq = 0;
                gpx->conf_fw = 0;
                gpx->conf_bt = -1;
                gpx->conf_bk = 0;
                gpx->conf_cd = -1;
                gpx->conf_kt = -1;
                gpx->burst = -1;
                // don't reset gpx->frame[] !
                gpx->T = -273.15f;
                gpx->RH = -1.0f;
                gpx->P = -1.0f;
                gpx->alt  = -10000;
                // new ID:
                memcpy(gpx->id, sondeid_bytes, 8);
                gpx->id[8] = '\0';

                gpx->ecdat.last_frnb = 0;
            }
        }
    }

    return 0;
}

static int get_FrameConf(int ofs) {
    int crc, err;
    ui8_t calfr;
    int i;

    crc = check_CRC(pos_FRAME+ofs, pck_FRAME);
    if (crc) gpx->crc |= crc_FRAME;

    err = crc;
    err |= get_SondeID(crc, ofs);
    err |= get_FrameNb(crc, ofs);
    err |= get_BattVolts(ofs);
    if (gpx->frame[pos_BitField0D+ofs] & burst_det)
        gpx->burst = 1;
    else gpx->burst = 0; 

    if (crc == 0) {
        calfr = gpx->frame[pos_CalData+ofs];
        if (gpx->calfrchk[calfr] == 0) // const?
        {                              // 0x32 not constant
            for (i = 0; i < 16; i++) {
                gpx->calibytes[calfr*16 + i] = gpx->frame[pos_CalData+ofs+1+i];
            }
            gpx->calfrchk[calfr] = 1;
        }
        gpx->ecdat.last_calfrm = calfr;
        gpx->ecdat.last_calfrm_ts = gpx->ecdat.ts;
    }
    return err;
}



// ----------------------------------------------------------------------------------------------------

static int get_CalData() {

    int j;

    memcpy(&(gpx->ptu_Rf1), gpx->calibytes+61, 4);  // 0x03*0x10+13
    memcpy(&(gpx->ptu_Rf2), gpx->calibytes+65, 4);  // 0x04*0x10+ 1

    memcpy(gpx->ptu_co1+0, gpx->calibytes+77, 4);  // 0x04*0x10+13
    memcpy(gpx->ptu_co1+1, gpx->calibytes+81, 4);  // 0x05*0x10+ 1
    memcpy(gpx->ptu_co1+2, gpx->calibytes+85, 4);  // 0x05*0x10+ 5

    memcpy(gpx->ptu_calT1+0, gpx->calibytes+89, 4);  // 0x05*0x10+ 9
    memcpy(gpx->ptu_calT1+1, gpx->calibytes+93, 4);  // 0x05*0x10+13
    memcpy(gpx->ptu_calT1+2, gpx->calibytes+97, 4);  // 0x06*0x10+ 1
    // ptu_calT1[3..6]

    memcpy(gpx->ptu_calH+0, gpx->calibytes+117, 4);  // 0x07*0x10+ 5
    memcpy(gpx->ptu_calH+1, gpx->calibytes+121, 4);  // 0x07*0x10+ 9

    memcpy(gpx->ptu_co2+0, gpx->calibytes+293, 4);  // 0x12*0x10+ 5
    memcpy(gpx->ptu_co2+1, gpx->calibytes+297, 4);  // 0x12*0x10+ 9
    memcpy(gpx->ptu_co2+2, gpx->calibytes+301, 4);  // 0x12*0x10+13

    memcpy(gpx->ptu_calT2+0, gpx->calibytes+305, 4);  // 0x13*0x10+ 1
    memcpy(gpx->ptu_calT2+1, gpx->calibytes+309, 4);  // 0x13*0x10+ 5
    memcpy(gpx->ptu_calT2+2, gpx->calibytes+313, 4);  // 0x13*0x10+ 9
    // ptu_calT2[3..6]


    // cf. DF9DQ
    memcpy(&(gpx->ptu_Cf1), gpx->calibytes+69, 4);  // 0x04*0x10+ 5
    memcpy(&(gpx->ptu_Cf2), gpx->calibytes+73, 4);  // 0x04*0x10+ 9
    for (j = 0; j < 42; j++) {  // 0x07*0x10+13 = 0x07D = 125
        memcpy(gpx->ptu_mtxH+j, gpx->calibytes+125+4*j, 4);
    }
    for (j = 0; j <  3; j++) {  // 0x2A*0x10+6 = 0x2A6 = 678
        memcpy(gpx->ptu_corHp+j, gpx->calibytes+678+4*j, 4);
    }
    for (j = 0; j < 12; j++) {  // 0x2B*0x10+A = 0x2BA = 698
        memcpy(gpx->ptu_corHt+j, gpx->calibytes+698+4*j, 4);
    }
    // cf. DF9DQ or stsst/RS-fork
    memcpy(gpx->ptu_calP+0,  gpx->calibytes+606, 4); // 0x25*0x10+14 = 0x25E
    memcpy(gpx->ptu_calP+4,  gpx->calibytes+610, 4); // ..
    memcpy(gpx->ptu_calP+8,  gpx->calibytes+614, 4);
    memcpy(gpx->ptu_calP+12, gpx->calibytes+618, 4);
    memcpy(gpx->ptu_calP+16, gpx->calibytes+622, 4);
    memcpy(gpx->ptu_calP+20, gpx->calibytes+626, 4);
    memcpy(gpx->ptu_calP+24, gpx->calibytes+630, 4);
    memcpy(gpx->ptu_calP+1,  gpx->calibytes+634, 4);
    memcpy(gpx->ptu_calP+5,  gpx->calibytes+638, 4);
    memcpy(gpx->ptu_calP+9,  gpx->calibytes+642, 4);
    memcpy(gpx->ptu_calP+13, gpx->calibytes+646, 4);
    memcpy(gpx->ptu_calP+2,  gpx->calibytes+650, 4);
    memcpy(gpx->ptu_calP+6,  gpx->calibytes+654, 4);
    memcpy(gpx->ptu_calP+10, gpx->calibytes+658, 4);
    memcpy(gpx->ptu_calP+14, gpx->calibytes+662, 4);
    memcpy(gpx->ptu_calP+3,  gpx->calibytes+666, 4);
    memcpy(gpx->ptu_calP+7,  gpx->calibytes+670, 4); // ..
    memcpy(gpx->ptu_calP+11, gpx->calibytes+674, 4); // 0x2A*0x10+ 2

    return 0;
}

// temperature, platinum resistor
// T-sensor:    gpx->ptu_co1 , gpx->ptu_calT1
// T_RH-sensor: gpx->ptu_co2 , gpx->ptu_calT2
static float get_T(ui32_t f, ui32_t f1, ui32_t f2, float *ptu_co, float *ptu_calT) {
    float *p = ptu_co;
    float *c = ptu_calT;
    float  g = (float)(f2-f1)/(gpx->ptu_Rf2-gpx->ptu_Rf1),       // gain
          Rb = (f1*gpx->ptu_Rf2-f2*gpx->ptu_Rf1)/(float)(f2-f1), // ofs
          Rc = f/g - Rb,
          R = Rc * c[0],
          T = (p[0] + p[1]*R + p[2]*R*R + c[1])*(1.0 + c[2]);
    return T; // [Celsius]
}

// rel.hum., capacitor
// (data:) ftp://ftp-cdc.dwd.de/climate_environment/CDC/observations_germany/radiosondes/
// (diffAlt: Ellipsoid-Geoid)
// (note: humidity sensor has significant time-lag at low temperatures)
static float get_RHemp(ui32_t f, ui32_t f1, ui32_t f2, float T) {
    float a0 = 7.5;                    // empirical
    float a1 = 350.0/gpx->ptu_calH[0]; // empirical
    float fh = (f-f1)/(float)(f2-f1);
    float rh = 100.0 * ( a1*fh - a0 );
    float T0 = 0.0, T1 = -20.0, T2 = -40.0; // T/C    v0.4
    rh += T0 - T/5.5;                       // empir. temperature compensation
    if (T < T1) rh *= 1.0 + (T1-T)/100.0;   // empir. temperature compensation
    if (T < T2) rh *= 1.0 + (T2-T)/120.0;   // empir. temperature compensation
    if (rh < 0.0) rh = 0.0;
    if (rh > 100.0) rh = 100.0;
    if (T < -273.0) rh = -1.0;
    return rh;
}


//
// cf. github DF9DQ or stsst/RS-fork
static float get_P(ui32_t f, ui32_t f1, ui32_t f2, int fx)
{
    double p = 0.0;
    double a0, a1, a0j, a1k;
    int j, k;
    if (f1 == f2 || f1 == f) return 0.0;
    a0 = gpx->ptu_calP[24] / ((float)(f - f1) / (float)(f2 - f1));
    a1 = fx * 0.01;

    a0j = 1.0;
    for (j = 0; j < 6; j++) {
        a1k = 1.0;
        for (k = 0; k < 4; k++) {
            p += a0j * a1k * gpx->ptu_calP[j*4+k];
            a1k *= a1;
        }
        a0j *= a0;
    }
    return (float)p;
}
// ---------------------------------------------------------------------------------------
//
// Altitude calculation using barometric formula for standard atmosphere
static int calc_PAlt() {
    float Pb, Tb, Lb, hb, RgM = 8.31446/(9.80665*0.0289644);
    float pressure = gpx->P;  //hPa

    if (pressure <= 0.0f) return -1;
    if (pressure < 8.6802) {
        Pb = 8.6802;
        Tb = 228.65;
        Lb = 0.0028;
        hb = 32000.0;
    }
    else if (pressure < 54.7489) {
        Pb = 54.7489;
        Tb = 216.65;
        Lb = 0.001;
        hb = 20000.0;
    }
    else if (pressure < 226.321) {
        Pb = 226.321;
        Tb = 216.65;
        Lb = 0.0;
        hb = 11000.0;
    }
    else {                 
        Pb = 1013.25;
        Tb = 288.15;
        Lb = -0.0065;
        hb = 0.0;
    }

    if (Lb == 0.0) gpx->alt = round(hb - RgM*Tb*log(pressure/Pb));
    else           gpx->alt = round(hb + Tb/Lb*(pow(pressure/Pb, -RgM*Lb)-1.0));
    return 0;
}


static int get_PTU(int ofs, int pck) {
    int err=0, i;
    int bR, bc1, bT1,
            bc2, bT2;
    int bH;
    int bP;
    ui32_t meas[12];
    float Tc = -273.15;
    float TH = -273.15;
    float RH = -1.0;
    float P = -1.0;

    get_CalData();

    err = check_CRC(pos_PTU+ofs, pck);
    if (err) gpx->crc |= crc_PTU;

    if (err == 0)
    {
        // 0x7A2A: 16 byte (P)TU
        // 0x7F1B: 12 byte _TU
        for (i = 0; i < 12; i++) {
            meas[i] = u3(gpx->frame+pos_PTU+ofs+2+3*i);
        }

        bR  = gpx->calfrchk[0x03] && gpx->calfrchk[0x04];
        bc1 = gpx->calfrchk[0x04] && gpx->calfrchk[0x05];
        bT1 = gpx->calfrchk[0x05] && gpx->calfrchk[0x06];
        bc2 = gpx->calfrchk[0x12] && gpx->calfrchk[0x13];
        bT2 = gpx->calfrchk[0x13];
        bH  = gpx->calfrchk[0x07];

        bP  = gpx->calfrchk[0x21] && gpx->calibytes[0x21F] == 'P'
           && gpx->calfrchk[0x25] && gpx->calfrchk[0x26]
           && gpx->calfrchk[0x27] && gpx->calfrchk[0x28]
           && gpx->calfrchk[0x29] && gpx->calfrchk[0x2A];

        if (bR && bc1 && bT1) {
            Tc = get_T(meas[0], meas[1], meas[2], gpx->ptu_co1, gpx->ptu_calT1);
        }
        gpx->T = Tc;

        if (bR && bc2 && bT2) {
            TH = get_T(meas[6], meas[7], meas[8], gpx->ptu_co2, gpx->ptu_calT2);
        }
        gpx->TH = TH;

        if (bH && Tc > -273.0) {
            RH = get_RHemp(meas[3], meas[4], meas[5], Tc); // TH, TH-Tc (sensorT - T)
        }
        gpx->RH = RH;

        // cf. DF9DQ, stsst/RS-fork
        if (bP) {
            P = get_P(meas[9], meas[10], meas[11], i2(gpx->frame+pos_PTU+ofs+2+38));
        }
        gpx->P = P;

        calc_PAlt();

        if (option.dbg == 1 && (gpx->crc & crc_PTU)==0)
        {
            printf("1: %8d %8d %8d", meas[0], meas[1], meas[2]);
            printf("   #   ");
            printf("2: %8d %8d %8d", meas[3], meas[4], meas[5]);
            printf("   #   ");
            printf("3: %8d %8d %8d", meas[6], meas[7], meas[8]);
            printf("   #   ");

            if (0 && Tc > -273.0 && RH > -0.5)
            {
                printf("  ");
                printf(" Tc:%.2f ", Tc);
                printf(" RH:%.1f ", RH);
                printf(" TH:%.2f ", TH);
            }
            printf("\n");
            printf("    %6.1f ; %6.1f ", gpx->ptu_Rf1, gpx->ptu_Rf2);
            printf("; %10.6f ; %10.6f ; %10.6f ", gpx->ptu_calT1[0], gpx->ptu_calT1[1], gpx->ptu_calT1[2]);
            //printf(";  %8d ; %8d ; %8d ", meas[0], meas[1], meas[2]);
            printf("; %10.6f ; %10.6f ", gpx->ptu_calH[0], gpx->ptu_calH[1]);
            //printf(";  %8d ; %8d ; %8d ", meas[3], meas[4], meas[5]);
            printf("; %10.6f ; %10.6f ; %10.6f ", gpx->ptu_calT2[0], gpx->ptu_calT2[1], gpx->ptu_calT2[2]);
            //printf(";  %8d ; %8d ; %8d" , meas[6], meas[7], meas[8]);
            printf("\n");
        }
    }
    return err;
}


static int get_Calconf(int out, int ofs) {
    int i;
    unsigned byte;
    ui8_t calfr = 0;
    ui16_t fw = 0;
    ui32_t freq = 0;
    char sondetyp[9];
    int err = 0;

    byte = gpx->frame[pos_CalData+ofs];
    calfr = byte;
    err = check_CRC(pos_FRAME+ofs, pck_FRAME);

    if (option.sub == 1) {
        fprintf(stdout, "\n");  // fflush(stdout);
        fprintf(stdout, "[%5d] ", gpx->frnr);
        fprintf(stdout, " 0x%02x: ", calfr);
        for (i = 0; i < 16; i++) {
            byte = gpx->frame[pos_CalData+ofs+1+i];
            fprintf(stdout, "%02x ", byte);
        }
        fprintf(stdout, " ");
    }

    if (err == 0)
    {
        if (calfr == 0x00) {
            ui32_t f_plus = gpx->frame[pos_Calfreq+ofs] + (gpx->frame[pos_Calfreq+ofs+1] << 8);
            freq = 1660000 + f_plus;
            if (out) fprintf(stdout, ": fq %d ", freq);
            gpx->freq = freq;
        }

        if (calfr == 0x01) {
            fw = gpx->frame[pos_CalData+ofs+6] | (gpx->frame[pos_CalData+ofs+7]<<8);
            if (out) fprintf(stdout, ": fw 0x%04x ", fw);
            gpx->conf_fw = fw;
        }

        if (calfr == 0x02) {    // 0x5E, 0x5A..0x5B
            ui8_t  bk = gpx->frame[pos_Calburst+ofs];  // fw >= 0x4ef5, burst-killtimer in 0x31 relevant
            ui16_t kt = gpx->frame[pos_CalData+ofs+8] + (gpx->frame[pos_CalData+ofs+9] << 8); // killtimer (short?)
            if (out) fprintf(stdout, ": BK %02X ", bk);
            if (out && kt != 0xFFFF ) fprintf(stdout, ": kt %.1fmin ", kt/60.0);
            gpx->conf_bk = bk;
            gpx->conf_kt = kt;
        }

        if (calfr == 0x31) {    // 0x59..0x5A
            ui16_t bt = gpx->frame[pos_CalData+ofs+7] + (gpx->frame[pos_CalData+ofs+8] << 8); // burst timer (short?)
            // fw >= 0x4ef5: default=[88 77]=0x7788sec=510min
            if (out && bt != 0x0000 && (option.dbg == 1  ||  gpx->conf_bk))
                fprintf(stdout, ": bt %.1fmin ", bt/60.0);
            gpx->conf_bt = bt;
        }

        if (calfr == 0x32) {
            ui16_t cd = gpx->frame[pos_CalData+ofs+1] + (gpx->frame[pos_CalData+ofs+2] << 8); // countdown (bt or kt) (short?)
            if (out && cd != 0xFFFF &&
                    (option.dbg == 1  ||  gpx->conf_bk || gpx->conf_kt != 0xFFFF)
               ) fprintf(stdout, ": cd %.1fmin ", cd/60.0);
            gpx->conf_cd = cd;  // (short/i16_t) ?
        }

        if (calfr == 0x22) {
            char mbver[10];
            for (i = 0; i < 9; i++) {
                byte = gpx->frame[pos_CalData+ofs + i + 3];
                if (((byte >= 0x20) && (byte < 0x7F)) || (byte == 0x00)) mbver[i] = byte;
            }
            mbver[9] = '\0';
            if (out) fprintf(stdout, ": %s ", mbver);
        }

        if (calfr == 0x21) {  // ... eventuell noch 2 bytes in 0x22
            for (i = 0; i < 9; i++) sondetyp[i] = 0;
            for (i = 0; i < 8; i++) {
                byte = gpx->frame[pos_CalRSTyp+ofs + i];
                if ((byte >= 0x20) && (byte < 0x7F)) sondetyp[i] = byte;
                else if (byte == 0x00) sondetyp[i] = '\0';
            }
            if (out) fprintf(stdout, ": %s ", sondetyp);
            strcpy(gpx->rstyp, sondetyp);
            if (out && option.dbg == 1) { // Stationsdruck QFE
                float qfe1 = 0.0, qfe2 = 0.0;
                memcpy(&qfe1, gpx->frame+pos_CalData+1, 4);
                memcpy(&qfe2, gpx->frame+pos_CalData+5, 4);
                if (qfe1 > 0.0 || qfe2 > 0.0) {
                    fprintf(stdout, " ");
                    if (qfe1 > 0.0) fprintf(stdout, "QFE1:%.1fhPa ", qfe1);
                    if (qfe2 > 0.0) fprintf(stdout, "QFE2:%.1fhPa ", qfe2);
                }
            }
        }
    }
    return 0;
}

/* ------------------------------------------------------------------------------------ */

#define rs_N 255
#define rs_R 24
#define rs_K (rs_N-rs_R)

static int rs41_ecc(int msglen) {

    int i;
    int errors;
    ui8_t cw[rs_N];
    ui8_t err_pos[rs_R], err_val[rs_R];

    memset(cw, 0, rs_N);

    if (msglen > FRAME_LEN) msglen = FRAME_LEN;
    for (i = msglen; i < FRAME_LEN; i++) gpx->frame[i] = 0;//xFF;


    for (i = 0; i < rs_R;            i++) cw[i]      = gpx->frame[cfg_rs41.parpos+i];
    for (i = 0; i < cfg_rs41.msglen; i++) cw[rs_R+i] = gpx->frame[cfg_rs41.msgpos+i];

    errors = rs_decode(&RS, cw, err_pos, err_val);

    for (i = 0; i < rs_R;            i++) gpx->frame[cfg_rs41.parpos+i] = cw[i];
    for (i = 0; i < cfg_rs41.msglen; i++) gpx->frame[cfg_rs41.msgpos+i] = cw[rs_R+i];

    return errors;
}

/* ------------------------------------------------------------------------------------ */

static int prn_frm() {
    fprintf(stdout, "[%5d] ", gpx->frnr);
    fprintf(stdout, "(%s) ", gpx->id);
    if (option.dbg == 1) fprintf(stdout, "(%.1f V) ", gpx->batt);
    fprintf(stdout, " ");
    return 0;
}

static int prn_ptu() {
    fprintf(stdout, " ");
    if (gpx->T > -273.0) fprintf(stdout, " T=%.1fC ", gpx->T);
    if (gpx->RH > -0.5)  fprintf(stdout, " _RH=%.0f%% ", gpx->RH);
    if (gpx->P > 0.0) {
        if (gpx->P < 100.0) fprintf(stdout, " P=%.2fhPa ", gpx->P);
        else                fprintf(stdout, " P=%.1fhPa ", gpx->P);
    if (gpx->alt > -10000)  fprintf(stdout, " alt=%dm ", gpx->alt);
    }

    // dew point
    if (option.dwp)
    {
        float rh = gpx->RH;
        float Td = -273.15f; // dew point Td
        if (rh > 0.0f && gpx->T > -273.0f) {
            float gamma = logf(rh / 100.0f) + (17.625f * gpx->T / (243.04f + gpx->T));
            Td = 243.04f * gamma / (17.625f - gamma);
            fprintf(stdout, " Td=%.1fC ", Td);
        }
    }
    return 0;
}


static int print_position(int ec) {
    int i;
    int err = 0, err0 = 0;
    int out;
    int ofs_ptu = 0, pck_ptu = 0;
    if (option.slt || option.jsn==2) out = 0; else out = 1;

    if ( ec >= 0 )
    {
        int pos, blk, len, crc, pck;
        int flen = FRAME_LEN;

        int ofs_cal = 0;
        int frm_end = FRAME_LEN-2;

        pos = pos_FRAME;
        gpx->crc = 0;

        while (pos < flen-1) {
            blk = gpx->frame[pos];
            len = gpx->frame[pos+1];
            crc = check_CRC(pos, blk<<8);
            pck = (blk<<8) | len;

            if ( crc == 0 )  // ecc-OK, crc-OK
            {
                int ofs = 0;
                switch (pck)
                {
                    case pck_FRAME: // 0x7928
                            ofs = pos - pos_FRAME;
                            ofs_cal = ofs;
                            err = get_FrameConf(ofs);
                            if ( !err ) {
                                if (out) prn_frm();
                            }
                            break;

                    case pck_PTU: // 0x7A2A
                            ofs_ptu = pos - pos_PTU;
                            pck_ptu = pck_PTU;
                            break;

                    default:
                            if (blk == 0x76) {
                                // ZERO-Padding pck
                            }
                            else {
                                if (out) fprintf(stdout, " [%04X] ", pck);
                            }
                }
            }
            else { // CRC-ERROR (ECC-OK)
                if (out) fprintf(stdout, " [ERROR]\n");
                break;
            }

            pos += 2+len+2; // next pck

            if ( pos > frm_end )  // end of (sub)frame
            {
                if (option.ptu && pck_ptu > 0) {
                    err0 = get_PTU(ofs_ptu, pck_ptu);
                    if (!err0 && out) prn_ptu();
                }

                get_Calconf(out, ofs_cal);

                if (out && option.ecn && ec > 0 && pos > flen-1) fprintf(stdout, " (%d)", ec);

                gpx->crc = 0;
                frm_end = FRAME_LEN-2;

                if (out) fprintf(stdout, "\n");

                if (option.jsn || option.udp) {
                    // Print out telemetry data as JSON
                    if (!err && !err0) {
                        char str[200], sentence[1000];
                        char *ver_jsn = NULL;
                        sprintf(sentence, "{\"type\":\"RS41\", \"subtype\":\"RS41-D\"");
                        sprintf(str, ", \"frame\":%d, \"id\":\"%s\", \"batt\":%.2f, \"burst\":%d",
                                       gpx->frnr, gpx->id, gpx->batt, gpx->burst);
                        strcat(sentence, str);
                        if (gpx->conf_bt >= 0) {
                            sprintf(str, ", \"bt\":%d",  gpx->conf_bt );
                            strcat(sentence, str);
                        }
                        if (gpx->conf_cd >= 0) {
                            sprintf(str, ", \"cd\":%d",  gpx->conf_cd );
                            strcat(sentence, str);
                        }
                        if (option.ptu) {
                            float _RH = gpx->RH;
                            if (gpx->T > -273.0) {
                                sprintf(str, ", \"temp\":%.1f",  gpx->T );
                                strcat(sentence, str);
                            }
                            if (_RH > -0.5) {
                                sprintf(str, ", \"humidity\":%.1f",  _RH );
                                strcat(sentence, str);
                            }
                            if (gpx->P > 0.0) {
                                sprintf(str, ", \"pressure\":%.2f",  gpx->P );
                                strcat(sentence, str);
                            }
                            if (gpx->alt >-10000 || option.arx) {
                                sprintf(str, ", \"alt\":%d", gpx->alt);
                                strcat(sentence, str);
                            }                            
                        }
                        if (gpx->jsn_freq > 0) {  // rs41-frequency: gpx->freq
                            int fq_kHz = gpx->jsn_freq;
                            if (gpx->freq > 0) fq_kHz = gpx->freq;
                            sprintf(str, ", \"freq\":%d", fq_kHz);
                            strcat(sentence, str);
                        }

                        // Include frequency derived from subframe information if available.
                        if (gpx->freq > 0) {
                            sprintf(str, ", \"tx_frequency\":%d", gpx->freq );
                            strcat(sentence, str);
                        }
                        if (option.arx || option.dat) {
                            time_t now;
                            now = time(NULL);
                            char dt_now[sizeof "2023-01-01T07:35:00Z"];
                            strftime(dt_now, sizeof dt_now, "%FT%TZ", gmtime(&now));
                            sprintf(str, ", \"datetime\":\"%s\"", dt_now );
                            strcat(sentence, str);
                        }
                        #ifdef VER_JSN_STR
                            ver_jsn = VER_JSN_STR;
                        #endif
                        if (ver_jsn && *ver_jsn != '\0') {
                            sprintf(str, ", \"version\":\"%s\"", ver_jsn);
                            strcat(sentence, str);
                        }    
                        for (int i = 0; i < extracnt; i++) {
                            if ((val[i][0]>='0' && val[i][0]<='9') || val[i][0]=='-' || val[i][0]=='+') //number?
                                sprintf(str, ", \"%s\":%s", key[i], val[i]);
                            else
                                sprintf(str, ", \"%s\":\"%s\"", key[i], val[i]);
                            strcat(sentence, str);
                        }
                        strcat(sentence, "}");
                        if (!option.slt && option.jsn) {
                            fprintf(stdout, "%s", sentence); //JSON string to sdout
                            fprintf(stdout, "\n");
                        }
                        if (!option.slt && option.jsn==1) fprintf(stdout, "\n");
                        if (option.udp)      //UDP packet
                            send_UDP(sentence);
                    }
                }
            }
        }
    }

    if (ec < 0 && (out)) {
        int pck;
        int output = 0;

        gpx->crc = 0;

        err = get_FrameConf(0);
        if (out && !err) {
            prn_frm();
            output = 1;
        }

        pck = (gpx->frame[pos_PTU]<<8) | gpx->frame[pos_PTU+1];

        if (pck < 0x8000) {

            err0 = get_PTU(0, pck);

            if (out) {

                if (!err0 && option.ptu) {
                    prn_ptu();
                    output = 1;
                }

                if (option.dbg && output) {
                    fprintf(stdout, " ");
                    fprintf(stdout, "[");
                    for (i=0; i<2; i++) fprintf(stdout, "%d", (gpx->crc>>i)&1);
                    fprintf(stdout, "]");
                }
            }
        }

        if (out && output)
        {
            if (option.ecn) {
                if      (ec == -1)  fprintf(stdout, " (-+)");
                else if (ec == -2)  fprintf(stdout, " (+-)");
                else   /*ec == -3*/ fprintf(stdout, " (--)");
            }
            fprintf(stdout, "\n");  // fflush(stdout);
        }
    }
    return  0;
}

static void print_frame(int len) {
    int i, ec = 0;
    float max_minscore = 0.0;

    gpx->crc = 0;

    // len < FRAME_LEN: EOF
    if (len < pos_ZEROstd) {
        for (i = len; i < FRAME_LEN; i++) gpx->frame[i] = 0;
    }

    //we can repair fix data in unused frame space before running ecc
    if (gpx->frame[pos_ZEROstd] == pck_ZERO)     
        for (i = 0; i < sizeof(RS41D_padding); i++) {
            gpx->frame[i + pos_ZEROstd] = RS41D_padding[i];
        }

    len = FRAME_LEN;

    for (i = FRAMESTART; i < len; i++) {
        if (fabs(gpx->ecdat.frm_bytescore[i]) > max_minscore) max_minscore = fabs(gpx->ecdat.frm_bytescore[i]);
    }
    max_minscore = floor(max_minscore+1.5);

    ec = rs41_ecc(len);

    if (option.raw) {
        for (i = 0; i < len; i++) {
            fprintf(stdout, "%02x", gpx->frame[i]);
        }
        if (option.ecn) {
            if (ec >= 0) fprintf(stdout, " [OK]"); else fprintf(stdout, " [NO]");
            if (ec > 0) fprintf(stdout, " (%d)", ec);
            if (ec < 0) {
                if      (ec == -1)  fprintf(stdout, " (-+)");
                else if (ec == -2)  fprintf(stdout, " (+-)");
                else   /*ec == -3*/ fprintf(stdout, " (--)");
            }
        }
        fprintf(stdout, "\n");
        if (option.slt && option.jsn) {
            print_position(ec);
        }
    }
    else {
        print_position(ec);
    }
}

/* -------------------------------------------------------------------------- */


int main(int argc, char *argv[]) {

    FILE *fp;
    char *fpname = NULL;

    int option_min = 0;
    int option_iq = 0;
    int option_iqdc = 0;
    int option_lp = 0;
    int option_dc = 0;
    int option_noLUT = 0;
    int option_softin = 0;
    int option_pcmraw = 0;
    int sel_wavch = 0;     // audio channel: left
    int spike = 0;
    int fileloaded = 0;
    int rawhex = 0;
    int cfreq = -1;

    char bitbuf[BITS];
    int bitpos = 0,
        b8pos = 0,
        byte_count = FRAMESTART;
    int bit, byte;
    int bitQ;

    int k;

    int header_found = 0;

    float thres = 0.5;
    float _mv = 0.0;

    float set_lpIQbw = -1.0f;

    int symlen = 2;
    int bitofs = 2; // +0 .. +3
    int shift = 0;

    pcm_t pcm = {0};
    dsp_t dsp = {0};  //memset(&dsp, 0, sizeof(dsp));

    hdb_t hdb = {0};

    gpx_t gpx1 = {0};
    gpx_t gpx2 = {0};
    gpx1_pointer = &gpx1;
    gpx2_pointer = &gpx2;
    gpx = gpx1_pointer; //pointer to currently used dataset

#ifdef CYGWIN
    _setmode(fileno(stdin), O_BINARY);
#endif
    setbuf(stdout, NULL);

    fpname = argv[0];
    ++argv;
    while ((*argv) && (!fileloaded)) {
        if      ( (strcmp(*argv, "-h") == 0) || (strcmp(*argv, "--help") == 0) ) {
            fprintf(stderr, "%s [options] <file>\n", fpname);
            fprintf(stderr, "  file: audio.wav or raw_data\n");
            fprintf(stderr, "  options:\n");
            fprintf(stderr, "       -r, --raw    (raw hex output)\n");
            fprintf(stderr, "       -i, --invert\n");
            fprintf(stderr, "       --sub        (print subframe data)\n");
            fprintf(stderr, "       --eccn       (Reed-Solomon err count)\n");
            fprintf(stderr, "       --ths <x>    (peak threshold; default=%.1f)\n", thres);
            fprintf(stderr, "       --json       (adds JSON output)\n");
            fprintf(stderr, "       --jsnonly    (exclusively JSON out)\n");
            fprintf(stderr, "       -u <host> <port> (UDP JSON output)\n");
            return 0;
        }
        else if ( (strcmp(*argv, "-v")  == 0) ) { }
        else if ( (strcmp(*argv, "--sub") == 0) ) { option.sub = 1; }
        else if ( (strcmp(*argv, "--aux") == 0) ) { }
        else if   (strcmp(*argv, "--json") == 0)    { option.jsn = 1;}
        else if   (strcmp(*argv, "--jsnonly") == 0) { option.jsn = 2;}
        else if   (strcmp(*argv, "--eccn") == 0) { option.ecn = 1; }
        else if   (strcmp(*argv, "--ptu" ) == 0) { option.ptu = 1; }
        else if   (strcmp(*argv, "--dewp") == 0) { option.dwp = 1; }
        else if ( (strcmp(*argv, "-r") == 0) || (strcmp(*argv, "--raw") == 0) ) {
            option.raw = 1;
        }
        else if ( (strcmp(*argv, "-i") == 0) || (strcmp(*argv, "--invert") == 0) ) {
            option.inv = 1;
        }
        else if   (strcmp(*argv, "--jsn_cfq") == 0) {
            int frq = -1;  // center frequency / Hz
            ++argv;
            if (*argv) frq = atoi(*argv); else return -1;
            if (frq < 300000000) frq = -1;
            cfreq = frq;
        }
        else if   (strcmp(*argv, "--spike") == 0) { spike = 1; }
        else if   (strcmp(*argv, "--ch2") == 0) { sel_wavch = 1; }  // right channel (default: 0=left)
        else if   (strcmp(*argv, "--softin") == 0) { option_softin = 1; }  // float32 soft input
        else if   (strcmp(*argv, "--silent") == 0) { option.slt = 1; }
        else if   (strcmp(*argv, "--ths") == 0) {
            ++argv;
            if (*argv) {
                thres = atof(*argv);
            }
            else return -1;
        }
        else if ( (strcmp(*argv, "-d") == 0) ) {
            ++argv;
            if (*argv) {
                shift = atoi(*argv);
                if (shift >  4) shift =  4;
                if (shift < -4) shift = -4;
            }
            else return -1;
        }
        else if   (strcmp(*argv, "--iq0") == 0) { option_iq = 1; }  // differential/FM-demod
        else if   (strcmp(*argv, "--iq2") == 0) { option_iq = 2; }
        else if   (strcmp(*argv, "--iq3") == 0) { option_iq = 3; }  // iq2==iq3
        else if   (strcmp(*argv, "--iqdc") == 0) { option_iqdc = 1; }  // iq-dc removal (iq0,2,3)
        else if   (strcmp(*argv, "--IQ") == 0) { // fq baseband -> IF (rotate from and decimate)
            double fq = 0.0;                     // --IQ <fq> , -0.5 < fq < 0.5
            ++argv;
            if (*argv) fq = atof(*argv);
            else return -1;
            if (fq < -0.5) fq = -0.5;
            if (fq >  0.5) fq =  0.5;
            dsp.xlt_fq = -fq; // S(t) -> S(t)*exp(-f*2pi*I*t)
            option_iq = 5;
        }
        else if   (strcmp(*argv, "--lpIQ") == 0) { option_lp |= LP_IQ; }  // IQ/IF lowpass
        else if   (strcmp(*argv, "--lpbw") == 0) {  // IQ lowpass BW / kHz
            double bw = 0.0;
            ++argv;
            if (*argv) bw = atof(*argv);
            else return -1;
            if (bw > 4.6 && bw < 48.0) set_lpIQbw = bw*1e3;
            option_lp |= LP_IQ;
        }
        else if   (strcmp(*argv, "--lpFM") == 0) { option_lp |= LP_FM; }  // FM lowpass
        else if   (strcmp(*argv, "--dc") == 0) { option_dc = 1; }
        else if   (strcmp(*argv, "--noLUT") == 0) { option_noLUT = 1; }
        else if   (strcmp(*argv, "--min") == 0) {
            option_min = 1;
        }

        else if (strcmp(*argv, "--dbg" )   == 0) { option.dbg = 1; }
        else if (strcmp(*argv, "--rawhex") == 0) { rawhex = 2; }  // raw hex input
        else if (strcmp(*argv, "-") == 0) {
            int sample_rate = 0, bits_sample = 0, channels = 0;
            ++argv;
            if (*argv) sample_rate = atoi(*argv); else return -1;
            ++argv;
            if (*argv) bits_sample = atoi(*argv); else return -1;
            channels = 2;
            if (sample_rate < 1 || (bits_sample != 8 && bits_sample != 16 && bits_sample != 32)) {
                fprintf(stderr, "- <sr> <bs>\n");
                return -1;
            }
            pcm.sr  = sample_rate;
            pcm.bps = bits_sample;
            pcm.nch = channels;
            option_pcmraw = 1;
        }
        else if   (strcmp(*argv, "-u") == 0) {
            option.udp = 1;
            hostcfg.sfd = -1;
            ++argv;
            if (*argv) hostcfg.host = *argv; else return -1;
            ++argv;
            if (*argv) hostcfg.port = *argv; else return -1;
        }
        else if   (strcmp(*argv, "--autorx") == 0) { option.arx = 1;}
        else if   (strcmp(*argv, "--datetime") == 0) { option.dat = 1;}
        else if   (strcmp(*argv, "--add") == 0) {
            ++argv;
            if (*argv) key[extracnt] = *argv; else return -1;
            ++argv;
            if (*argv) val[extracnt] = *argv; else return -1;
            ++extracnt;
        }
        else {
            fp = fopen(*argv, "rb");
            if (fp == NULL) {
                fprintf(stderr, "error: open %s\n", *argv);
                return -1;
            }
            fileloaded = 1;
        }
        ++argv;
    }
    if (!fileloaded) fp = stdin;

    if (option.ptu < 1) option.ptu = 1;
    option.aut = 0;
    if (option_iq == 5 && option_dc) option_lp |= LP_FM;

    // LUT faster for decM, however frequency correction after decimation
    // LUT recommonded if decM > 2
    //
    if (option_noLUT && option_iq == 5) dsp.opt_nolut = 1; else dsp.opt_nolut = 0;

    rs_init_RS255(&RS);

    // init gpx
    memcpy(gpx1.frame, RS41D_header_bytes, sizeof(RS41D_header_bytes)); // 6 header bytes
    memcpy(gpx2.frame, RS41D_header_bytes, sizeof(RS41D_header_bytes));

    if (cfreq > 0) {
        gpx1.jsn_freq = (cfreq+500)/1000;
        gpx2.jsn_freq = gpx1.jsn_freq;
    }


    #ifdef EXT_FSK
    if (!option_softin) {
        option_softin = 1;
        fprintf(stderr, "reading float32 soft symbols\n");
    }
    #endif

    if (!rawhex) {

        if (!option_softin) {

            if (option_iq == 0 && option_pcmraw) {
                fclose(fp);
                fprintf(stderr, "error: raw data not IQ\n");
                return -1;
            }
            if (option_iq) sel_wavch = 0;

            pcm.sel_ch = sel_wavch;
            if (option_pcmraw == 0) {
                k = read_wav_header(&pcm, fp);
                if ( k < 0 ) {
                    fclose(fp);
                    fprintf(stderr, "error: wav header\n");
                    return -1;
                }
            }

            if (cfreq > 0) {
                int fq_kHz = (cfreq - dsp.xlt_fq*pcm.sr + 500)/1e3;
                gpx->jsn_freq = fq_kHz;
            }

            symlen = 2;
            // init dsp
            //
            dsp.fp = fp;
            dsp.sr = pcm.sr;
            dsp.bps = pcm.bps;
            dsp.nch = pcm.nch;
            dsp.ch = pcm.sel_ch;
            dsp.br = (float)BIT_RATE;
            dsp.sps = (float)dsp.sr/dsp.br;
            dsp.symlen = symlen;
            dsp.symhd  = symlen;
            dsp._spb = dsp.sps*symlen;
            dsp.hdr = rs41d_rawheader;
            dsp.hdrlen = strlen(rs41d_rawheader);
            dsp.BT = 0.5; // bw/time (ISI) // 0.3..0.5
            dsp.opt_iq = option_iq;
            dsp.opt_iqdc = option_iqdc;
            dsp.opt_lp = option_lp;
            dsp.lpFM_bw = 6e3; // FM audio lowpass
            dsp.opt_dc = option_dc;
            dsp.opt_IFmin = option_min;
            dsp.h = 3.8;        // modulation index: 1680/400=4.2, 4.2*0.9=3.8=4.75*0.8
            dsp.lpIQ_bw = 65e3; // IF lowpass bandwidth // 32e3=4.2*7.6e3 // 28e3..32e3
            if (set_lpIQbw > 0.0f) dsp.lpIQ_bw = set_lpIQbw;

            if ( dsp.sps < 8 ) {
                fprintf(stderr, "note: sample rate low (%.1f sps)\n", dsp.sps);
            }


            k = init_buffers(&dsp); // BT=0.5  (IQ-Int: BT > 0.5 ?)
            if ( k < 0 ) {
                fprintf(stderr, "error: init buffers\n");
                return -1;
            };

            bitofs += shift;
        }
        else {
            // init circular header bit buffer
            hdb.hdr = rs41d_rawheader;
            hdb.len = strlen(rs41d_rawheader);
            //hdb.thb = 1.0 - 3.1/(float)hdb.len; // 1.0-max_bit_errors/hdrlen
            hdb.bufpos = -1;
            hdb.buf = NULL;
            hdb.ths = 0.8;
            hdb.sbuf = calloc(hdb.len, sizeof(float));
            if (hdb.sbuf == NULL) {
                fprintf(stderr, "error: malloc\n");
                return -1;
            }
        }


        while ( 1 )
        {
            if (option_softin) {
                for (k = 0; k < hdb.len; k++) hdb.sbuf[k] = 0.0;
                header_found = find_softbinhead(fp, &hdb, &_mv);
            }
            else {
                header_found = find_header(&dsp, thres, 3, bitofs, dsp.opt_dc);
                _mv = dsp.mv;
            }

            if (header_found == EOF) break;

            // mv == correlation score
            if (_mv *(0.5-option.inv) < 0) {
                if (option.aut == 0) header_found = 0;
                else option.inv ^= 0x1;
            }

            if (header_found) {

                byte_count = FRAMESTART;
                bitpos = 0;
                b8pos = 0;

                while ( byte_count < FRAME_LEN ) {

                    if (option_softin) {
                        float s1 = 0.0;
                        float s2 = 0.0;
                        float s = 0.0;
                        bitQ = f32soft_read(fp, &s1);
                        if (bitQ != EOF) {
                            bitQ = f32soft_read(fp, &s2);
                            if (bitQ != EOF) {
                                s = s2-s1; // integrate both symbols  // only 2nd Manchester symbol: s2
                                bit = (s>=0.0); // no soft decoding
                            }
                        }
                    }
                    else {
                        float bl = -1;
                        if (option_iq > 2) bl = 4.0;
                        bitQ = read_slbit(&dsp, &bit, 0, bitofs, bitpos, bl, spike); // symlen=2
                    }
                    if ( bitQ == EOF) break;

                    if (option.inv) bit ^= 1;

                    bitpos += 1;
                    bitbuf[b8pos] = bit;
                    b8pos++;
                    if (b8pos >= BITS) {
                        b8pos = 0;
                        byte = bits2byte(bitbuf);
                        gpx->frame[byte_count] = byte;
                        byte_count++;
                    }
                }
                header_found = 0;
                print_frame(byte_count);
                byte_count = FRAMESTART;
            }
        }


        if (!option_softin) free_buffers(&dsp);
        else {
            if (hdb.buf) { free(hdb.buf); hdb.buf = NULL; }
        }
    }
    else //if (rawhex)
    {
        char buffer_rawhex[2*FRAME_LEN+12];
        char *pbuf = NULL, *buf_sp = NULL;
        ui8_t frmbyte;
        int frameofs = 0, len, i;

        while (1) {

            pbuf = fgets(buffer_rawhex, 2*FRAME_LEN+12, fp);
            if (pbuf == NULL) break;
            buffer_rawhex[2*FRAME_LEN] = '\0';
            buf_sp = strchr(buffer_rawhex, ' ');
            if (buf_sp != NULL && buf_sp - buffer_rawhex < 2*FRAME_LEN) {
                buffer_rawhex[buf_sp - buffer_rawhex] = '\0';
            }
            len = strlen(buffer_rawhex) / 2;
            if (len > pos_PTU + 24) {
                for (i = 0; i < len; i++) { //%2x  SCNx8=%hhx(inttypes.h)
                    sscanf(buffer_rawhex+2*i, "%2hhx", &frmbyte);
                    // wenn ohne %hhx: sscanf(buffer_rawhex+rawhex*i, "%2x", &byte); frame[frameofs+i] = (ui8_t)byte;
                    gpx->frame[frameofs+i] = frmbyte;
                }
                print_frame(frameofs+len);
            }
        }
    }
    fclose(fp);

    return 0;
}
