
/*
 *  RS92D
 *  sync header: correlation/matched filter
 *  files: RS92D.c nav_gps_vel.c bch_ecc_mod.c bch_ecc_mod.h demod_mod.c demod_mod.h
 *  compile:
 *  (a)
 *      gcc -c demod_mod.c
 *      gcc -DINCLUDESTATIC RS92D.c demod_mod.o -lm -o RS92D
 *  (b)
 *      gcc -c demod_mod.c
 *      gcc -c bch_ecc_mod.c
 *      gcc RS92D.c demod_mod.o bch_ecc_mod.o -lm -o RS92D
 *  CYGWIN
 *      gcc -DCYGWIN RS92D.c demod_mod.o bch_ecc_mod.o -lm -O3 -o RS92D  
 *
 *  author: zilog80 code modified to RS92-D by diehardsk
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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


//typedef unsigned char  ui8_t;
//typedef unsigned short ui16_t;
//typedef unsigned int   ui32_t;

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
    i8_t inv;
    i8_t aut;
    i8_t aux;  // aux/ozone
    i8_t jsn;  // JSON output
    i8_t dbg;
} option_t;

typedef struct {
    int typ;
    int msglen;
    int msgpos;
    int parpos;
    int hdrlen;
    int frmlen;
} rscfg_t;

static rscfg_t cfg_rs92 = { 92, 240-6-24, 6, 240-24, 6, 240};


/* --- RS92 digital: 8N1 manchester --- */
#define BITS (1+8+1)  // 10

#define FRAMESTART  6
#define FRAME_LEN 240

static char rs92_rawheader[] = //"10100110011001101001" //2A 8N1 Manchester encoded
                               //"10100110011001101001" //2A
                                 "10100110011001101001" //2A
                                 "10100110011001101001" //2A
                                 "1010011001100110100110101010100110101001"; //2A 10

static ui8_t RS92_header_bytes[6] = { 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x10};

static ui8_t RS92D_padding[130] = {
0x34, 0x12, 0x78, 0x56, 0x12, 0x90, 0x00, 0x04, 0x01, 0x04, 0x02, 0x04, 0x03, 0x04, 0x04, 0x04,
0x05, 0x04, 0x06, 0x04, 0x07, 0x04, 0x08, 0x04, 0x09, 0x04, 0x0a, 0x04, 0x0b, 0x04, 0x0c, 0x04,
0x0d, 0x04, 0x0e, 0x04, 0x0f, 0x04, 0x10, 0x04, 0x11, 0x04, 0x12, 0x04, 0x13, 0x04, 0x14, 0x04,
0x15, 0x04, 0x16, 0x04, 0x17, 0x04, 0x18, 0x04, 0x19, 0x04, 0x1a, 0x04, 0x1b, 0x04, 0x1c, 0x04,
0x1d, 0x04, 0x1e, 0x04, 0x1f, 0x04, 0x20, 0x04, 0x21, 0x04, 0x22, 0x04, 0x23, 0x04, 0x24, 0x04,
0x25, 0x04, 0x26, 0x04, 0x27, 0x04, 0x28, 0x04, 0x29, 0x04, 0x2a, 0x04, 0x2b, 0x04, 0x2c, 0x04,
0x2d, 0x04, 0x2e, 0x04, 0x2f, 0x04, 0x30, 0x04, 0x31, 0x04, 0x32, 0x04, 0x33, 0x04, 0x34, 0x04,
0x35, 0x04, 0x36, 0x04, 0x37, 0x04, 0x38, 0x04, 0x39, 0x04, 0x3a, 0x04, 0x3b, 0x04, 0x3c, 0x04,
0x00, 0x00 };

typedef struct {
    int frnr;
    char id[11];
    ui16_t conf_kt; // kill timer (sec)
    int freq;       // freq/kHz (RS92)
    int jsn_freq;   // freq/kHz (SDR)
    ui32_t crc;
    ui8_t frame[FRAME_LEN];
    ui8_t cal_state[2];
    ui8_t calfrms;
    ui8_t calibytes[32*16];
    ui8_t calfrchk[32];
    float cal_f32[256];
    float T;
    float _RH; float RH;
    float _P; float P;
    unsigned short aux[4];
    option_t option;
    RS_t RS;
} gpx_t;


/* ------------------------------------------------------------------------------------ */

#define BIT_RATE 4800

/* ------------------------------------------------------------------------------------ */

// manchester1 1->10,0->01: 1.bit
// manchester2 0->10,1->01: 2.bit
// RS92-SGP: 8N1 manchester2
static int bits2byte(char bits[]) {
    int i, byteval=0, d=1;

    //if (bits[0] != 0) return 0x100; // erasure?
    //if (bits[9] != 1) return 0x100; // erasure?

    for (i = 1; i <= 8; i++) {   // little endian
    /* for (i = 8; i > 1; i--) { // big endian */
        if      (bits[i] == 1)  byteval += d;
        else if (bits[i] == 0)  byteval += 0;
        d <<= 1;
    }
    return byteval;
}



/* ------------------------------------------------------------------------------------ */

#define crc_STAT    (1<<0)
#define pos_FrameNb   0x08  // 2 byte
#define pos_SondeID   0x0C  // 8 byte  // oder: 0x0A, 10 byte?
#define pos_BitField0C  0x14  // 1 byte
#define pos_BitField0D  0x15  // 1 byte
#define pos_CalData   0x17  // 1 byte, counter 0x00..0x1f
#define pos_Calfreq   0x1A  // 2 byte, calfr 0x00

#define crc_PTU      (1<<1)
#define pos_PTU       0x2C  // 24 byte

#define crc_AUX      (1<<2)
#define pos_AUX       0x48  // 10 byte
#define pos_AuxData   0x4A  // 8 byte

#define pos_PADD      0x56  // 130 bytes


#define BLOCK_CFG 0x6510  // frame[pos_FrameNb-2], frame[pos_FrameNb-1]
#define BLOCK_PTU 0x690C
//#define BLOCK_GPS 0x673D
#define BLOCK_AUX 0x6805
#define BLOCK_PADD 0xFF41

#define LEN_CFG (2*(BLOCK_CFG & 0xFF))
#define LEN_PTU (2*(BLOCK_PTU & 0xFF))
#define LEN_AUX (2*(BLOCK_AUX & 0xFF))

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c %c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

static int crc16(gpx_t *gpx, int start, int len) {
    int crc16poly = 0x1021;
    int rem = 0xFFFF, i, j;
    int byte;

    if (start+len >= FRAME_LEN) return -1;
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

static int crc_fail(gpx_t *gpx, int start, int len) {
    int crc_frame, crc;
    crc_frame = gpx->frame[start + len] | (gpx->frame[start + len + 1] << 8);
    crc = crc16(gpx, start, len);
    if (crc_frame != crc)
        return -1;
    return 0;
}

static int get_FrameNb(gpx_t *gpx) {
    int i;
    unsigned byte;
    ui8_t frnr_bytes[2];
    int frnr;

    for (i = 0; i < 2; i++) {
        byte = gpx->frame[pos_FrameNb + i];
        frnr_bytes[i] = byte;
    }
    frnr = frnr_bytes[0] + (frnr_bytes[1] << 8);
    gpx->frnr = frnr;
    return 0;
}


static int get_SondeID(gpx_t *gpx) {
    int i;
    unsigned byte;
    char sondeid_bytes[10];
    ui8_t calfr;

    if (crc_fail(gpx, pos_FrameNb, LEN_CFG)) {
        gpx->crc |= crc_STAT;
        return -2;
    }

    for (i = 0; i < 8; i++) {
        byte = gpx->frame[pos_SondeID + i];
        if ((byte < 0x20) || (byte > 0x7E)) return -1;
        sondeid_bytes[i] = byte;
    }
    sondeid_bytes[8] = '\0';

    if ( strncmp(gpx->id, sondeid_bytes, 8) != 0 ) {
        memset(gpx->calibytes, 0, 32*16);
        memset(gpx->calfrchk, 0, 32);
        memset(gpx->cal_f32, 0, 256*4);
        gpx->calfrms = 0;
        gpx->T = -275.15f;
        gpx->_RH = -1.0f;
        gpx->_P  = -1.0f;
        gpx->RH  = -1.0f;
        gpx->P   = -1.0f;
        // new ID:
        memcpy(gpx->id, sondeid_bytes, 8);
    }

    memcpy(gpx->cal_state, gpx->frame+(pos_FrameNb + 12), 2);

    calfr = gpx->frame[pos_CalData]; // 0..31
    if (calfr < 32) {
        if (gpx->calfrchk[calfr] == 0) // const?
        {
            for (i = 0; i < 16; i++) {
                gpx->calibytes[calfr*16 + i] = gpx->frame[pos_CalData+1+i];
            }
            gpx->calfrchk[calfr] = 1;
        }
    }

    if (gpx->calfrms < 32) {
        gpx->calfrms = 0;
        for (i = 0; i < 32; i++) gpx->calfrms += (gpx->calfrchk[i]>0);
    }
    if (gpx->calfrms == 32)
    {
        ui8_t xcal[66*5];

        gpx->calfrms += 1;

        for (int j = 0; j < 66*5; j++) {
            xcal[j] = gpx->calibytes[0x40+j];
        }

        for (int j = 0; j < 66; j++) {
            ui8_t idx = xcal[5*j];
            ui8_t *dat = xcal+(5*j+1);
            ui32_t le_dat32 = dat[0] | (dat[1]<<8) | (dat[2]<<16) | (dat[3]<<24);
            float *pf32 = (float*)&le_dat32;
            gpx->cal_f32[idx] = *pf32;

            if (gpx->option.dbg)
            {
                if (idx/10 == 3 || idx/10 == 4 || idx/10 == 5)
                {
                    printf(" %3d :", idx);
                    for (int i = 1; i < 5; i++) {
                        printf(" %02x", xcal[5*j+i]);
                    }
                    printf(" : %f", *pf32);
                    printf("\n");
                }
            }
        }
    }
    return 0;
}


// ----------------------------------------------------------------------------------------------------

// PTU
// cf. Haeberli (2001),
//     https://brmlab.cz/project/weathersonde/telemetry_decoding
//
static float poly5(float x, float *a) {
    float p = 0.0;
    p = ((((a[5]*x+a[4])*x+a[3])*x+a[2])*x+a[1])*x+a[0];
    return p;
}
static float nu(float t, float t0, float y0) {
    // t=1/f2-1/f , t0=1/f2-1/f1 , 1/freq=meas24
    float y = t / t0;
    return 1.0f / (y0 - y);
}

static int get_Meas(gpx_t *gpx) {
    ui32_t temp, pres, hum1, hum2, ref1, ref2, ref3, ref4;
    ui8_t *meas24 = gpx->frame+pos_PTU;
    float T, U1, U2, _P, _rh, x;

    temp = meas24[ 0] | (meas24[ 1]<<8) | (meas24[ 2]<<16);  // ch1
    hum1 = meas24[ 3] | (meas24[ 4]<<8) | (meas24[ 5]<<16);  // ch2
    hum2 = meas24[ 6] | (meas24[ 7]<<8) | (meas24[ 8]<<16);  // ch3
    ref1 = meas24[ 9] | (meas24[10]<<8) | (meas24[11]<<16);  // ch4
    ref2 = meas24[12] | (meas24[13]<<8) | (meas24[14]<<16);  // ch5
    pres = meas24[15] | (meas24[16]<<8) | (meas24[17]<<16);  // ch6
    ref3 = meas24[18] | (meas24[19]<<8) | (meas24[20]<<16);  // ch7
    ref4 = meas24[21] | (meas24[22]<<8) | (meas24[23]<<16);  // ch8

    if (gpx->calfrms > 0x20)
    {
        // Temperature
        x = nu( (float)(ref1 - temp), (float)(ref1 - ref4), gpx->cal_f32[37] );
        T = poly5(x, gpx->cal_f32+30);
        if (T > -120.0f && T < 80.0f)  gpx->T = T;
        else gpx->T = -273.15f;

        // rel. Humidity (ref3 or ref4 ?)
        x = nu( (float)(ref1 - hum1), (float)(ref1 - ref3), gpx->cal_f32[47] );
        U1 = poly5(x, gpx->cal_f32+40); // c[44]=c[45]=0
        x = nu( (float)(ref1 - hum2), (float)(ref1 - ref3), gpx->cal_f32[57] );
        U2 = poly5(x, gpx->cal_f32+50); // c[54]=c[55]=0
        _rh = U1 > U2 ? U1 : U2; // max(U1,U2), vgl. cal_state[1].bit3
        gpx->_RH = _rh;
        if (gpx->_RH < 0.0f) gpx->_RH = 0.0f;
        if (gpx->_RH > 100.0f) gpx->_RH = 100.0f;
        // correction for higher RH-sensor temperature (at low temperatures)?
        // (cf. amt-7-4463-2014)
        // if (T<-60C || P<100hPa): cal_state[1].bit2=0
        // (Hyland and Wexler)
        // if (T>-60C && P>100hPa): rh = _rh*vaporSatP(Trh)/vaporSatP(T) ...
        // estimate Trh ?

        // (uncorrected) Pressure
        x = nu( (float)(ref1 - pres), (float)(ref1 - ref4), gpx->cal_f32[17] );
        _P = poly5(x, gpx->cal_f32+10);
        if (_P < 0.0f && _P > 2000.0f) _P = -1.0f;
        gpx->_P = _P;
        // correction for x and coefficients?
    }
    return 0;
}

// ----------------------------------------------------------------------------------------------------

static int get_PTU(gpx_t *gpx) {
    if (crc_fail(gpx, pos_PTU, LEN_PTU)) {
        gpx->crc |= crc_PTU;
        return -2;
    }
    if (gpx->calfrms > 0x20) return get_Meas(gpx);
    else return 0;
}


static int get_Aux(gpx_t *gpx) {
    int i;
    unsigned short byte;

    for (i = 0; i < 4; i++) {
        byte = gpx->frame[pos_AuxData+2*i] + (gpx->frame[pos_AuxData+2*i+1]<<8);
        gpx->aux[i] = byte;
    }

    if (crc_fail(gpx, pos_AUX, LEN_AUX)) {
        gpx->crc |= crc_AUX;
        return -2;
    }
    else return 0;
}


static int get_Cal(gpx_t *gpx) {
    int i;
    unsigned byte;
    ui8_t calfr = 0;
    ui8_t bytes[2];
    int freq = 0;
    ui16_t killtime = 0;

    byte = gpx->frame[pos_CalData];
    calfr = byte;

    if (gpx->option.sub == 1) {
        fprintf(stdout, "\n");
        fprintf(stdout, "[%5d] ", gpx->frnr);
        fprintf(stdout, "  0x%02x:", calfr);
        for (i = 0; i < 16; i++) {
            byte = gpx->frame[pos_CalData+1+i];
            fprintf(stdout, " %02x", byte);
        }
        if ((gpx->crc & crc_STAT)==0) fprintf(stdout, " [OK]"); else fprintf(stdout, " [NO]");
    }

    if (calfr == 0x00) {
        for (i = 0; i < 2; i++) {
            bytes[i] = gpx->frame[pos_Calfreq + i];
        }
        byte = bytes[0] + (bytes[1] << 8);
        freq = 1600000 + 10*byte; // kHz
        gpx->freq = freq;
        if(gpx->option.jsn < 2) fprintf(stdout, ": fq %d", freq);
        for (i = 0; i < 2; i++) {
            bytes[i] = gpx->frame[pos_Calfreq + 2 + i];
        }
        killtime = bytes[0] + (bytes[1] << 8); // signed?
        if (killtime < 0xFFFF && gpx->option.jsn < 2) {
            fprintf(stdout, "; KT:%ds", killtime);
        }
        gpx->conf_kt = killtime;
    }
    return 0;
}


/* ------------------------------------------------------------------------------------ */

#define rs_N 255
#define rs_R 24
#define rs_K (rs_N-rs_R)

static int rs92_ecc(gpx_t *gpx, int msglen) {

    int i;
    int errors;
    ui8_t cw[rs_N];
    ui8_t err_pos[rs_R], err_val[rs_R];

    memset(cw, 0, rs_N);

    if (msglen > FRAME_LEN) msglen = FRAME_LEN;
    for (i = msglen; i < FRAME_LEN; i++) gpx->frame[i] = 0;//xFF;


    for (i = 0; i < rs_R;            i++) cw[i]      = gpx->frame[cfg_rs92.parpos+i];
    for (i = 0; i < cfg_rs92.msglen; i++) cw[rs_R+i] = gpx->frame[cfg_rs92.msgpos+i];

    errors = rs_decode(&gpx->RS, cw, err_pos, err_val);

    for (i = 0; i < rs_R;            i++) gpx->frame[cfg_rs92.parpos+i] = cw[i];
    for (i = 0; i < cfg_rs92.msglen; i++) gpx->frame[cfg_rs92.msgpos+i] = cw[rs_R+i];

    return errors;
}

/* ------------------------------------------------------------------------------------ */

static int print_data(gpx_t *gpx, int ec) {
    int j = 0;
    int err1, err2, err4;

    err1  = get_FrameNb(gpx);
    err1 |= get_SondeID(gpx);
    err2  = get_PTU(gpx);
    err4  = get_Aux(gpx);

    if (!err1)
    {
        if (gpx->option.jsn < 2) {
            fprintf(stdout, "[%5d] ", gpx->frnr);
            fprintf(stdout, "(%s)  ", gpx->id);
            
            if (gpx->option.dbg) {
                ui8_t BFC = gpx->frame[pos_BitField0C];
                ui8_t BFD = gpx->frame[pos_BitField0D];
                fprintf(stdout, " BF="BYTE_TO_BINARY_PATTERN", "BYTE_TO_BINARY_PATTERN,
                    BYTE_TO_BINARY(BFC), BYTE_TO_BINARY(BFD));
            }

            if (!err2 && gpx->option.ptu) {
                if (gpx->T > -273.0f) fprintf(stdout, " T=%.1fC ", gpx->T);
                if (gpx->_RH > -0.5f) fprintf(stdout, " _RH=%.0f%% ", gpx->_RH);
                if (gpx->_P  >  0.0f) fprintf(stdout, " _P=%.1fhPa ", gpx->_P);
            }

            if (gpx->option.aux && !err4) {
                if (gpx->aux[0] != 0 || gpx->aux[1] != 0 || gpx->aux[2] != 0 || gpx->aux[3] != 0) {
                    fprintf(stdout, " # %04x %04x %04x %04x", gpx->aux[0], gpx->aux[1], gpx->aux[2], gpx->aux[3]);
                }
            }

            fprintf(stdout, "  ");
            if (gpx->option.dbg) {
                fprintf(stdout, "# [");
                for (j=0; j<3; j++) fprintf(stdout, "%d", (gpx->crc>>j)&1);
                fprintf(stdout, "]");
            }
            if (gpx->option.ecn) {
                if (ec > 0) fprintf(stdout, " (%d)", ec);
                if (ec < 0) fprintf(stdout, " (-)");
            }
        }

        get_Cal(gpx);

        if (gpx->option.jsn) {
            // Print out telemetry data as JSON
            if ((gpx->crc & crc_STAT)==0) //(!err1)
            {
                char *ver_jsn = NULL;
                if (gpx->option.jsn==1) fprintf(stdout, "\n");
                fprintf(stdout, "{ \"type\": \"RS92\"");
                fprintf(stdout, ", \"frame\": %d, \"id\": \"%s\"", gpx->frnr, gpx->id);
                if (gpx->option.ptu && !err2) {
                    if (gpx->T > -273.0f) {
                        fprintf(stdout, ", \"temp\": %.1f",  gpx->T );
                    }
                    if (gpx->_RH > -0.5f) {
                        fprintf(stdout, ", \"humidity\": %.1f",  gpx->_RH );
                    }
                    if (gpx->_P > 0.0f) {
                        fprintf(stdout, ", \"pressure\": %.2f",  gpx->_P );
                    }
                }
                if ((gpx->crc & crc_AUX)==0 && (gpx->aux[0] != 0 || gpx->aux[1] != 0 || gpx->aux[2] != 0 || gpx->aux[3] != 0)) {
                    fprintf(stdout, ", \"aux\": \"%04x%04x%04x%04x\"", gpx->aux[0], gpx->aux[1], gpx->aux[2], gpx->aux[3]);
                }
                fprintf(stdout, ", \"subtype\": \"RS92-D\"" );
                if (gpx->jsn_freq > 0) {  // rs92-frequency: gpx->freq
                    int fq_kHz = gpx->jsn_freq;
                    //if (gpx->freq > 0) fq_kHz = gpx->freq; // L-band: option.ngp ?
                    fprintf(stdout, ", \"freq\": %d", fq_kHz );
                }

                // Include frequency derived from subframe information if available.
                if (gpx->freq > 0) {
                    fprintf(stdout, ", \"tx_frequency\": %d", gpx->freq );
                }

                #ifdef VER_JSN_STR
                    ver_jsn = VER_JSN_STR;
                #endif
                if (ver_jsn && *ver_jsn != '\0') fprintf(stdout, ", \"version\": \"%s\"", ver_jsn);
                fprintf(stdout, " }");
                if (gpx->option.jsn==1) fprintf(stdout, "\n");
            }
        }
        fprintf(stdout, "\n");
    }
    return err1;
}

static void print_frame(gpx_t *gpx, int len) {
    int i, ec = 0;
    ui8_t byte;

    gpx->crc = 0;
    //we can repair fix data in unused frame space before running ecc
    for (i = 0; i < sizeof(RS92D_padding); i++) {
        gpx->frame[i + pos_PADD] = RS92D_padding[i];
    }

    ec = rs92_ecc(gpx, len);

    for (i = len; i < FRAME_LEN; i++) {
        gpx->frame[i] = 0;
    }

    if (gpx->option.raw) {
        for (i = 0; i < len; i++) {
            byte = gpx->frame[i];
            fprintf(stdout, "%02x", byte);
        }
        if (gpx->option.ecn) {
            fprintf(stdout, " ");
            if (ec >= 0) fprintf(stdout, " [OK]"); else fprintf(stdout, " [NO]");
            if (ec > 0) fprintf(stdout, " (%d)", ec);
            if (ec < 0) fprintf(stdout, " (-)");
        }
        fprintf(stdout, "\n");
    }
    else print_data(gpx, ec);
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

    gpx_t gpx = {0};

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
            fprintf(stderr, "       --aux        (print aux data if any)\n");
            fprintf(stderr, "       --eccn       (Reed-Solomon err count)\n");
            fprintf(stderr, "       --ths <x>    (peak threshold; default=%.1f)\n", thres);
            fprintf(stderr, "       --json       (adds JSON output)\n");
            fprintf(stderr, "       --jsnonly    (exclusively JSON out)\n");
            return 0;
        }
        else if ( (strcmp(*argv, "-v")  == 0) ) { }
        else if ( (strcmp(*argv, "--sub") == 0) ) { gpx.option.sub = 1; }
        else if ( (strcmp(*argv, "--aux") == 0) ) { gpx.option.aux = 1; }
        else if   (strcmp(*argv, "--jsnonly") == 0) { gpx.option.jsn = 2; }
        else if   (strcmp(*argv, "--eccn") == 0) { gpx.option.ecn = 1; }
        else if   (strcmp(*argv, "--ptu" ) == 0) { gpx.option.ptu = 1; }
        else if ( (strcmp(*argv, "-r") == 0) || (strcmp(*argv, "--raw") == 0) ) {
            gpx.option.raw = 1;
        }
        else if ( (strcmp(*argv, "-i") == 0) || (strcmp(*argv, "--invert") == 0) ) {
            gpx.option.inv = 1;
        }
        else if   (strcmp(*argv, "--json") == 0) {
            gpx.option.jsn = 1;
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
        else if   (strcmp(*argv, "--silent") == 0) { gpx.option.jsn = 2; }
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
        else if   (strcmp(*argv, "--dbg" ) == 0) { gpx.option.dbg = 1; }
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

    gpx.option.ptu = 1;
    gpx.option.aut = 0;
    if (option_iq == 5 && option_dc) option_lp |= LP_FM;

    // LUT faster for decM, however frequency correction after decimation
    // LUT recommonded if decM > 2
    //
    if (option_noLUT && option_iq == 5) dsp.opt_nolut = 1; else dsp.opt_nolut = 0;

    rs_init_RS255(&gpx.RS);

    // init gpx
    memcpy(gpx.frame, RS92_header_bytes, sizeof(RS92_header_bytes)); // 6 header bytes

    if (cfreq > 0) gpx.jsn_freq = (cfreq+500)/1000;


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
                gpx.jsn_freq = fq_kHz;
            }

            // rs92-sgp: BT=0.5, h=1.0 ?
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
            dsp.hdr = rs92_rawheader;
            dsp.hdrlen = strlen(rs92_rawheader);
            dsp.BT = 0.5; // bw/time (ISI) // 0.3..0.5
            dsp.opt_iq = option_iq;
            dsp.opt_iqdc = option_iqdc;
            dsp.opt_lp = option_lp;
            dsp.lpFM_bw = 6e3; // FM audio lowpass
            dsp.opt_dc = option_dc;
            dsp.opt_IFmin = option_min;
            dsp.h = 3.8;        // modulation index: 1680/400=4.2, 4.2*0.9=3.8=4.75*0.8
            dsp.lpIQ_bw = 40e3; // IF lowpass bandwidth // 32e3=4.2*7.6e3 // 28e3..32e3
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
            hdb.hdr = rs92_rawheader;
            hdb.len = strlen(rs92_rawheader);
            //hdb.thb = 1.0 - 3.1/(float)hdb.len; // 1.0-max_bit_errors/hdrlen
            hdb.bufpos = -1;
            hdb.buf = NULL;
            // caution ths=0.7: -3 byte offset, false positive
            // 2A 2A 2A 2A 2A 10|65 10 ..
            // header sync could be extended into the frame
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
            if (_mv *(0.5-gpx.option.inv) < 0) {
                if (gpx.option.aut == 0) header_found = 0;
                else gpx.option.inv ^= 0x1;
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

                    if (gpx.option.inv) bit ^= 1;

                    bitpos += 1;
                    bitbuf[b8pos] = bit;
                    b8pos++;
                    if (b8pos >= BITS) {
                        b8pos = 0;
                        byte = bits2byte(bitbuf);
                        gpx.frame[byte_count] = byte;
                        byte_count++;
                    }
                }
                header_found = 0;
                print_frame(&gpx, byte_count);
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
                    gpx.frame[frameofs+i] = frmbyte;
                }
                print_frame(&gpx, frameofs+len);
            }
        }
    }
    fclose(fp);

    return 0;
}
