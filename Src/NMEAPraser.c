/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo
Description:
    Library of functions to manage a GNSS module (typically GPS) for accurate
    timestamping of packets and synchronisation of gateways.
    A limited set of module brands/models are supported.
License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Michael Coracin
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#define _GNU_SOURCE     /* needed for qsort_r to be defined */
#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <string.h>     /* memcpy */

#include <time.h>       /* struct timespec */
#include <fcntl.h>      /* open */
#include <math.h>       /* modf */

#include <stdlib.h>

#include <NMEAPraser.h>

#include "stm32f7xx_hal.h"

#include "SEGGER_RTT.h"

#define GPS_TIME_OFFSET_FROM_BUFFER_SEC 1
/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_GPS == 0
    #define DEBUG_MSG(args...)
    #define DEBUG_ARRAY(a,b,c)  for(a=0;a!=0;){}
    #define CHECK_NULL(a)       if(a==NULL){return LGW_GPS_ERROR;}
#else
    #define DEBUG_MSG(args...)  SEGGER_RTT_printf(0, args)
    #define DEBUG_ARRAY(a,b,c)  for(a=0;a<b;++a) SEGGER_RTT_printf(0,"%x.",c[a]);SEGGER_RTT_printf(0,"end\n")
    #define CHECK_NULL(a)       if(a==NULL){SEGGER_RTT_printf(0,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_GPS_ERROR;}

#endif
#define TRACE()         fprintf(stderr, "@ %s %d\n", __FUNCTION__, __LINE__);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

//TODO Replace with calculated Sysclock freq
#define TS_CPS              108E6 /* count-per-second of the timestamp counter */ //replaced with sysfreq

#define PLUS_1PERCENT          1.01*(double)(TS_CPS)
#define MINUS_1PERCENT         0.99*(double)(TS_CPS)

#define UBX_MSG_NAVTIMEGPS_LEN  16

#ifndef GPS_TIME_OFFSET_FROM_BUFFER_SEC
#define GPS_TIME_OFFSET_FROM_BUFFER_SEC 0
#endif
/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */


/* result of the NMEA parsing */
static short gps_yea = 0; /* year (2 or 4 digits) */
static short gps_mon = 0; /* month (1-12) */
static short gps_day = 0; /* day of the month (1-31) */
static short gps_hou = 0; /* hours (0-23) */
static short gps_min = 0; /* minutes (0-59) */
static short gps_sec = 0; /* seconds (0-60)(60 is for leap second) */
static float gps_fra = 0.0; /* fractions of seconds (<1) */
static bool gps_time_ok = false;
static int16_t gps_week = 0; /* GPS week number of the navigation epoch */
static uint64_t gps_iTOW = 0; /* GPS time of week in milliseconds */
static int32_t gps_fTOW = 0; /* Fractional part of iTOW (+/-500000) in nanosec */

static short gps_dla = 0; /* degrees of latitude */
static double gps_mla = 0.0; /* minutes of latitude */
static char gps_ola = 0; /* orientation (N-S) of latitude */
static short gps_dlo = 0; /* degrees of longitude */
static double gps_mlo = 0.0; /* minutes of longitude */
static char gps_olo = 0; /* orientation (E-W) of longitude */
static float gps_alt = 0; /* altitude */
static bool gps_pos_ok = false;

static float gps_hdop = 0.0; /* horizontal delution of precision */
static float gps_cmg = 0.0; /* Course Made Good */
static float gps_sog = 0.0; /* Speed over ground in m/s */

static char gps_mod = 'N'; /* GPS mode (N no fix, A autonomous, D differential) */
static short gps_sat = 0; /* number of satellites used for fix */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static int nmea_checksum(const char *nmea_string, int buff_size, char *checksum);

static char nibble_to_hexchar(uint8_t a);

static bool validate_nmea_checksum(const char *serial_buff, int buff_size);

static bool match_label(const char *s, char *label, int size, char wildcard);

static int str_chop(char *s, int buff_size, char separator, int *idx_ary, int max_idx);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/*
Calculate the checksum for a NMEA string
Skip the first '$' if necessary and calculate checksum until '*' character is
reached (or buff_size exceeded).
Checksum must point to a 2-byte (or more) char array.
Return position of the checksum in the string
*/
static int nmea_checksum(const char *nmea_string, int buff_size, char *checksum) {
    int i = 0;
    uint8_t check_num = 0;

    /* check input parameters */
    if ((nmea_string == NULL) ||  (checksum == NULL) || (buff_size <= 1)) {
        DEBUG_MSG("Invalid parameters for nmea_checksum\n");
        return -1;
    }

    /* skip the first '$' if necessary */
    if (nmea_string[i] == '$') {
        i += 1;
    }

    /* xor until '*' or max length is reached */
    while (nmea_string[i] != '*') {
        check_num ^= nmea_string[i];
        i += 1;
        if (i >= buff_size) {
            DEBUG_MSG("Maximum length reached for nmea_checksum\n");
            return -1;
        }
    }

    /* Convert checksum value to 2 hexadecimal characters */
    checksum[0] = nibble_to_hexchar(check_num / 16); /* upper nibble */
    checksum[1] = nibble_to_hexchar(check_num % 16); /* lower nibble */

    return i + 1;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static char nibble_to_hexchar(uint8_t a) {
    if (a < 10) {
        return '0' + a;
    } else if (a < 16) {
        return 'A' + (a-10);
    } else {
        return '?';
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/*
Calculate the checksum of a NMEA frame and compare it to the checksum that is
present at the end of it.
Return true if it matches
*/
static bool validate_nmea_checksum(const char *serial_buff, int buff_size) {
    int checksum_index;
    char checksum[2]; /* 2 characters to calculate NMEA checksum */

    checksum_index = nmea_checksum(serial_buff, buff_size, checksum);

    /* could we calculate a verification checksum ? */
    if (checksum_index < 0) {
        DEBUG_MSG("ERROR: IMPOSSIBLE TO PARSE NMEA SENTENCE\n");
        return false;
    }

    /* check if there are enough char in the serial buffer to read checksum */
    if (checksum_index >= (buff_size - 2)) {
        DEBUG_MSG("ERROR: IMPOSSIBLE TO READ NMEA SENTENCE CHECKSUM\n");
        return false;
    }

    /* check the checksum per se */
    if ((serial_buff[checksum_index] == checksum[0]) && (serial_buff[checksum_index+1] == checksum[1])) {
        return true;
    } else {
        DEBUG_MSG("ERROR: NMEA CHECKSUM %c%c DOESN'T MATCH VERIFICATION CHECKSUM %c%c\n", serial_buff[checksum_index], serial_buff[checksum_index+1], checksum[0], checksum[1]);
        return false;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/*
Return true if the "label" string (can contain wildcard characters) matches
the begining of the "s" string
*/
static bool match_label(const char *s, char *label, int size, char wildcard) {
    int i;

    for (i=0; i < size; i++) {
        if (label[i] == wildcard) continue;
        if (label[i] != s[i]) return false;
    }
    return true;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/*
Chop a string into smaller strings
Replace every separator in the input character buffer by a null character so
that all s[index] are valid strings.
Populate an array of integer 'idx_ary' representing indexes of token in the
string.
buff_size and max_idx are there to prevent segfaults.
Return the number of token found (number of idx_ary filled).
*/
int str_chop(char *s, int buff_size, char separator, int *idx_ary, int max_idx) {
    int i = 0; /* index in the string */
    int j = 0; /* index in the result array */

    if ((s == NULL) || (buff_size < 0) || (separator == 0) || (idx_ary == NULL) || (max_idx < 0)) {
        /* unsafe to do anything */
        return -1;
    }
    if ((buff_size == 0) || (max_idx == 0)) {
        /* nothing to do */
        return 0;
    }
    s[buff_size - 1] = 0; /* add string terminator at the end of the buffer, just to be sure */
    idx_ary[j] = 0;
    j += 1;
    /* loop until string terminator is reached */
    while (s[i] != 0) {
        if (s[i] == separator) {
            s[i] = 0; /* replace separator by string terminator */
            if (j >= max_idx) { /* no more room in the index array */
                return j;
            }
            idx_ary[j] = i+1; /* next token start after replaced separator */
            ++j;
        }
        ++i;
    }
    return j;
}

enum gps_msg lgw_parse_ubx(const char *serial_buff, size_t buff_size, size_t *msg_size) {
    bool valid = 0;    /* iTOW, fTOW and week validity */
    unsigned int payload_length;
    uint8_t ck_a, ck_b;
    uint8_t ck_a_rcv, ck_b_rcv;
    unsigned int i;

    *msg_size = 0; /* ensure msg_size alway receives a value */

    /* check input parameters */
    if (serial_buff == NULL) {
        return IGNORED;
    }
    if (buff_size < 8) {
        DEBUG_MSG("ERROR: TOO SHORT TO BE A VALID UBX MESSAGE\n");
        return IGNORED;
    }

    /* display received serial data and checksum */
    DEBUG_MSG("Note: parsing UBX frame> ");
    for (i=0; i<buff_size; i++) {
        DEBUG_MSG("%02x ", serial_buff[i]);
    }
    DEBUG_MSG("\n");

    /* Check for UBX sync chars 0xB5 0x62 */
    if ((serial_buff[0] == (char)0xB5) && (serial_buff[1] == (char)0x62)) {

        /* Get payload length to compute message size */
        payload_length  = (uint8_t)serial_buff[4];
        payload_length |= (uint8_t)serial_buff[5] << 8;
        *msg_size = 6 + payload_length + 2; /* header + payload + checksum */

        /* check for complete message in buffer */
        if(*msg_size <= buff_size) {
            /* Validate checksum of message */
            ck_a_rcv = serial_buff[*msg_size-2]; /* received checksum */
            ck_b_rcv = serial_buff[*msg_size-1]; /* received checksum */
            /* Use 8-bit Fletcher Algorithm to compute checksum of actual payload */
            ck_a = 0; ck_b = 0;
            for (i=0; i<(4 + payload_length); i++) {
                ck_a = ck_a + serial_buff[i+2];
                ck_b = ck_b + ck_a;
            }

            /* Compare checksums and parse if OK */
            if ((ck_a == ck_a_rcv) && (ck_b == ck_b_rcv)) {
                /* Check for Class 0x01 (NAV) and ID 0x20 (NAV-TIMEGPS) */
                if ((serial_buff[2] == 0x01) && (serial_buff[3] == 0x20)) {
                    /* Check validity of information */
                    valid = serial_buff[17] & 0x3; /* towValid, weekValid */
                    if (valid) {
                        /* Parse buffer to extract GPS time */
                        /* Warning: payload byte ordering is Little Endian */
                        gps_iTOW =  (uint8_t)serial_buff[6];
                        gps_iTOW |= (uint8_t)serial_buff[7] << 8;
                        gps_iTOW |= (uint8_t)serial_buff[8] << 16;
                        gps_iTOW |= (uint8_t)serial_buff[9] << 24; /* GPS time of week, in ms */

                        gps_fTOW =  (uint8_t)serial_buff[10];
                        gps_fTOW |= (uint8_t)serial_buff[11] << 8;
                        gps_fTOW |= (uint8_t)serial_buff[12] << 16;
                        gps_fTOW |= (uint8_t)serial_buff[13] << 24; /* Fractional part of iTOW, in ns */

                        gps_week =  (uint8_t)serial_buff[14];
                        gps_week |= (uint8_t)serial_buff[15] << 8; /* GPS week number */

                        gps_time_ok = true;
#if 0
                        /* For debug */
                        {
                            short ubx_gps_hou = 0; /* hours (0-23) */
                            short ubx_gps_min = 0; /* minutes (0-59) */
                            short ubx_gps_sec = 0; /* seconds (0-59) */

                            /* Format GPS time in hh:mm:ss based on iTOW */
                            ubx_gps_sec = (gps_iTOW / 1000) % 60;
                            ubx_gps_min = (gps_iTOW / 1000 / 60) % 60;
                            ubx_gps_hou = (gps_iTOW / 1000 / 60 / 60) % 24;
                            printf("  GPS time = %02d:%02d:%02d\n", ubx_gps_hou, ubx_gps_min, ubx_gps_sec);
                        }
#endif
                    } else { /* valid */
                        gps_time_ok = false;
                    }

                    return UBX_NAV_TIMEGPS;
                } else if ((serial_buff[2] == 0x05) && (serial_buff[3] == 0x00)) {
                    DEBUG_MSG("NOTE: UBX ACK-NAK received\n");
                    return IGNORED;
                } else if ((serial_buff[2] == 0x05) && (serial_buff[3] == 0x01)) {
                    DEBUG_MSG("NOTE: UBX ACK-ACK received\n");
                    return IGNORED;
                } else { /* not a supported message */
                    DEBUG_MSG("ERROR: UBX message is not supported (%02x %02x)\n", serial_buff[2], serial_buff[3]);
                    return IGNORED;
                }
            } else { /* checksum failed */
                DEBUG_MSG("ERROR: UBX message is corrupted, checksum failed\n");
                return INVALID;
            }
        } else { /* message contains less bytes than indicated by header */
            DEBUG_MSG("ERROR: UBX message incomplete\n");
            return INCOMPLETE;
        }
    } else { /* Not a UBX message */
        /* Ignore messages which are not UBX ones for now */
        return IGNORED;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

enum gps_msg lgw_parse_nmea(const char *serial_buff, int buff_size) {
    int i, j, k;
    int str_index[30]; /* string index from the string chopping */
    int nb_fields; /* number of strings detected by string chopping */
    char parser_buf[256]; /* parsing modifies buffer so need a local copy */

    /* check input parameters */
    if (serial_buff == NULL) {
        return UNKNOWN;
    }
    DEBUG_MSG("MSG to Parse :");
    //DEBUG_MSG(serial_buff); TODO check this
    if(buff_size > (int)(sizeof(parser_buf) - 1)) {
        DEBUG_MSG("Note: input string to big for parsing\n");
        return INVALID;
    }

    /* look for some NMEA sentences in particular */
    if (buff_size < 8) {
        DEBUG_MSG("ERROR: TOO SHORT TO BE A VALID NMEA SENTENCE\n");
        return UNKNOWN;
    } else if (!validate_nmea_checksum(serial_buff, buff_size)) {
        DEBUG_MSG("Warning: invalid NMEA sentence (bad checksum)\n");
        return INVALID;
    } else if (match_label(serial_buff, "$G?RMC", 6, '?')) {
        /*
        NMEA sentence format: $xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode*cs<CR><LF>
        Valid fix: $GPRMC,083559.34,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A*00
        No fix: $GPRMC,,V,,,,,,,,,,N*00
        */
        memcpy(parser_buf, serial_buff, buff_size);
        parser_buf[buff_size] = '\0';
        nb_fields = str_chop(parser_buf, buff_size, ',', str_index, ARRAY_SIZE(str_index));
        if (nb_fields != 13) {
            DEBUG_MSG("Warning: invalid RMC sentence (number of fields)\n");
            return IGNORED;
        }
        /* parse GPS status */
        gps_mod = *(parser_buf + str_index[12]); /* get first character, no need to bother with sscanf */
        if ((gps_mod != 'N') && (gps_mod != 'A') && (gps_mod != 'D')) {
            gps_mod = 'N';
        }
        /* parse complete time */
        i = sscanf(parser_buf + str_index[1], "%2hd%2hd%2hd%4f", &gps_hou, &gps_min, &gps_sec, &gps_fra);
        j = sscanf(parser_buf + str_index[9], "%2hd%2hd%2hd", &gps_day, &gps_mon, &gps_yea);
        sscanf(parser_buf + str_index[7], "%f", &gps_sog);
        sscanf(parser_buf + str_index[8], "%f", &gps_cmg);
        if ((i == 4) && (j == 3)) {
            if ((gps_mod == 'A') || (gps_mod == 'D')) {
                gps_time_ok = true;
                DEBUG_MSG("Note: Valid RMC sentence, GPS locked, date: 20%02d-%02d-%02dT%02d:%02d:%06.3fZ\n", gps_yea, gps_mon, gps_day, gps_hou, gps_min, gps_fra + (float)gps_sec);
            } else {
                gps_time_ok = false;
                DEBUG_MSG("Note: Valid RMC sentence, no satellite fix, estimated date: 20%02d-%02d-%02dT%02d:%02d:%06.3fZ\n", gps_yea, gps_mon, gps_day, gps_hou, gps_min, gps_fra + (float)gps_sec);
            }
        } else {
            /* could not get a valid hour AND date */
            gps_time_ok = false;
            DEBUG_MSG("Note: Valid RMC sentence, mode %c, no date\n", gps_mod);
        }
        return NMEA_RMC;
    } else if (match_label(serial_buff, "$G?GGA", 6, '?')) {
        /*
        NMEA sentence format: $xxGGA,time,lat,NS,long,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs<CR><LF>
        Valid fix: $GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B
        */
        memcpy(parser_buf, serial_buff, buff_size);
        parser_buf[buff_size] = '\0';
        nb_fields = str_chop(parser_buf, buff_size, ',', str_index, ARRAY_SIZE(str_index));
        if (nb_fields != 15) {
            DEBUG_MSG("Warning: invalid GGA sentence (number of fields)\n");
            return IGNORED;
        }
        /* parse number of satellites used for fix */
        sscanf(parser_buf + str_index[7], "%hd", &gps_sat);
        /* parse number of satellites used for fix */
        sscanf(parser_buf + str_index[8], "%f", &gps_hdop);
        /* parse 3D coordinates */
        i = sscanf(parser_buf + str_index[2], "%2hd%10lf", &gps_dla, &gps_mla);
        gps_ola = *(parser_buf + str_index[3]);
        j = sscanf(parser_buf + str_index[4], "%3hd%10lf", &gps_dlo, &gps_mlo);
        gps_olo = *(parser_buf + str_index[5]);
        k = sscanf(parser_buf + str_index[9], "%f", &gps_alt);
        if ((i == 2) && (j == 2) && (k == 1) && ((gps_ola=='N')||(gps_ola=='S')) && ((gps_olo=='E')||(gps_olo=='W'))) {
            gps_pos_ok = true;
            DEBUG_MSG("Note: Valid GGA sentence, %d sat, lat %02ddeg %06.3fmin %c, lon %03ddeg%06.3fmin %c, alt %f\n", gps_sat, gps_dla, gps_mla, gps_ola, gps_dlo, gps_mlo, gps_olo, gps_alt);
        } else {
            /* could not get a valid latitude, longitude AND altitude */
            gps_pos_ok = false;
            DEBUG_MSG("Note: Valid GGA sentence, %d sat, no coordinates\n", gps_sat);
        }
        return NMEA_GGA;
    } else {
        DEBUG_MSG("Note: ignored NMEA sentence\n"); /* quite verbose */
        return IGNORED;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_gps_get(struct timespec *utc, struct timespec *gps_time, struct coord_s *loc, struct coord_s *err) {
    struct tm x;
    time_t y;
    double intpart, fractpart;

    if (utc != NULL) {
        if (!gps_time_ok) {
            DEBUG_MSG("ERROR: NO VALID TIME TO RETURN\n");
            return LGW_GPS_ERROR;
        }
        memset(&x, 0, sizeof(x));
        if (gps_yea < 100) { /* 2-digits year, 20xx */
            x.tm_year = gps_yea + 100; /* 100 years offset to 1900 */
        } else { /* 4-digits year, Gregorian calendar */
            x.tm_year = gps_yea - 1900;
        }
        x.tm_mon = gps_mon - 1; /* tm_mon is [0,11], gps_mon is [1,12] */
        x.tm_mday = gps_day;
        x.tm_hour = gps_hou;
        x.tm_min = gps_min;
        x.tm_sec = gps_sec;
        y = mktime(&x); /* need to substract timezone bc mktime assumes time vector is local time */
        if (y == (time_t)(-1)) {
            DEBUG_MSG("ERROR: FAILED TO CONVERT BROKEN-DOWN TIME\n");
            return LGW_GPS_ERROR;
        }
        utc->tv_sec = y+GPS_TIME_OFFSET_FROM_BUFFER_SEC;
        utc->tv_nsec = (int32_t)(gps_fra * 1e9);
    }
    if (gps_time != NULL) {
        if (!gps_time_ok) {
            DEBUG_MSG("ERROR: NO VALID TIME TO RETURN\n");
            return LGW_GPS_ERROR;
        }
        fractpart = modf(((double)gps_iTOW / 1E3) + ((double)gps_fTOW / 1E9), &intpart);
        /* Number of seconds since beginning on current GPS week */
        gps_time->tv_sec = (time_t)intpart;
        /* Number of seconds since GPS epoch 06.Jan.1980 */
        gps_time->tv_sec += (time_t)gps_week * 604800; /* day*hours*minutes*secondes: 7*24*60*60; */

        gps_time->tv_sec +=GPS_TIME_OFFSET_FROM_BUFFER_SEC;
        /* Fractional part in nanoseconds */
        gps_time->tv_nsec = (long)(fractpart * 1E9);
    }
    if (loc != NULL) {
        if (!gps_pos_ok) {
            DEBUG_MSG("ERROR: NO VALID POSITION TO RETURN\n");
            return LGW_GPS_ERROR;
        }
        loc->lat = ((double)gps_dla + (gps_mla/60.0)) * ((gps_ola == 'N')?1.0:-1.0);
        loc->lon = ((double)gps_dlo + (gps_mlo/60.0)) * ((gps_olo == 'E')?1.0:-1.0);
        loc->alt = gps_alt;
        loc->hdop =gps_hdop;
        loc->cmg=gps_cmg;
        loc->sog=gps_sog;
        loc->sat=gps_sat;

    }
    if (err != NULL) {
        DEBUG_MSG("Warning: localization error processing not implemented yet\n");
        err->lat = 0.0;
        err->lon = 0.0;
        err->alt = 0;
    }

    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_gps_sync(struct tref *ref, uint64_t count_us, struct timespec utc, struct timespec gps_time) {
    double cnt_diff; /* internal concentrator time difference (in seconds) */
    double utc_diff; /* UTC time difference (in seconds) */
    double slope; /* time slope between new reference and old reference (for sanity check) */

    bool aber_n0=false; /* is the update value for synchronization aberrant or not ? */
    static bool aber_min1 = false; /* keep track of whether value at sync N-1 was aberrant or not  */
    static bool aber_min2 = false; /* keep track of whether value at sync N-2 was aberrant or not  */

    CHECK_NULL(ref);
    int64_t cntdiff64=count_us - ref->count_us;
    /* calculate the slope */
    cnt_diff = (double)(cntdiff64); /* uncorrected by xtal_err  / (double)(TS_CPS)*/
    utc_diff = (double)(utc.tv_sec - (ref->utc).tv_sec) + (1E-9 * (double)(utc.tv_nsec - (ref->utc).tv_nsec));

    /* detect aberrant points by measuring if slope limits are exceeded */
    if (utc_diff != 0) { // prevent divide by zero
        slope = cnt_diff/utc_diff;
        if ((slope > PLUS_1PERCENT) || (slope < MINUS_1PERCENT)) {
            DEBUG_MSG("Warning: correction range exceeded\n");
            aber_n0 = false;
        } else {
            aber_n0 = false;
            ref->xtal_err_array[ref->array_update_pointer]=slope;
            if(ref->array_update_pointer<(sizeof(ref->xtal_err_array)/sizeof(ref->xtal_err_array[0]))-1)
            {
            ref->array_update_pointer++;
            }
            else
            {
            	ref->array_update_pointer=0;
            }
            if(ref->array_valid_data_count<(sizeof(ref->xtal_err_array)/sizeof(ref->xtal_err_array[0])))
            {
            ref->array_valid_data_count++; // count up to array size than stop counting until array flush since from now on ever value in the array is valide
            }
        }
    } else {
        DEBUG_MSG("Warning: aberrant UTC value for synchronization\n");
        aber_n0 = true;
    }

    double mean_tmp=0;
    double std_tmp=0;
    for (int i=0;i<ref->array_valid_data_count;i++)
    {
    	mean_tmp=mean_tmp+ref->xtal_err_array[i];
    }
    mean_tmp/=ref->array_valid_data_count;
    for(int i=0;i<ref->array_valid_data_count;i++)
    	std_tmp += (ref->xtal_err_array[i] - mean_tmp)*(ref->xtal_err_array[i] - mean_tmp);
    DEBUG_MSG("GPS Slope std val is:%d\n",std_tmp);
    std_tmp=sqrt(std_tmp/(ref->array_valid_data_count-1));
    DEBUG_MSG("GPS Slope std val is:%d\n",std_tmp);
    /* watch if the 3 latest sync point were aberrant or not */
    if (aber_n0 == false) {
        /* value no aberrant -> sync with smoothed slope */
        ref->systime = time(NULL);
        ref->count_us = count_us;
        ref->utc.tv_sec = utc.tv_sec;
        ref->utc.tv_nsec = utc.tv_nsec;
        ref->gps.tv_sec = gps_time.tv_sec;
        ref->gps.tv_nsec = gps_time.tv_nsec;
        ref->xtal_err_deviation=std_tmp;
        //ref->xtal_err = slope;
        ref->xtal_err=mean_tmp;
        aber_min2 = aber_min1;
        aber_min1 = aber_n0;
        return LGW_GPS_SUCCESS;
    } else if (aber_n0 && aber_min1 && aber_min2) {
        /* 3 successive aberrant values -> sync reset (keep xtal_err) */
        ref->systime = time(NULL);
        ref->count_us = count_us;
        ref->utc.tv_sec = utc.tv_sec;
        ref->utc.tv_nsec = utc.tv_nsec;
        ref->gps.tv_sec = gps_time.tv_sec;
        ref->gps.tv_nsec = gps_time.tv_nsec;
        /* reset xtal_err only if the present value is out of range */
        if ((ref->xtal_err > PLUS_1PERCENT) || (ref->xtal_err < MINUS_1PERCENT)) {
            ref->xtal_err = 1.0;
        }
        DEBUG_MSG("Warning: 3 successive aberrant sync attempts, sync reset\n");
        aber_min2 = aber_min1;
        aber_min1 = aber_n0;
        return LGW_GPS_SUCCESS;
    } else {
        /* only 1 or 2 successive aberrant values -> ignore and return an error */
        aber_min2 = aber_min1;
        aber_min1 = aber_n0;
        return LGW_GPS_ERROR;
    }

    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_cnt2utc(struct tref ref, uint64_t count_us, struct timespec *utc,uint32_t * time_uncertainty) {
	utc->tv_sec=0;
	utc->tv_nsec=0;
    double delta_sec;
    double intpart, fractpart;
    long tmp;

    CHECK_NULL(utc);
    if ((ref.systime == 0) || (ref.xtal_err > PLUS_1PERCENT) || (ref.xtal_err < MINUS_1PERCENT)) {
        DEBUG_MSG("ERROR: INVALID REFERENCE FOR CNT -> UTC CONVERSION\n");
        return LGW_GPS_ERROR;
    }

    /* calculate delta in seconds between reference count_us and target count_us */
    int64_t cntdiff64=0;
    cntdiff64=(count_us - ref.count_us);
#if DEBUG_GPS == 1
    if(cntdiff64<0)
    {
    	DEBUG_MSG("DELTA TICKS<0\n");
    }
#endif
    delta_sec = (double)(cntdiff64) / (ref.xtal_err);//TS_CPS *
    double time_uncertainty_tmp=(2*ref.xtal_err_deviation)*(2*ref.xtal_err_deviation)+(2*ref.xtal_err_deviation*delta_sec)*(2*ref.xtal_err_deviation*delta_sec);
#define NANOSECONDSTOTICKSSCALEFACKTOR 1E9/TS_CPS
    //TODO implement weightend menad value filter
    time_uncertainty_tmp=2*sqrt(time_uncertainty_tmp)*NANOSECONDSTOTICKSSCALEFACKTOR;
    time_uncertainty_tmp=round(time_uncertainty_tmp);
    uint32_t time_uncertainty_int32=(uint32_t)time_uncertainty_tmp;
    //DEBUG_MSG("unsicherheit= %f , %lu\r\n",time_uncertainty_tmp,time_uncertainty_int32);
    memcpy(time_uncertainty,&time_uncertainty_int32,sizeof(time_uncertainty_int32));
    /* now add that delta to reference UTC time */
    fractpart = modf (delta_sec , &intpart);
    tmp = ref.utc.tv_nsec + (long)(fractpart * 1E9);
    if (tmp < (long)1E9 &&tmp>=0) { /* the nanosecond part doesn't over or underflow */
        utc->tv_sec = ref.utc.tv_sec + (time_t)intpart;
        utc->tv_nsec = tmp;
    } else if(tmp > (long)1E9 &&tmp>0) { /* must carry one second */
        utc->tv_sec = ref.utc.tv_sec + (time_t)intpart + 1;
        utc->tv_nsec = tmp - (long)1E9;
    } else if(tmp < (long)1E9 &&tmp<0)
    {
        utc->tv_sec = ref.utc.tv_sec + (time_t)intpart -1;
        utc->tv_nsec = (long)1E9+tmp;
    }
    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_utc2cnt(struct tref ref, struct timespec utc, uint64_t *count_us) {
    double delta_sec;

    CHECK_NULL(count_us);
    if ((ref.systime == 0) || (ref.xtal_err > PLUS_1PERCENT) || (ref.xtal_err < MINUS_1PERCENT)) {
        DEBUG_MSG("ERROR: INVALID REFERENCE FOR UTC -> CNT CONVERSION\n");
        return LGW_GPS_ERROR;
    }

    /* calculate delta in seconds between reference utc and target utc */
    uint64_t time_diff=utc.tv_sec - ref.utc.tv_sec;
    delta_sec = (double)(time_diff);
    uint64_t reff_div=utc.tv_nsec - ref.utc.tv_nsec;
    delta_sec += 1E-9 * (double)(reff_div);

    /* now convert that to internal counter tics and add that to reference counter value */
    *count_us = ref.count_us + (uint64_t)(delta_sec * ref.xtal_err);//* TS_CPS

    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_cnt2gps(struct tref ref, uint64_t count_us, struct timespec *gps_time) {
    double delta_sec;
    double intpart, fractpart;
    long tmp;

    CHECK_NULL(gps_time);
    if ((ref.systime == 0) || (ref.xtal_err > PLUS_1PERCENT) || (ref.xtal_err < MINUS_1PERCENT)) {
        DEBUG_MSG("ERROR: INVALID REFERENCE FOR CNT -> GPS CONVERSION\n");
        return LGW_GPS_ERROR;
    }

    /* calculate delta in milliseconds between reference count_us and target count_us */
    delta_sec = (double)(count_us - ref.count_us) / ( ref.xtal_err);//TS_CPS *

    /* now add that delta to reference GPS time */
    fractpart = modf (delta_sec , &intpart);
    tmp = ref.gps.tv_nsec + (long)(fractpart * 1E9);
    if (tmp < (long)1E9) { /* the nanosecond part doesn't overflow */
        gps_time->tv_sec = ref.gps.tv_sec + (time_t)intpart;
        gps_time->tv_nsec = tmp;
    } else { /* must carry one second */
        gps_time->tv_sec = ref.gps.tv_sec + (time_t)intpart + 1;
        gps_time->tv_nsec = tmp - (long)1E9;
    }

    return LGW_GPS_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_gps2cnt(struct tref ref, struct timespec gps_time, uint64_t *count_us) {
    double delta_sec;

    CHECK_NULL(count_us);
    if ((ref.systime == 0) || (ref.xtal_err > PLUS_1PERCENT) || (ref.xtal_err < MINUS_1PERCENT)) {
        DEBUG_MSG("ERROR: INVALID REFERENCE FOR GPS -> CNT CONVERSION\n");
        return LGW_GPS_ERROR;
    }

    /* calculate delta in seconds between reference gps time and target gps time */
    delta_sec = (double)(gps_time.tv_sec - ref.gps.tv_sec);
    delta_sec += 1E-9 * (double)(gps_time.tv_nsec - ref.gps.tv_nsec);

    /* now convert that to internal counter tics and add that to reference counter value */
    *count_us = ref.count_us + (uint64_t)(delta_sec  * ref.xtal_err);//* TS_CPS

    return LGW_GPS_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
