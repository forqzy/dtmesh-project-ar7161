/*
 * Copyright (c) 2009, Atheros Communications Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#ident "$Id: //depot/sw/releases/7.3_AP/wlan/common/lmac/ratectrl/ar5416Phy.c#4 $"

#ifdef __FreeBSD__
#include <dev/ath/ath_rate/atheros/ratectrl.h>
#else
#include <osdep.h>
#include "ratectrl.h"
#include "ratectrl11n.h"
#endif

#define SHORT_PRE 1
#define LONG_PRE 0

#define WLAN_PHY_HT_20_SS       WLAN_RC_PHY_HT_20_SS
#define WLAN_PHY_HT_20_DS       WLAN_RC_PHY_HT_20_DS
#define WLAN_PHY_HT_20_DS_HGI   WLAN_RC_PHY_HT_20_DS_HGI
#define WLAN_PHY_HT_40_SS       WLAN_RC_PHY_HT_40_SS
#define WLAN_PHY_HT_40_SS_HGI   WLAN_RC_PHY_HT_40_SS_HGI
#define WLAN_PHY_HT_40_DS       WLAN_RC_PHY_HT_40_DS
#define WLAN_PHY_HT_40_DS_HGI   WLAN_RC_PHY_HT_40_DS_HGI

#ifndef ATH_NO_5G_SUPPORT
static RATE_TABLE_11N ar5416_11naRateTable = {

    42,  /* number of rates */
     /*              Multi     Single   Single                                                                                                                           */ 
    {/*               strm       strm     strm                                                  rate  short   dot11 ctrl RssiAck  RssiAck  base  cw40  sgi    ht  4ms tx */
     /*              valid      valid     STBC                                   Kbps   uKbps   Code  Preamb  Rate  Rate ValidMin DeltaMin Idx   Idx   Idx   Idx   limit */
     /*    6 Mb */ {  TRUE,      TRUE,    TRUE,         WLAN_PHY_OFDM,           6000,   5400,  0x0b,  0x00,   12,   0,    2,       1,     0,    0,    0,    0,      0},
     /*    9 Mb */ {  TRUE,      TRUE,    TRUE,         WLAN_PHY_OFDM,           9000,   7800,  0x0f,  0x00,   18,   0,    3,       1,     1,    1,    1,    1,      0},
     /*   12 Mb */ {  TRUE,      TRUE,    TRUE,         WLAN_PHY_OFDM,          12000,  10000,  0x0a,  0x00,   24,   2,    4,       2,     2,    2,    2,    2,      0},
     /*   18 Mb */ {  TRUE,      TRUE,    TRUE,         WLAN_PHY_OFDM,          18000,  13900,  0x0e,  0x00,   36,   2,    6,       2,     3,    3,    3,    3,      0},
     /*   24 Mb */ {  TRUE,      TRUE,    TRUE,         WLAN_PHY_OFDM,          24000,  17300,  0x09,  0x00,   48,   4,   10,       3,     4,    4,    4,    4,      0},
     /*   36 Mb */ {  TRUE,      TRUE,    TRUE,         WLAN_PHY_OFDM,          36000,  23000,  0x0d,  0x00,   72,   4,   14,       3,     5,    5,    5,    5,      0},
     /*   48 Mb */ {  TRUE,      TRUE,    TRUE,         WLAN_PHY_OFDM,          48000,  27400,  0x08,  0x00,   96,   4,   20,       3,     6,    6,    6,    6,      0},
     /*   54 Mb */ {  TRUE,      TRUE,    TRUE,         WLAN_PHY_OFDM,          54000,  29300,  0x0c,  0x00,  108,   4,   23,       3,     7,    7,    7,    7,      0},
     /*  6.5 Mb */ {  TRUE_2040, TRUE_2040, TRUE_2040,  WLAN_PHY_HT_20_SS,       6500,   6400,  0x80,  0x00,    0,   0,    2,       3,     8,   24,    8,   24,   3216},
     /*   13 Mb */ {  TRUE_20,   TRUE_20, TRUE_20,      WLAN_PHY_HT_20_SS,      13000,  12700,  0x81,  0x00,    1,   2,    4,       3,     9,   25,    9,   25,   6434},
     /* 19.5 Mb */ {  TRUE_20,   TRUE_20, TRUE_20,      WLAN_PHY_HT_20_SS,      19500,  18800,  0x82,  0x00,    2,   2,    6,       3,    10,   26,   10,   26,   9650},
     /*   26 Mb */ {  TRUE_20,   TRUE_20, TRUE_20,      WLAN_PHY_HT_20_SS,      26000,  25000,  0x83,  0x00,    3,   4,   10,       3,    11,   27,   11,   27,  12868},
     /*   39 Mb */ {  TRUE_20,   TRUE_20, TRUE_20,      WLAN_PHY_HT_20_SS,      39000,  36700,  0x84,  0x00,    4,   4,   14,       3,    12,   28,   12,   28,  19304},
     /*   52 Mb */ {  FALSE,     TRUE_20, TRUE_20,      WLAN_PHY_HT_20_SS,      52000,  48100,  0x85,  0x00,    5,   4,   20,       3,    13,   29,   13,   29,  25740},
     /* 58.5 Mb */ {  FALSE,     TRUE_20, TRUE_20,      WLAN_PHY_HT_20_SS,      58500,  53500,  0x86,  0x00,    6,   4,   23,       3,    14,   30,   14,   30,  28956},
     /*   65 Mb */ {  FALSE,     TRUE_20, FALSE,        WLAN_PHY_HT_20_SS,      65000,  59000,  0x87,  0x00,    7,   4,   25,       3,    15,   31,   15,   32,  32180},
     /*   13 Mb */ {  FALSE,     FALSE,   FALSE,        WLAN_PHY_HT_20_DS,      13000,  12700,  0x88,  0x00,    8,   0,    2,       3,    16,   33,   16,   33,   6430},
     /*   26 Mb */ {  FALSE,     FALSE,   FALSE,        WLAN_PHY_HT_20_DS,      26000,  24800,  0x89,  0x00,    9,   2,    4,       3,    17,   34,   17,   34,  12860},
     /*   39 Mb */ {  FALSE,     FALSE,   FALSE,        WLAN_PHY_HT_20_DS,      39000,  36600,  0x8a,  0x00,   10,   2,    6,       3,    18,   35,   18,   35,  19300},
     /*   52 Mb */ {  TRUE_20,   FALSE,   FALSE,        WLAN_PHY_HT_20_DS,      52000,  48100,  0x8b,  0x00,   11,   4,   10,       3,    19,   36,   19,   36,  25736},
     /*   78 Mb */ {  TRUE_20,   FALSE,   TRUE_20,      WLAN_PHY_HT_20_DS,      78000,  69500,  0x8c,  0x00,   12,   4,   14,       3,    20,   37,   20,   37,  38600},
     /*  104 Mb */ {  TRUE_20,   FALSE,   TRUE_20,      WLAN_PHY_HT_20_DS,     104000,  89500,  0x8d,  0x00,   13,   4,   20,       3,    21,   38,   21,   38,  51472},
     /*  117 Mb */ {  TRUE_20,   FALSE,   TRUE_20,      WLAN_PHY_HT_20_DS,     117000,  98900,  0x8e,  0x00,   14,   4,   23,       3,    22,   39,   22,   39,  57890},
     /*  130 Mb */ {  TRUE_20,   FALSE,   TRUE_20,      WLAN_PHY_HT_20_DS,     130000, 108300,  0x8f,  0x00,   15,   4,   25,       3,    23,   40,   23,   41,  64320},
     /* 13.5 Mb */ {  TRUE_40,   TRUE_40, TRUE_40,      WLAN_PHY_HT_40_SS,      13500,  13200,  0x80,  0x00,    0,   0,    2,       3,     8,   24,   24,   24,   6684},
     /* 27.0 Mb */ {  TRUE_40,   TRUE_40, TRUE_40,      WLAN_PHY_HT_40_SS,      27500,  25900,  0x81,  0x00,    1,   2,    4,       3,     9,   25,   25,   25,  13368},
     /* 40.5 Mb */ {  TRUE_40,   TRUE_40, TRUE_40,      WLAN_PHY_HT_40_SS,      40500,  38600,  0x82,  0x00,    2,   2,    6,       3,    10,   26,   26,   26,  20052},
     /*   54 Mb */ {  TRUE_40,   TRUE_40, TRUE_40,      WLAN_PHY_HT_40_SS,      54000,  49800,  0x83,  0x00,    3,   4,   10,       3,    11,   27,   27,   27,  26738},
     /*   81 Mb */ {  TRUE_40,   TRUE_40, TRUE_40,      WLAN_PHY_HT_40_SS,      81500,  72200,  0x84,  0x00,    4,   4,   14,       3,    12,   28,   28,   28,  40104},
     /*  108 Mb */ {  FALSE,     TRUE_40, TRUE_40,      WLAN_PHY_HT_40_SS,     108000,  92900,  0x85,  0x00,    5,   4,   20,       3,    13,   29,   29,   29,  53476},
     /* 121.5Mb */ {  FALSE,     TRUE_40, TRUE_40,      WLAN_PHY_HT_40_SS,     121500, 102700,  0x86,  0x00,    6,   4,   23,       3,    14,   30,   30,   30,  60156},
     /*  135 Mb */ {  FALSE,     TRUE_40, FALSE,        WLAN_PHY_HT_40_SS,     135000, 112000,  0x87,  0x00,    7,   4,   25,       3,    15,   31,   32,   32,  66840},
     /*  150 Mb */ {  FALSE,     TRUE_40, FALSE,        WLAN_PHY_HT_40_SS_HGI, 150000, 122000,  0x87,  0x00,    7,   4,   25,       3,    15,   31,   32,   32,  74200},
     /*   27 Mb */ {  FALSE,     FALSE,   FALSE,        WLAN_PHY_HT_40_DS,      27000,  25800,  0x88,  0x00,    8,   0,    2,       3,    16,   33,   33,   33,  13360},
     /*   54 Mb */ {  FALSE,     FALSE,   FALSE,        WLAN_PHY_HT_40_DS,      54000,  49800,  0x89,  0x00,    9,   2,    4,       3,    17,   34,   34,   34,  26720},
     /*   81 Mb */ {  FALSE,     FALSE,   FALSE,        WLAN_PHY_HT_40_DS,      81000,  71900,  0x8a,  0x00,   10,   2,    6,       3,    18,   35,   35,   35,  40080},
     /*  108 Mb */ {  TRUE_40,   FALSE,   FALSE,        WLAN_PHY_HT_40_DS,     108000,  92500,  0x8b,  0x00,   11,   4,   10,       3,    19,   36,   36,   36,  53440},
     /*  162 Mb */ {  TRUE_40,   FALSE,   TRUE_40,      WLAN_PHY_HT_40_DS,     162000, 130300,  0x8c,  0x00,   12,   4,   14,       3,    20,   37,   37,   37,  80160},
     /*  216 Mb */ {  TRUE_40,   FALSE,   TRUE_40,      WLAN_PHY_HT_40_DS,     216000, 162800,  0x8d,  0x00,   13,   4,   20,       3,    21,   38,   38,   38, 106880},
     /*  243 Mb */ {  TRUE_40,   FALSE,   TRUE_40,      WLAN_PHY_HT_40_DS,     243000, 178200,  0x8e,  0x00,   14,   4,   23,       3,    22,   39,   39,   39, 120240},
     /*  270 Mb */ {  TRUE_40,   FALSE,   TRUE_40,      WLAN_PHY_HT_40_DS,     270000, 192100,  0x8f,  0x00,   15,   4,   25,       3,    23,   40,   41,   41, 133600},
     /*  300 Mb */ {  TRUE_40,   FALSE,   TRUE_40,      WLAN_PHY_HT_40_DS_HGI, 300000, 207000,  0x8f,  0x00,   15,   4,   25,       3,    23,   40,   41,   41, 148400},
    },
    50,  /* probe interval */
    50,  /* rssi reduce interval */
    WLAN_RC_HT_FLAG,  /* Phy rates allowed initially */
};
#endif /* #ifndef ATH_NO_5G_SUPPORT */

	/* TRUE_ALL - valid for 20/40/Legacy, TRUE - Legacy only, TRUE_20 - HT 20 only, TRUE_40 - HT 40 only */
    /* 4ms frame limit not used for NG mode.  The values filled for HT are the 64K max aggregate limit */

static RATE_TABLE_11N ar5416_11ngRateTable = {

    46,  /* number of rates - should match the no. of rows below */
     /*              Multi      Single      Single                                                                                                                         */ 
    {/*               strm        strm        strm                                                       short   dot11 ctrl RssiAck  RssiAck  base cw40  sgi   ht   4ms tx */
     /*              valid       valid        STBC                                  Kbps    uKbps   RC   Preamb  Rate  Rate ValidMin DeltaMin Idx  Idx   Idx   Idx   limit */
     /*    1 Mb */ {  TRUE_ALL,  TRUE_ALL,  TRUE_ALL,      WLAN_PHY_CCK,            1000,    900,  0x1b,  0x00,    2,   0,    0,       1,     0,    0,    0,    0,      0},
     /*    2 Mb */ {  TRUE_ALL,  TRUE_ALL,  TRUE_ALL,      WLAN_PHY_CCK,            2000,   1900,  0x1a,  0x04,    4,   1,    1,       1,     1,    1,    1,    1,      0},
     /*  5.5 Mb */ {  TRUE_ALL,  TRUE_ALL,  TRUE_ALL,      WLAN_PHY_CCK,            5500,   4900,  0x19,  0x04,   11,   2,    2,       2,     2,    2,    2,    2,      0},
     /*   11 Mb */ {  TRUE_ALL,  TRUE_ALL,  TRUE_ALL,      WLAN_PHY_CCK,           11000,   8100,  0x18,  0x04,   22,   3,    3,       2,     3,    3,    3,    3,      0},
     /*    6 Mb */ {  FALSE,     FALSE,     FALSE,         WLAN_PHY_OFDM,           6000,   5400,  0x0b,  0x00,   12,   4,    2,       1,     4,    4,    4,    4,      0},
     /*    9 Mb */ {  FALSE,     FALSE,     FALSE,         WLAN_PHY_OFDM,           9000,   7800,  0x0f,  0x00,   18,   4,    3,       1,     5,    5,    5,    5,      0},
     /*   12 Mb */ {  TRUE,      TRUE,      TRUE,          WLAN_PHY_OFDM,          12000,  10100,  0x0a,  0x00,   24,   6,    4,       1,     6,    6,    6,    6,      0},
     /*   18 Mb */ {  TRUE,      TRUE,      TRUE,          WLAN_PHY_OFDM,          18000,  14100,  0x0e,  0x00,   36,   6,    6,       2,     7,    7,    7,    7,      0},
     /*   24 Mb */ {  TRUE,      TRUE,      TRUE,          WLAN_PHY_OFDM,          24000,  17700,  0x09,  0x00,   48,   8,   10,       3,     8,    8,    8,    8,      0},
     /*   36 Mb */ {  TRUE,      TRUE,      TRUE,          WLAN_PHY_OFDM,          36000,  23700,  0x0d,  0x00,   72,   8,   14,       3,     9,    9,    9,    9,      0},
     /*   48 Mb */ {  TRUE,      TRUE,      TRUE,          WLAN_PHY_OFDM,          48000,  27400,  0x08,  0x00,   96,   8,   20,       3,    10,   10,   10,   10,      0},
     /*   54 Mb */ {  TRUE,      TRUE,      TRUE,          WLAN_PHY_OFDM,          54000,  30900,  0x0c,  0x00,  108,   8,   23,       3,    11,   11,   11,   11,      0},
     /*  6.5 Mb */ {  FALSE,     FALSE,     FALSE,         WLAN_PHY_HT_20_SS,       6500,   6400,  0x80,  0x00,    0,   4,    2,       3,    12,   28,   12,   28,   3216},
     /*   13 Mb */ {  TRUE_20,   TRUE_20,   TRUE_20,       WLAN_PHY_HT_20_SS,      13000,  12700,  0x81,  0x00,    1,   6,    4,       3,    13,   29,   13,   29,   6434},
     /* 19.5 Mb */ {  TRUE_20,   TRUE_20,   TRUE_20,       WLAN_PHY_HT_20_SS,      19500,  18800,  0x82,  0x00,    2,   6,    6,       3,    14,   30,   14,   30,   9650},
     /*   26 Mb */ {  TRUE_20,   TRUE_20,   TRUE_20,       WLAN_PHY_HT_20_SS,      26000,  25000,  0x83,  0x00,    3,   8,   10,       3,    15,   31,   15,   31,  12868},
     /*   39 Mb */ {  TRUE_20,   TRUE_20,   TRUE_20,       WLAN_PHY_HT_20_SS,      39000,  36700,  0x84,  0x00,    4,   8,   14,       3,    16,   32,   16,   32,  19304},
     /*   52 Mb */ {  FALSE,     TRUE_20,   TRUE_20,       WLAN_PHY_HT_20_SS,      52000,  48100,  0x85,  0x00,    5,   8,   20,       3,    17,   33,   17,   33,  25740},
     /* 58.5 Mb */ {  FALSE,     TRUE_20,   TRUE_20,       WLAN_PHY_HT_20_SS,      58500,  53500,  0x86,  0x00,    6,   8,   23,       3,    18,   34,   18,   34,  28956},
     /*   65 Mb */ {  FALSE,     TRUE_20,   FALSE,         WLAN_PHY_HT_20_SS,      65000,  59000,  0x87,  0x00,    7,   8,   25,       3,    19,   35,   19,   36,  32180},
     /*   13 Mb */ {  FALSE,     FALSE,     FALSE,         WLAN_PHY_HT_20_DS,      13000,  12700,  0x88,  0x00,    8,   4,    2,       3,    20,   37,   20,   37,   6430},
     /*   26 Mb */ {  FALSE,     FALSE,     FALSE,         WLAN_PHY_HT_20_DS,      26000,  24800,  0x89,  0x00,    9,   6,    4,       3,    21,   38,   21,   38,  12860},
     /*   39 Mb */ {  FALSE,     FALSE,     FALSE,         WLAN_PHY_HT_20_DS,      39000,  36600,  0x8a,  0x00,   10,   6,    6,       3,    22,   39,   22,   39,  19300},
     /*   52 Mb */ {  TRUE_20,   FALSE,     FALSE,         WLAN_PHY_HT_20_DS,      52000,  48100,  0x8b,  0x00,   11,   8,   10,       3,    23,   40,   23,   40,  25736},
     /*   78 Mb */ {  TRUE_20,   FALSE,     TRUE_20,       WLAN_PHY_HT_20_DS,      78000,  69500,  0x8c,  0x00,   12,   8,   14,       3,    24,   41,   24,   41,  38600},
     /*  104 Mb */ {  TRUE_20,   FALSE,     TRUE_20,       WLAN_PHY_HT_20_DS,     104000,  89500,  0x8d,  0x00,   13,   8,   20,       3,    25,   42,   25,   42,  51472},
     /*  117 Mb */ {  TRUE_20,   FALSE,     TRUE_20,       WLAN_PHY_HT_20_DS,     117000,  98900,  0x8e,  0x00,   14,   8,   23,       3,    26,   43,   26,   44,  57890},
     /*  130 Mb */ {  TRUE_20,   FALSE,     TRUE_20,       WLAN_PHY_HT_20_DS,     130000, 108300,  0x8f,  0x00,   15,   8,   25,       3,    27,   44,   27,   45,  64320},
     /* 13.5 Mb */ {  TRUE_40,   TRUE_40,   TRUE_40,       WLAN_PHY_HT_40_SS,      13500,  13200,  0x80,  0x00,    0,   8,    2,       3,    12,   28,   28,   28,   6684},
     /* 27.0 Mb */ {  TRUE_40,   TRUE_40,   TRUE_40,       WLAN_PHY_HT_40_SS,      27500,  25900,  0x81,  0x00,    1,   8,    4,       3,    13,   29,   29,   29,  13368},
     /* 40.5 Mb */ {  TRUE_40,   TRUE_40,   TRUE_40,       WLAN_PHY_HT_40_SS,      40500,  38600,  0x82,  0x00,    2,   8,    6,       3,    14,   30,   30,   30,  20052},
     /*   54 Mb */ {  TRUE_40,   TRUE_40,   TRUE_40,       WLAN_PHY_HT_40_SS,      54000,  49800,  0x83,  0x00,    3,   8,   10,       3,    15,   31,   31,   31,  26738},
     /*   81 Mb */ {  TRUE_40,   TRUE_40,   TRUE_40,       WLAN_PHY_HT_40_SS,      81500,  72200,  0x84,  0x00,    4,   8,   14,       3,    16,   32,   32,   32,  40104},
     /*  108 Mb */ {  FALSE,     TRUE_40,   TRUE_40,       WLAN_PHY_HT_40_SS,     108000,  92900,  0x85,  0x00,    5,   8,   20,       3,    17,   33,   33,   33,  53476},
     /* 121.5Mb */ {  FALSE,     TRUE_40,   TRUE_40,       WLAN_PHY_HT_40_SS,     121500, 102700,  0x86,  0x00,    6,   8,   23,       3,    18,   34,   34,   34,  60156},
     /*  135 Mb */ {  FALSE,     TRUE_40,   FALSE,         WLAN_PHY_HT_40_SS,     135000, 112000,  0x87,  0x00,    7,   8,   23,       3,    19,   35,   36,   36,  66840},
     /*  150 Mb */ {  FALSE,     TRUE_40,   FALSE,         WLAN_PHY_HT_40_SS_HGI, 150000, 122000,  0x87,  0x00,    7,   8,   25,       3,    19,   35,   36,   36,  74200},
     /*   27 Mb */ {  FALSE,     FALSE,     FALSE,         WLAN_PHY_HT_40_DS,      27000,  25800,  0x88,  0x00,    8,   8,    2,       3,    20,   37,   37,   37,  13360},
     /*   54 Mb */ {  FALSE,     FALSE,     FALSE,         WLAN_PHY_HT_40_DS,      54000,  49800,  0x89,  0x00,    9,   8,    4,       3,    21,   38,   38,   38,  26720},
     /*   81 Mb */ {  FALSE,     FALSE,     FALSE,         WLAN_PHY_HT_40_DS,      81000,  71900,  0x8a,  0x00,   10,   8,    6,       3,    22,   39,   39,   39,  40080},
     /*  108 Mb */ {  TRUE_40,   FALSE,     FALSE,         WLAN_PHY_HT_40_DS,     108000,  92500,  0x8b,  0x00,   11,   8,   10,       3,    23,   40,   40,   40,  53440},
     /*  162 Mb */ {  TRUE_40,   FALSE,     TRUE_40,       WLAN_PHY_HT_40_DS,     162000, 130300,  0x8c,  0x00,   12,   8,   14,       3,    24,   41,   41,   41,  80160},
     /*  216 Mb */ {  TRUE_40,   FALSE,     TRUE_40,       WLAN_PHY_HT_40_DS,     216000, 162800,  0x8d,  0x00,   13,   8,   20,       3,    25,   42,   42,   42, 106880},
     /*  243 Mb */ {  TRUE_40,   FALSE,     TRUE_40,       WLAN_PHY_HT_40_DS,     243000, 178200,  0x8e,  0x00,   14,   8,   23,       3,    26,   43,   43,   43, 120240},
     /*  270 Mb */ {  TRUE_40,   FALSE,     TRUE_40,       WLAN_PHY_HT_40_DS,     270000, 192100,  0x8f,  0x00,   15,   8,   23,       3,    27,   44,   45,   45, 133600},
     /*  300 Mb */ {  TRUE_40,   FALSE,     TRUE_40,       WLAN_PHY_HT_40_DS_HGI, 300000, 207000,  0x8f,  0x00,   15,   8,   25,       3,    27,   44,   45,   45, 148400},
    },
    50,  /* probe interval */
    50,  /* rssi reduce interval */
    WLAN_RC_HT_FLAG,  /* Phy rates allowed initially */
};

#ifndef ATH_NO_5G_SUPPORT

static RATE_TABLE_11N ar5416_11aRateTable = {
    8,  /* number of rates */
    {/*              Multi-strm Single-strm                                                short     dot11   ctrl  RssiAck  RssiAck  */
     /*              valid      valid                                 Kbps    uKbps   RC   Preamble  Rate    Rate  ValidMin DeltaMin */
     /*   6 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   6000,   5400,  0x0b,  0x00, (0x80|12),  0,     2,       1,     0,    0},
     /*   9 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   9000,   7800,  0x0f,  0x00,        18,  0,     3,       1,     1,    0},
     /*  12 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  12000,  10000,  0x0a,  0x00, (0x80|24),  2,     4,       2,     2,    0},
     /*  18 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  18000,  13900,  0x0e,  0x00,        36,  2,     6,       2,     3,    0},
     /*  24 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  24000,  17300,  0x09,  0x00, (0x80|48),  4,    10,       3,     4,    0},
     /*  36 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  36000,  23000,  0x0d,  0x00,        72,  4,    14,       3,     5,    0},
     /*  48 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  48000,  27400,  0x08,  0x00,        96,  4,    19,       3,     6,    0},
     /*  54 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  54000,  29300,  0x0c,  0x00,       108,  4,    23,       3,     7,    0},
    },    
    50,  /* probe interval */    
    50,  /* rssi reduce interval */
    0,   /* Phy rates allowed initially */
};


static RATE_TABLE_11N ar5416_11aRateTable_Half = {
    8,  /* number of rates */
    {/*              Multi-strm Single-strm                                                short    dot11   ctrl  RssiAck  RssiAck  */
     /*              valid      valid                                 Kbps   uKbps   RC   Preamble  Rate    Rate  ValidMin DeltaMin */
     /*   6 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   3000,  2700,  0x0b,  0x00,  (0x80|6),   0,     2,       1,     0,    0},
     /*   9 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   4500,  3900,  0x0f,  0x00,         9,   0,     3,       1,     1,    0},
     /*  12 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   6000,  5000,  0x0a,  0x00, (0x80|12),   2,     4,       2,     2,    0},
     /*  18 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   9000,  6950,  0x0e,  0x00,        18,   2,     6,       2,     3,    0},
     /*  24 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  12000,  8650,  0x09,  0x00, (0x80|24),   4,    10,       3,     4,    0},
     /*  36 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  18000, 11500,  0x0d,  0x00,        36,   4,    14,       3,     5,    0},
     /*  48 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  24000, 13700,  0x08,  0x00,        48,   4,    19,       3,     6,    0},
     /*  54 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  27000, 14650,  0x0c,  0x00,        54,   4,    23,       3,     7,    0},
    },    
    50,  /* probe interval */    
    50,  /* rssi reduce interval */
    0,   /* Phy rates allowed initially */
};

static RATE_TABLE_11N ar5416_11aRateTable_Quarter = {
    8,  /* number of rates */
    {/*              Multi-strm Single-strm                                                short    dot11   ctrl  RssiAck  RssiAck  */
     /*              valid      valid                                 Kbps   uKbps   RC   Preamble  Rate    Rate  ValidMin DeltaMin */
     /*   6 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   1500,  1350,  0x0b,  0x00,  (0x80|3),   0,     2,       1,     0,    0},
     /*   9 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   2250,  1950,  0x0f,  0x00,         4,   0,     3,       1,     1,    0},
     /*  12 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   3000,  2500,  0x0a,  0x00,  (0x80|6),   2,     4,       2,     2,    0},
     /*  18 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   4500,  3475,  0x0e,  0x00,         9,   2,     6,       2,     3,    0},
     /*  24 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   6000,  4325,  0x09,  0x00, (0x80|12),   4,    10,       3,     4,    0},
     /*  36 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,   9000,  5750,  0x0d,  0x00,        18,   4,    14,       3,     5,    0},
     /*  48 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  12000,  6850,  0x08,  0x00,        24,   4,    19,       3,     6,    0},
     /*  54 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_OFDM,  13500,  7325,  0x0c,  0x00,        27,   4,    23,       3,     7,    0},
    },    
    50,  /* probe interval */    
    50,  /* rssi reduce interval */
    0,   /* Phy rates allowed initially */
};

#endif /* #ifndef ATH_NO_5G_SUPPORT */

static RATE_TABLE_11N ar5416_TurboRateTable = {
    8,  /* number of rates */
    {/*              Multi-strm Single-strm                                                short    dot11   ctrl  RssiAck  RssiAck  */
     /*              valid      valid                                 Kbps   uKbps   RC   Preamble  Rate    Rate  ValidMin DeltaMin */
     /*   6 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_TURBO,  6000,  5400,  0x0b,  0x00, (0x80|12),   0,     2,       1,     0,    0},
     /*   9 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_TURBO,  9000,  7800,  0x0f,  0x00,        18,   0,     4,       1,     1,    0},
     /*  12 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_TURBO, 12000, 10000,  0x0a,  0x00, (0x80|24),   2,     7,       2,     2,    0},
     /*  18 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_TURBO, 18000, 13900,  0x0e,  0x00,        36,   2,     9,       2,     3,    0},
     /*  24 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_TURBO, 24000, 17300,  0x09,  0x00, (0x80|48),   4,    14,       3,     4,    0},
     /*  36 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_TURBO, 36000, 23000,  0x0d,  0x00,        72,   4,    17,       3,     5,    0},
     /*  48 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_TURBO, 48000, 27400,  0x08,  0x00,        96,   4,    22,       3,     6,    0},
     /*  54 Mb */ {  TRUE,       TRUE,     FALSE,    WLAN_PHY_TURBO, 54000, 29300,  0x0c,  0x00,       108,   4,    26,       3,     7,    0},
    },    
    50, /* probe interval */    
    50,  /* rssi reduce interval */
    0,   /* Phy rates allowed initially */
};

/* Venice TODO: roundUpRate() is broken when the rate table does not represent rates
 * in increasing order  e.g.  5.5, 11, 6, 9.    
 * An average rate of 6 Mbps will currently map to 11 Mbps. 
 */
static RATE_TABLE_11N ar5416_11gRateTable = {
    12,  /* number of rates */
    {/*              Multi-strm Single-strm                                                short    dot11  ctrl  RssiAck  RssiAck  */
     /*              valid      valid                                 Kbps   uKbps   RC   Preamble  Rate   Rate  ValidMin DeltaMin */
     /*   1 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_CCK,   1000,   900,  0x1b,  0x00,      2,     0,     0,       1,     0,    0},
     /*   2 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_CCK,   2000,  1900,  0x1a,  0x04,      4,     1,     1,       1,     1,    0},
     /* 5.5 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_CCK,   5500,  4900,  0x19,  0x04,     11,     2,     2,       2,     2,    0},
     /*  11 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_CCK,  11000,  8100,  0x18,  0x04,     22,     3,     3,       2,     3,    0},
     /*   6 Mb */ {  FALSE,      FALSE,     FALSE,    WLAN_PHY_OFDM,  6000,  5400,  0x0b,  0x00,     12,     4,     2,       1,     4,    0},
     /*   9 Mb */ {  FALSE,      FALSE,     FALSE,    WLAN_PHY_OFDM,  9000,  7800,  0x0f,  0x00,     18,     4,     3,       1,     5,    0},
     /*  12 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_OFDM, 12000, 10000,  0x0a,  0x00,     24,     6,     4,       1,     6,    0},
     /*  18 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_OFDM, 18000, 13900,  0x0e,  0x00,     36,     6,     6,       2,     7,    0},
     /*  24 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_OFDM, 24000, 17300,  0x09,  0x00,     48,     8,    10,       3,     8,    0},
     /*  36 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_OFDM, 36000, 23000,  0x0d,  0x00,     72,     8,    14,       3,     9,    0},
     /*  48 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_OFDM, 48000, 27400,  0x08,  0x00,     96,     8,    19,       3,    10,    0},
     /*  54 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_OFDM, 54000, 29300,  0x0c,  0x00,    108,     8,    23,       3,    11,    0},
    },    
    50,  /* probe interval */    
    50,  /* rssi reduce interval */    
    0,   /* Phy rates allowed initially */    
};

static RATE_TABLE_11N ar5416_11bRateTable = {
    4,  /* number of rates */
    {/*              Multi-strm Single-strm                                                short    dot11   ctrl  RssiAck  RssiAck  */
     /*              valid      valid                                Kbps   uKbps   RC   Preamble   Rate    Rate  ValidMin DeltaMin */
     /*   1 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_CCK,  1000,   900,  0x1b,   0x00, (0x80| 2),   0,    0,       1,     0,    0},
     /*   2 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_CCK,  2000,  1800,  0x1a,   0x04, (0x80| 4),   1,    1,       1,     1,    0},
     /* 5.5 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_CCK,  5500,  4300,  0x19,   0x04, (0x80|11),   1,    2,       2,     2,    0},
     /*  11 Mb */ {  TRUE,       TRUE,      FALSE,    WLAN_PHY_CCK, 11000,  7100,  0x18,   0x04, (0x80|22),   1,    4,     100,     3,    0},
    },    
    100, /* probe interval */    
    100, /* rssi reduce interval */    
    0,   /* Phy rates allowed initially */
};

void
ar5416SetupRateTables(void)
{
}

void
ar5416AttachRateTables(struct atheros_softc *sc)
{
    /*
     * Attach device specific rate tables; for ar5212.
     * 11a static turbo and 11g static turbo share the same table.
     * Dynamic turbo uses combined rate table.
     */
    sc->hwRateTable[WIRELESS_MODE_11b]   		= &ar5416_11bRateTable;

#ifndef ATH_NO_5G_SUPPORT
    sc->hwRateTable[WIRELESS_MODE_11a]   		= &ar5416_11aRateTable;
#endif
    sc->hwRateTable[WIRELESS_MODE_11g]   		= &ar5416_11gRateTable;
    sc->hwRateTable[WIRELESS_MODE_108a]  		= &ar5416_TurboRateTable;
    sc->hwRateTable[WIRELESS_MODE_108g]  		= &ar5416_TurboRateTable;
#ifndef ATH_NO_5G_SUPPORT
    sc->hwRateTable[WIRELESS_MODE_11NA_HT20]		= &ar5416_11naRateTable;
#endif
    sc->hwRateTable[WIRELESS_MODE_11NG_HT20]   		= &ar5416_11ngRateTable;
#ifndef ATH_NO_5G_SUPPORT
    sc->hwRateTable[WIRELESS_MODE_11NA_HT40PLUS]   	= &ar5416_11naRateTable;
    sc->hwRateTable[WIRELESS_MODE_11NA_HT40MINUS]   	= &ar5416_11naRateTable;
#endif
    sc->hwRateTable[WIRELESS_MODE_11NG_HT40PLUS]   	= &ar5416_11ngRateTable;
    sc->hwRateTable[WIRELESS_MODE_11NG_HT40MINUS]   	= &ar5416_11ngRateTable;
}

#ifndef ATH_NO_5G_SUPPORT
void
ar5416SetQuarterRateTable(struct atheros_softc *sc)
{
    sc->hwRateTable[WIRELESS_MODE_11a] = &ar5416_11aRateTable_Quarter;
    return;
}

void
ar5416SetHalfRateTable(struct atheros_softc *sc)
{
    sc->hwRateTable[WIRELESS_MODE_11a] = &ar5416_11aRateTable_Half;
    return;
}

void
ar5416SetFullRateTable(struct atheros_softc *sc)
{
    sc->hwRateTable[WIRELESS_MODE_11a]   = &ar5416_11aRateTable;
    return;
}
#endif /* #ifndef ATH_NO_5G_SUPPORT */
