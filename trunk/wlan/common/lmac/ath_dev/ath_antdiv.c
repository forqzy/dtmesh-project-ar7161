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

#include "ath_internal.h"
#ifdef ATH_SUPPORT_DFS
#include "dfs.h"
#endif

#include "ath_antdiv.h"

#define REMOVE_PKT_LOG
#ifndef REMOVE_PKT_LOG
#include "pktlog.h"
extern struct ath_pktlog_funcs *g_pktlog_funcs;
#define MAX_LOG 128
#endif

void
ath_slow_ant_div_init(struct ath_antdiv *antdiv, struct ath_softc *sc, int32_t rssitrig,
                      u_int32_t min_dwell_time, u_int32_t settle_time)
{
    antdiv->antdiv_sc = sc;
    antdiv->antdiv_rssitrig = rssitrig;
    antdiv->antdiv_min_dwell_time = min_dwell_time * 1000;
    antdiv->antdiv_settle_time = settle_time * 1000;
}

void
ath_slow_ant_div_start(struct ath_antdiv *antdiv, u_int8_t num_antcfg, const u_int8_t *bssid)
{
    antdiv->antdiv_num_antcfg   = num_antcfg < ATH_ANT_DIV_MAX_CFG ? num_antcfg : ATH_ANT_DIV_MAX_CFG;
    antdiv->antdiv_state        = ATH_ANT_DIV_IDLE;
    antdiv->antdiv_curcfg       = 0;
    antdiv->antdiv_bestcfg      = 0;
    antdiv->antdiv_laststatetsf = 0;
    antdiv->rssiThrLow          = 10;
    antdiv->rssiThrHigh         = 50;
    antdiv->bytesThrRx          = 1514;          
    antdiv->antdiv_min_dwell_time = 60 * 1000;
    antdiv->antdiv_settle_time    =  2 * 1000;

    OS_MEMCPY(antdiv->antdiv_bssid, bssid, sizeof(antdiv->antdiv_bssid));
    
    antdiv->antdiv_start = 1;
}

void
ath_slow_ant_div_stop(struct ath_antdiv *antdiv)
{
    antdiv->antdiv_start = 0;
}

static void find_max_sdev(int8_t *val_in, u_int8_t num_val, u_int8_t *max_index, u_int16_t *sdev)
{
    int8_t   *max_val, *val = val_in, *val_last = val_in + num_val;

    if (val_in && num_val) {
        max_val = val;
        while (++val != val_last) {
            if (*val > *max_val)
                max_val = val;
        };

        if (max_index)
            *max_index = (u_int8_t)(max_val - val_in);

        if (sdev) {
            val = val_in;
            *sdev = 0;
            do {
                *sdev += (*max_val - *val);
            } while (++val != val_last);
        }
    }
}

void
ath_slow_ant_div(struct ath_antdiv *antdiv, struct ieee80211_frame *wh, struct ath_rx_status *rx_stats)
{
    struct ath_softc *sc = antdiv->antdiv_sc;
    struct ath_hal   *ah = sc->sc_ah;
    u_int64_t curtsf = 0;
    u_int8_t  bestcfg, bestcfg_chain[ATH_ANT_DIV_MAX_CHAIN], curcfg = antdiv->antdiv_curcfg;
    u_int16_t sdev_chain[ATH_ANT_DIV_MAX_CHAIN];
#ifndef REMOVE_PKT_LOG
    char logtext[MAX_LOG];
#endif

    if (antdiv->antdiv_start && IEEE80211_IS_BEACON(wh) && ATH_ADDR_EQ(wh->i_addr3, antdiv->antdiv_bssid)){
            antdiv->antdiv_lastbrssictl[0][curcfg] = rx_stats->rs_rssi_ctl0;
            antdiv->antdiv_lastbrssictl[1][curcfg] = rx_stats->rs_rssi_ctl1;
            antdiv->antdiv_lastbrssictl[2][curcfg] = rx_stats->rs_rssi_ctl2;
/* not yet
            antdiv->antdiv_lastbrssi[curcfg] = rx_stats->rs_rssi;
            antdiv->antdiv_lastbrssiext[0][curcfg] = rx_stats->rs_rssi_ext0;
            antdiv->antdiv_lastbrssiext[1][curcfg] = rx_stats->rs_rssi_ext1;
            antdiv->antdiv_lastbrssiext[2][curcfg] = rx_stats->rs_rssi_ext2;
*/
            antdiv->antdiv_lastbtsf[curcfg] = ath_hal_gettsf64(sc->sc_ah);
            curtsf = antdiv->antdiv_lastbtsf[curcfg];
    } else {
        return;
    }

    switch (antdiv->antdiv_state) {
    case ATH_ANT_DIV_IDLE:
        if ((curtsf - antdiv->antdiv_laststatetsf) > antdiv->antdiv_min_dwell_time) {
            int8_t min_rssi = antdiv->antdiv_lastbrssictl[0][curcfg];
            int      i;
            for (i = 1; i < ATH_ANT_DIV_MAX_CHAIN; i++) {
                if (antdiv->antdiv_lastbrssictl[i][curcfg] != (-128)) {
                    if (antdiv->antdiv_lastbrssictl[i][curcfg] < min_rssi) {
                        min_rssi = antdiv->antdiv_lastbrssictl[i][curcfg];
                    }
                }
            }
            if (min_rssi < antdiv->antdiv_rssitrig) {
                curcfg ++;
                if (curcfg == antdiv->antdiv_num_antcfg) {
                    curcfg = 0;
                }

                if (HAL_OK == ath_hal_selectAntConfig(ah, curcfg)) {
                    antdiv->antdiv_bestcfg = antdiv->antdiv_curcfg;
                    antdiv->antdiv_curcfg = curcfg;
                    antdiv->antdiv_laststatetsf = curtsf;
                    antdiv->antdiv_state = ATH_ANT_DIV_SCAN;
#ifndef REMOVE_PKT_LOG
                    /* do pktlog */
                    {
                        ath_sprintf(logtext, MAX_LOG, "SlowAntDiv: select_antcfg = %d\n", antdiv->antdiv_curcfg);
                        ath_log_text(sc, logtext, 0); 
                    }
#endif
                }
            }
        }
        break;
    
    case ATH_ANT_DIV_SCAN:
        if((curtsf - antdiv->antdiv_laststatetsf) < antdiv->antdiv_settle_time)
            break;

        curcfg ++;
        if (curcfg == antdiv->antdiv_num_antcfg) {
            curcfg = 0;
        }

        if (curcfg == antdiv->antdiv_bestcfg) {
            u_int16_t *max_sdev;
            int       i;
            for (i = 0; i < ATH_ANT_DIV_MAX_CHAIN; i++) {
                find_max_sdev(&antdiv->antdiv_lastbrssictl[i][0], antdiv->antdiv_num_antcfg, &bestcfg_chain[i], &sdev_chain[i]);
#ifndef REMOVE_PKT_LOG
                /* do pktlog */
                {
                    ath_sprintf(logtext, MAX_LOG, "SlowAntDiv: best cfg (chain%d) = %d (cfg0:%d, cfg1:%d, sdev:%d)\n", i, 
                            bestcfg_chain[i], antdiv->antdiv_lastbrssictl[i][0],
                            antdiv->antdiv_lastbrssictl[i][1], sdev_chain[i]);
                    ath_log_text(sc, logtext, 0); 
                }
#endif
            }

            max_sdev = sdev_chain;
            for (i = 1; i < ATH_ANT_DIV_MAX_CHAIN; i++) {
                if (*(sdev_chain + i) > *max_sdev) {
                    max_sdev = sdev_chain + i;
                }
            }
            bestcfg = bestcfg_chain[(max_sdev - sdev_chain)];

            if (bestcfg != antdiv->antdiv_curcfg) {
                if (HAL_OK == ath_hal_selectAntConfig(ah, bestcfg)) {
                    antdiv->antdiv_bestcfg = bestcfg;
                    antdiv->antdiv_curcfg = bestcfg;
#ifndef REMOVE_PKT_LOG
                    /* do pktlog */
                    {
                        ath_sprintf(logtext, MAX_LOG, "SlowAntDiv: select best cfg = %d\n", antdiv->antdiv_curcfg);
                        ath_log_text(sc, logtext, 0); 
                    }
#endif
                }
            }
            antdiv->antdiv_laststatetsf = curtsf;
            antdiv->antdiv_state = ATH_ANT_DIV_IDLE;

        } else {
            if (HAL_OK == ath_hal_selectAntConfig(ah, curcfg)) {
                antdiv->antdiv_curcfg = curcfg;
                antdiv->antdiv_laststatetsf = curtsf;
                antdiv->antdiv_state = ATH_ANT_DIV_SCAN;
#ifndef REMOVE_PKT_LOG
                /* do pktlog */
                {
                    ath_sprintf(logtext, MAX_LOG, "SlowAntDiv: select ant cfg = %d\n", antdiv->antdiv_curcfg);
                    ath_log_text(sc, logtext, 0); 
                }
#endif
            }
        }

        break;
    }
}

void
ath_ant_div_comb_init(struct ath_antcomb *antcomb, struct ath_softc *sc)
{
    struct ath_hal *ah = sc->sc_ah;


    printk("ANT_DIV_COMB: %s Called \n", __func__);
	if (ath_hal_AntDivCombSupport(ah))
	{
        printk("HAL supports ANTENNA_DIVERSITY");       
	}
    OS_MEMZERO(antcomb, sizeof(struct ath_antcomb));
    antcomb->antcomb_sc = sc;

}

void 
ath_ant_div_comb_scan(struct ath_antcomb *antcomb, struct ath_rx_status *rx_stats)
{
    struct ath_softc *sc = antcomb->antcomb_sc;
    struct ath_hal *ah = sc->sc_ah;
    int alt_ratio, alt_rssi_avg, main_rssi_avg, curr_alt_set, curr_main_set, curr_bias;
    int8_t main_rssi = rx_stats->rs_rssi_ctl0;
    int8_t alt_rssi = rx_stats->rs_rssi_ctl1;
    int8_t rx_ant_conf,  main_ant_conf;
    u_int8_t rx_pkt_moreaggr = rx_stats->rs_moreaggr;
    u_int8_t short_scan = 0;
    HAL_ANT_COMB_CONFIG div_ant_conf;

    /* Data Collection */
    rx_ant_conf = (rx_stats->rs_rssi_ctl2 >> 4) & 0x3;
    main_ant_conf= (rx_stats->rs_rssi_ctl2 >> 2) & 0x3;

    /* Only Record packet with alt_rssi is positive */
    if (alt_rssi > 0) {
        antcomb->total_pkt_count++;
        antcomb->main_total_rssi += main_rssi;
        antcomb->alt_total_rssi  += alt_rssi;
        if (main_ant_conf == rx_ant_conf) {          
           antcomb->main_recv_cnt++;
        }
        else {
           antcomb->alt_recv_cnt++;
        }
    } 


    /* short scan is always zero for AP*/
    short_scan = 0;
    /* Diversity and Combining algorithm */
    if (((antcomb->total_pkt_count < ANT_DIV_COMB_MAX_PKTCOUNT) ||
         (rx_pkt_moreaggr != 0)) &&
         (short_scan == 0)) {
        return;
    }

    /* When collect more than 512 packets, and finish aggregation packet
     * Or short scan=1.
     * Start to do decision
     */
    if (antcomb->total_pkt_count == 0) {
        alt_ratio = 0;
        main_rssi_avg = 0;
        alt_rssi_avg = 0;
    } else {
        alt_ratio = ((antcomb->alt_recv_cnt * 100) / antcomb->total_pkt_count);
        main_rssi_avg = (antcomb->main_total_rssi / antcomb->total_pkt_count);
        alt_rssi_avg  = (antcomb->alt_total_rssi / antcomb->total_pkt_count);
    }

    /*Get current LNA setting */
    ath_hal_getAntDivCombConf(ah, &div_ant_conf);
    curr_alt_set  = div_ant_conf.alt_lna_conf; 
    curr_main_set = div_ant_conf.main_lna_conf; 
    curr_bias     = div_ant_conf.fast_div_bias; 


    /*Get LNA1 LNA2 RSSI*/ 
    if (curr_main_set == HAL_ANT_DIV_COMB_LNA2) {
              antcomb->rssi_lna2 = main_rssi_avg;
              antcomb->rssi_lna1 = alt_rssi_avg;              
       } else if (curr_main_set == HAL_ANT_DIV_COMB_LNA1) {
              antcomb->rssi_lna1 = main_rssi_avg;
              antcomb->rssi_lna2 = alt_rssi_avg;              
       } 
    /*Choose Main LNA base on RSSI Value*/
    if (antcomb->rssi_lna2> (antcomb->rssi_lna1 + ANT_DIV_COMB_LNA1_LNA2_SWITCH_DELTA)) {
            div_ant_conf.main_lna_conf = HAL_ANT_DIV_COMB_LNA2;
            div_ant_conf.alt_lna_conf  = HAL_ANT_DIV_COMB_LNA1;    
    } else {
            div_ant_conf.main_lna_conf = HAL_ANT_DIV_COMB_LNA1;
            div_ant_conf.alt_lna_conf  = HAL_ANT_DIV_COMB_LNA2;    
    }

            goto div_comb_done;
 




div_comb_done:
      /*Find bias value*/
      switch (div_ant_conf.main_lna_conf<<4 | div_ant_conf.alt_lna_conf) {     
          case (0x12): //LNA2 LNA1
              div_ant_conf.fast_div_bias = 0x2;
              break;       
          case (0x21): //LNA1 LNA2
              div_ant_conf.fast_div_bias = 0x0;
              break;
          default:
              break;
        }    

    /* Write new setting to register*/
    ath_hal_setAntDivCombConf(ah, &div_ant_conf); 


/* SHAN TBR */
    //printk("ANT_DIV_COMB: \n");
    //printk("ANT_DIV_COMB pkt num, total: %3d, main: %3d, alt: %3d \n",
    //       antcomb->total_pkt_count, antcomb->main_recv_cnt, antcomb->alt_recv_cnt);
    //printk("ANT_DIV_COMB RSSI main=%d alt=%d main_avg= %3d alt_avg= %3d \n", curr_main_set,curr_alt_set, main_rssi_avg,alt_rssi_avg);
    //printk("ANT_DIV_COMB Current setting main=%d alt=%d bias =0x%x \n",curr_main_set,curr_alt_set,curr_bias);
    //printk("ANT_DIV_COMB next_main=%d next_alt=%d next_bias =0x%x \n",
    //    div_ant_conf.main_lna_conf,div_ant_conf.alt_lna_conf,div_ant_conf.fast_div_bias);


    /* Clear variable */
    antcomb->total_pkt_count = 0;
    antcomb->main_total_rssi = 0;
    antcomb->alt_total_rssi = 0;
    antcomb->main_recv_cnt = 0;
    antcomb->alt_recv_cnt = 0;
}
//modify diversity antenna
#ifdef AP_SLOW_ANT_DIV
static
OS_TIMER_FUNC(ath_slow_ant_div_ap)
{
    struct ath_softc    *sc;
    struct ath_antdiv   *antdiv;
    struct ath_hal      *ah;
    struct ath_node *an = NULL;
    u_int8_t            curcfg, scnt, nscnt;

    OS_GET_TIMER_ARG(sc, struct ath_softc *);

    antdiv = &sc->sc_antdiv;

    if (!sc->sc_nvaps || sc->sc_invalid ) {
        OS_SET_TIMER(&sc->sc_slow_ant_div, antdiv->antdiv_min_dwell_time /* ms */);
        return;
    }
#ifdef ATH_SUPPORT_DFS
    if (sc->sc_dfs && sc->sc_dfs->sc_dfswait) {
        OS_SET_TIMER(&sc->sc_slow_ant_div, antdiv->antdiv_min_dwell_time /* ms */);
        return;
    }
#endif

    ah     = sc->sc_ah;

    if (!antdiv->antdiv_flags) {
        antdiv->antdiv_flags = 1;
        OS_SET_TIMER(&sc->sc_slow_ant_div, antdiv->antdiv_settle_time /* ms */);
        return;
    }

    scnt = nscnt = 0;
    curcfg = antdiv->antdiv_curcfg;

    LIST_FOREACH(an, &sc->sc_nt, an_le) {
#if 0
        if (!an) {
            continue;
        }
#endif

        if (an->an_antdiv_bytes[curcfg] > antdiv->bytesThrRx) { // is this active??
           if (an->an_antdiv_rxcnt[curcfg]) {
               an->an_antdiv_rssictl[curcfg] /= an->an_antdiv_rxcnt[curcfg];
           }
           if ((an->an_antdiv_rssictl[curcfg] > antdiv->rssiThrLow) &&
               (an->an_antdiv_rssictl[curcfg] < antdiv->rssiThrHigh)) {
               nscnt++;
           } else {
               scnt ++;
           }
        }

        an->an_antdiv_rssictl[curcfg] = 0;
        an->an_antdiv_bytes[curcfg] = 0;
        an->an_antdiv_rxcnt[curcfg] = 0;
    }

    antdiv->laststatesuff[curcfg] = scnt;

    if (nscnt <= scnt) {
        curcfg = !curcfg;

        if (antdiv->antdiv_flags == 1) {
            if (HAL_OK == ath_hal_selectAntConfig(ah, curcfg)) {
                antdiv->antdiv_curcfg = curcfg;
                OS_SET_TIMER(&sc->sc_slow_ant_div, antdiv->antdiv_settle_time /* ms */);
                antdiv->antdiv_flags = 2;
                return;
            }
        } else {
            if (antdiv->laststatesuff[curcfg] < antdiv->laststatesuff[!curcfg]) {
                if (HAL_OK == ath_hal_selectAntConfig(ah, curcfg)) {
                    antdiv->antdiv_curcfg = curcfg;
                }
            }
        }
    }

    antdiv->antdiv_bestcfg = antdiv->antdiv_curcfg;
    antdiv->antdiv_flags = 0;
    OS_SET_TIMER(&sc->sc_slow_ant_div, antdiv->antdiv_min_dwell_time /* ms */);
}


/* Control functions to enable slow antenna diversity for AP */
void ath_slow_ant_div_setenable(struct ath_softc *sc, u_int32_t val)
{
    struct ath_antdiv* antdiv = &sc->sc_antdiv;

    if(!antdiv) return;

    if (!val && antdiv->enabled) {
	ath_slow_ant_div_stop(antdiv);
	OS_CANCEL_TIMER(&sc->sc_slow_ant_div);
    }

    if (val && !antdiv->enabled)
    {
        struct ath_hal *ah = sc->sc_ah;
        u_int32_t num_antcfg;

        /* Start slow antenna diversity */
        if (sc->sc_curchan.channelFlags & CHANNEL_5GHZ)
            ath_hal_getcapability(ah, HAL_CAP_ANT_CFG_5GHZ, 0, &num_antcfg);
        else
            ath_hal_getcapability(ah, HAL_CAP_ANT_CFG_2GHZ, 0, &num_antcfg);

	num_antcfg = 2;
        if (num_antcfg > 1) {
            ath_slow_ant_div_init(&sc->sc_antdiv, sc, 0, 60, 2);
            ath_slow_ant_div_start(&sc->sc_antdiv, num_antcfg, sc->sc_curbssid);
            OS_INIT_TIMER(sc->sc_osdev, &sc->sc_slow_ant_div, ath_slow_ant_div_ap, sc);
            OS_SET_TIMER(&sc->sc_slow_ant_div, antdiv->antdiv_min_dwell_time /* ms */);
        }
    }

    antdiv->enabled = !!val;

}

#endif
