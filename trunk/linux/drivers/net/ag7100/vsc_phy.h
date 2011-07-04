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

#ifndef _VSC_PHY_H
#define _VSC_PHY_H

#define VSC_MII_MODE_STATUS     0x1
#define AUTONEG_COMPLETE        (1 << 5)
#define LINK_UP                 (1 << 2)

#define VSC_AUX_CTRL_STATUS     0x1c
#define FDX                     (1 << 5)
#define SPEED_STATUS            (3 << 3)

void 		 vsc_phy_setup(int unit);
int 		 vsc_phy_is_up(int unit);
int 		 vsc_phy_is_fdx(int unit);
int 		 vsc_phy_speed(int unit);
unsigned int vsc_phy_get_link_status(int unit, 
                                        int *link, 
                                        int *fdx, 
                                        ag7100_phy_speed_t *speed, 
                                        unsigned int *cfg);

int			 vsc_phy_print_link_status(int unit);

#endif
