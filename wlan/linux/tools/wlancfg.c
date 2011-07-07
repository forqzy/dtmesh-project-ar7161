#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <getopt.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <endian.h>
#include <fcntl.h>
#include <linux/if_bridge.h>
#include <mtd/mtd-user.h>
#include <asm/param.h>
#include <sys/syscall.h>
#include <sys/stat.h>
#include <unistd.h>
#include <mntent.h>
#include <dirent.h>

#include "iwlib.h"
#include "wireless.h"
/*
 * Linux uses __BIG_ENDIAN and __LITTLE_ENDIAN while BSD uses _foo
 * and an explicit _BYTE_ORDER.  Sorry, BSD got there first--define
 * things in the BSD way...
 */
#define _LITTLE_ENDIAN  1234    /* LSB first: i386, vax */
#define _BIG_ENDIAN     4321    /* MSB first: 68000, ibm, net */
#include <asm/byteorder.h>
#if defined(__LITTLE_ENDIAN)
#define _BYTE_ORDER     _LITTLE_ENDIAN
#elif defined(__BIG_ENDIAN)
#define _BYTE_ORDER     _BIG_ENDIAN
#else
#error "Please fix asm/byteorder.h"
#endif

#include "net80211/ieee80211.h"
#include "net80211/ieee80211_crypto.h"
#include "net80211/ieee80211_ioctl.h"

#ifdef CONFIG_ATHR_ETH_EXTENSION
#include "athrs_ioctl.h"
#endif /* #ifdef CONFIG_ATHR_ETH_EXTENSION */

#define IEEE80211_NODE_TURBOP   0x0001          /* Turbo prime enable */
#define IEEE80211_NODE_COMP     0x0002          /* Compresssion enable */
#define IEEE80211_NODE_FF       0x0004          /* Fast Frame capable */
#define IEEE80211_NODE_XR       0x0008          /* Atheros WME enable */
#define IEEE80211_NODE_AR       0x0010          /* AR capable */
#define IEEE80211_NODE_BOOST    0x0080 

#define KEY_IS_WPA      0x00
#define KEY_IS_WEP      0x01
#define SILENT_ERROR    0x02

#define ATHCFG_NVRAM_WLAN_CFG_SIZE 1500
#define ATHCFG_NVRAM_WPS_OFFSET (ATHCFG_NVRAM_WLAN_CFG_SIZE + 1)
#define ATHCFG_NVRAM_WPS_CFG_SIZE 2500

struct bridge;
struct bridge_info;
struct fdb_entry;
struct port;
struct port_info;

struct bridge_id {
        unsigned char prio[2];
        unsigned char addr[6];
};

struct bridge_info {
        struct bridge_id designated_root;
        struct bridge_id bridge_id;
        int root_path_cost;
        struct timeval max_age;
        struct timeval hello_time;
        struct timeval forward_delay;
        struct timeval bridge_max_age;
        struct timeval bridge_hello_time;
        struct timeval bridge_forward_delay;
        unsigned topology_change:1;
        unsigned topology_change_detected:1;
        int root_port;
        unsigned stp_enabled:1;
        struct timeval ageing_time;
        struct timeval gc_interval;
        struct timeval hello_timer_value;
        struct timeval tcn_timer_value; 
        struct timeval topology_change_timer_value;
        struct timeval gc_timer_value;
};

struct bridge {
        struct bridge *next;
        int ifindex;
        char ifname[IFNAMSIZ];
        struct port *firstport;
        struct port *ports[256];
        struct bridge_info info;
};

struct fdb_entry {
        u_int8_t mac_addr[6];
        int port_no;
        unsigned is_local:1;
        struct timeval ageing_timer_value;
};

struct port_info {
        struct bridge_id designated_root;
        struct bridge_id designated_bridge;
        u_int16_t port_id;
        u_int16_t designated_port;
        int path_cost;
        int designated_cost;
        int state;
        unsigned top_change_ack:1;
        unsigned config_pending:1;
        struct timeval message_age_timer_value;
        struct timeval forward_delay_timer_value;
        struct timeval hold_timer_value;
};

struct port {
        struct port *next;

        int index;
        int ifindex;
        struct bridge *parent;
        struct port_info info;
};

static int br_socket_fd=-1;
static struct bridge *bridge_list;
static iwprivargs *wlan_hw_if_privcmd_list = NULL;
static iwprivargs *wlan_vap_if_privcmd_list = NULL;
static int iwpriv_hw_if_cmd_numbers=0;
static int iwpriv_vap_if_cmd_numbers=0;

static char cmdLine[1024];
extern char opBuff[65536];
extern char *CFG_get_by_name(char *, char *);
extern unsigned AbortFlag;
extern unsigned ModeFlag;

static int wlan_create_interface(const int skfd, char *apname, 
                          const const char *clone_dev_name, 
                          const char *mode, const int staflag)
{
    struct ieee80211_clone_params cp;
    struct ifreq ifr;

    memset(&ifr, 0, sizeof(ifr));
    memset(&cp, 0, sizeof(cp));

    strncpy(cp.icp_name, apname, IFNAMSIZ);
    /* NB: station mode is the default */
    cp.icp_opmode = IEEE80211_M_STA;
    /* NB: default is to request a unique bssid/mac */
    cp.icp_flags = IEEE80211_CLONE_BSSID;

    strncpy(ifr.ifr_name, clone_dev_name, IFNAMSIZ);

    if (!staflag)
        cp.icp_flags &= ~(IEEE80211_NO_STABEACONS);
    else
        cp.icp_flags |= IEEE80211_NO_STABEACONS;

   if (!strncmp(mode,"sta",3))
        cp.icp_opmode = IEEE80211_M_STA;
   else
        cp.icp_opmode = IEEE80211_M_HOSTAP;

#if 0
        if (streq(s, "ibss") || streq(s, "adhoc"))
                return IEEE80211_M_IBSS; 
        if (streq(s, "mon"))
                return IEEE80211_M_MONITOR;
        if (streq(s, "ap") || streq(s, "hostap"))
                return IEEE80211_M_HOSTAP;
        if (streq(s, "wds"))
                return IEEE80211_M_WDS;
#endif

    ifr.ifr_data = (void *) &cp;

    if (ioctl(skfd, SIOC80211IFCREATE, &ifr) < 0) {
        fprintf(stderr,"%s: ioctl error %d\n", __func__, errno);
        perror("wlan_create_interface");
        return -1;
    }

    // printf("%s: after ioctl cp.icp_name = %s ifr.ifr_name = %s\n", __func__,cp.icp_name, ifr.ifr_name);

    if (memcmp(apname, ifr.ifr_name, IFNAMSIZ) != 0)
            strncpy(apname, ifr.ifr_name, IFNAMSIZ);


    return 0;

}

static int wlan_destroy_interface(int skfd, const char *ifname)
{
    struct ifreq ifr;


    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ);

    if (ioctl(skfd, SIOC80211IFDESTROY, &ifr) < 0) {
        fprintf(stderr,"%s: ioctl error %d\n", __func__, errno);
        return -1;
    }

    return 0;
}


#define ERR_SET_EXT(rname, request) \
	fprintf(stderr, "Error for wireless request \"%s\" (%X) :\n", \
		rname, request)

#define ABORT_ARG_NUM(rname, request) \
	do { \
		ERR_SET_EXT(rname, request); \
		fprintf(stderr, "    too few arguments.\n"); \
		return(-1); \
	} while(0)

#define ABORT_ARG_TYPE(rname, request, arg) \
	do { \
		ERR_SET_EXT(rname, request); \
		fprintf(stderr, "    invalid argument \"%s\".\n", arg); \
		return(-2); \
	} while(0)

#define ABORT_ARG_SIZE(rname, request, max) \
	do { \
		ERR_SET_EXT(rname, request); \
		fprintf(stderr, "    argument too big (max %d)\n", max); \
		return(-3); \
	} while(0)


/*------------------------------------------------------------------*/
/*
 * Wrapper to push some Wireless Parameter in the driver
 * Use standard wrapper and add pretty error message if fail...
 */
#define IW_SET_EXT_ERR(skfd, ifname, request, wrq, rname) \
	do { \
	if(iw_set_ext(skfd, ifname, request, wrq) < 0) { \
		ERR_SET_EXT(rname, request); \
		fprintf(stderr, "    SET failed on device %-1.16s ; %s.\n", \
			ifname, strerror(errno)); \
		return(-5); \
	} } while(0)

/*------------------------------------------------------------------*/
/*
 * Wrapper to extract some Wireless Parameter out of the driver
 * Use standard wrapper and add pretty error message if fail...
 */
#define IW_GET_EXT_ERR(skfd, ifname, request, wrq, rname) \
	do { \
	if(iw_get_ext(skfd, ifname, request, wrq) < 0) { \
		ERR_SET_EXT(rname, request); \
		fprintf(stderr, "    GET failed on device %-1.16s ; %s.\n", \
			ifname, strerror(errno)); \
		return(-6); \
	} } while(0)
/*
 * Convert IEEE channel number to MHz frequency.
 */
static u_int
ieee80211_ieee2mhz(u_int chan)
{
	if (chan == 14)
		return 2484;
	if (chan < 14)			/* 0-13 */
		return 2407 + chan*5;
	if (chan < 27)			/* 15-26 */
		return 2512 + ((chan-15)*20);
	return 5000 + (chan*5);
}

/*
 * Convert MHz frequency to IEEE channel number.
 */
static u_int
ieee80211_mhz2ieee(u_int freq)
{
	if (freq == 2484)
		return 14;
	if (freq < 2484)
		return (freq - 2407) / 5;
	if (freq < 5000)
		return 15 + ((freq - 2512) / 20);
	return (freq - 5000) / 5;
}

static const char *
getcaps(int capinfo)
{
	static char capstring[32];
	char *cp = capstring;

	if (capinfo & IEEE80211_CAPINFO_ESS)
		*cp++ = 'E';
	if (capinfo & IEEE80211_CAPINFO_IBSS)
		*cp++ = 'I';
	if (capinfo & IEEE80211_CAPINFO_CF_POLLABLE)
		*cp++ = 'c';
	if (capinfo & IEEE80211_CAPINFO_CF_POLLREQ)
		*cp++ = 'C';
	if (capinfo & IEEE80211_CAPINFO_PRIVACY)
		*cp++ = 'P';
	if (capinfo & IEEE80211_CAPINFO_SHORT_PREAMBLE)
		*cp++ = 'S';
	if (capinfo & IEEE80211_CAPINFO_PBCC)
		*cp++ = 'B';
	if (capinfo & IEEE80211_CAPINFO_CHNL_AGILITY)
		*cp++ = 'A';
	if (capinfo & IEEE80211_CAPINFO_SHORT_SLOTTIME)
		*cp++ = 's';
	if (capinfo & IEEE80211_CAPINFO_RSN)
		*cp++ = 'R';
	if (capinfo & IEEE80211_CAPINFO_DSSSOFDM)
		*cp++ = 'D';
	*cp = '\0';
	return capstring;
}

static const char *
getathcaps(int capinfo)
{
	static char capstring[32];
	char *cp = capstring;

	if (capinfo & IEEE80211_NODE_TURBOP)
		*cp++ = 'D';
	if (capinfo & IEEE80211_NODE_COMP)
		*cp++ = 'C';
	if (capinfo & IEEE80211_NODE_FF)
		*cp++ = 'F';
	if (capinfo & IEEE80211_NODE_XR)
		*cp++ = 'X';
	if (capinfo & IEEE80211_NODE_AR)
		*cp++ = 'A';
	if (capinfo & IEEE80211_NODE_BOOST)
		*cp++ = 'T';
	*cp = '\0';
	return capstring;
}

static const char *
gethtcaps(int capinfo)
{
	static char capstring[32];
	char *cp = capstring;

	if (capinfo & IEEE80211_HTCAP_C_ADVCODING)
		*cp++ = 'A';
	if (capinfo & IEEE80211_HTCAP_C_CHWIDTH40)
		*cp++ = 'W';
	if ((capinfo & IEEE80211_HTCAP_C_SM_MASK) ==
             IEEE80211_HTCAP_C_SM_ENABLED)
		*cp++ = 'P';
	if ((capinfo & IEEE80211_HTCAP_C_SM_MASK) ==
             IEEE80211_HTCAP_C_SMPOWERSAVE_STATIC)
		*cp++ = 'Q';
	if ((capinfo & IEEE80211_HTCAP_C_SM_MASK) ==
             IEEE80211_HTCAP_C_SMPOWERSAVE_DYNAMIC)
		*cp++ = 'R';
	if (capinfo & IEEE80211_HTCAP_C_GREENFIELD)
		*cp++ = 'G';
	if (capinfo & IEEE80211_HTCAP_C_SHORTGI40)
		*cp++ = 'S';
	if (capinfo & IEEE80211_HTCAP_C_DELAYEDBLKACK)
		*cp++ = 'D';
	if (capinfo & IEEE80211_HTCAP_C_MAXAMSDUSIZE)
		*cp++ = 'M';
	*cp = '\0';
	return capstring;
}

static void
printie(const char* tag, const uint8_t *ie, size_t ielen, int maxlen)
{
        int verbose=0;
	printf("%s", tag);
	if (verbose) {
		maxlen -= strlen(tag)+2;
		if (2*ielen > maxlen)
			maxlen--;
		printf("<");
		for (; ielen > 0; ie++, ielen--) {
			if (maxlen-- <= 0)
				break;
			printf("%02x", *ie);
		}
		if (ielen != 0)
			printf("-");
		printf(">");
	}
}

/* unalligned little endian access */
#define LE_READ_4(p)					\
	((u_int32_t)					\
	 ((((const u_int8_t *)(p))[0]      ) |		\
	  (((const u_int8_t *)(p))[1] <<  8) |		\
	  (((const u_int8_t *)(p))[2] << 16) |		\
	  (((const u_int8_t *)(p))[3] << 24)))

static int __inline
iswpaoui(const u_int8_t *frm)
{
	return frm[1] > 3 && LE_READ_4(frm+2) == ((WPA_OUI_TYPE<<24)|WPA_OUI);
}

static int __inline
iswmeoui(const u_int8_t *frm)
{
	return frm[1] > 3 && LE_READ_4(frm+2) == ((WME_OUI_TYPE<<24)|WME_OUI);
}

static int __inline
isatherosoui(const u_int8_t *frm)
{
	return frm[1] > 3 && LE_READ_4(frm+2) == ((ATH_OUI_TYPE<<24)|ATH_OUI);
}

static void
printies(const u_int8_t *vp, int ielen, int maxcols)
{
	while (ielen > 0) {
		switch (vp[0]) {
		case IEEE80211_ELEMID_VENDOR:
			if (iswpaoui(vp))
				printie(" WPA", vp, 2+vp[1], maxcols);
			else if (iswmeoui(vp))
				printie(" WME", vp, 2+vp[1], maxcols);
			else if (isatherosoui(vp))
				printie(" ATH", vp, 2+vp[1], maxcols);
			else
				printie(" VEN", vp, 2+vp[1], maxcols);
			break;
		case IEEE80211_ELEMID_RSN:
			printie(" RSN", vp, 2+vp[1], maxcols);
			break;
		default:
			printie(" ???", vp, 2+vp[1], maxcols);
			break;
		}
		ielen -= 2+vp[1];
		vp += 2+vp[1];
	}
}

static const char *
ieee80211_ntoa(const uint8_t mac[IEEE80211_ADDR_LEN])
{
	static char a[18];
	int i;

	i = snprintf(a, sizeof(a), "%02x:%02x:%02x:%02x:%02x:%02x",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return (i < 17 ? NULL : a);
}

static int
athcfg_wlan_list_stations(const char *ifname)
{
	uint8_t buf[24*1024];
	struct iwreq iwr;
	uint8_t *cp;
	int s, len;
        u_int32_t txrate, rxrate;

	s = athcfg_sock_init(__func__);

	if (s < 0) {
              return -1;
        }

	(void) memset(&iwr, 0, sizeof(iwr));
	(void) strncpy(iwr.ifr_name, ifname, sizeof(iwr.ifr_name));
	iwr.u.data.pointer = (void *) buf;
	iwr.u.data.length = sizeof(buf);

	if (ioctl(s, IEEE80211_IOCTL_STA_INFO, &iwr) < 0)  {
              sprintf(buf,"%s: ioctl error",__func__);
              perror(buf);
              athcfg_sock_deinit(s);
              return -1;
        }

	len = iwr.u.data.length;

	if (len < sizeof(struct ieee80211req_sta_info)) {
              fprintf(stderr,"%s: No stations connected\n",__func__);
              athcfg_sock_deinit(s);
              return -1;
        }

	printf("%-17.17s %4s %4s %6s %6s %4s %6s %6s %4s %5s %3s %8s %6s\n"
		, "ADDR"
		, "AID"
		, "CHAN"
		, "TxRATE"
		, "RxRATE"
		, "RSSI"
		, "IDLE"
		, "TXSEQ"
		, "RXSEQ"
		, "CAPS"
	        , "ACAPS"
		, "ERP"
		, "STATE"
	        , "HTCAPS"
	);
	cp = buf;
	do {
		struct ieee80211req_sta_info *si;
		uint8_t *vp;

		si = (struct ieee80211req_sta_info *) cp;
		vp = (u_int8_t *)(si+1);

                if(si->isi_txratekbps == 0)
                   txrate = (si->isi_rates[si->isi_txrate] & IEEE80211_RATE_VAL)/2;
                else
                   txrate = si->isi_txratekbps / 1000;

                rxrate = si->isi_rxratekbps / 1000;

		printf("%s %4u %4d %5dM %5dM %4d %6d %6d %5d %4.4s %-5.5s %7x %8x %-6.6s"
			, ieee80211_ntoa(si->isi_macaddr)
			, IEEE80211_AID(si->isi_associd)
			, ieee80211_mhz2ieee(si->isi_freq)
                        , txrate
                        , rxrate
			, si->isi_rssi
			, si->isi_inact
			, si->isi_txseqs[0]
			, si->isi_rxseqs[0]
		        , getcaps(si->isi_capinfo)
		        , getathcaps(si->isi_athflags)
			, si->isi_erp
			, si->isi_state
		        , gethtcaps(si->isi_htcap)
		);
		printies(vp, si->isi_ie_len, 24);
		printf("\n");
		cp += si->isi_len, len -= si->isi_len;
	} while (len >= sizeof(struct ieee80211req_sta_info));

        athcfg_sock_deinit(s);
        return 0;
}

/*------------------------------------------------------------------*/
/*
 * Set the wireless options requested on command line
 * This function is too long and probably should be split,
 * because it look like the perfect definition of spaghetti code,
 * but I'm way to lazy
 */
static int
iwconfig_set_info(
     int            skfd,
	 char           args[][150],
	 int		count,		/* Args count */
	 char *		ifname)		/* Dev name */
{
          struct iwreq		wrq;
          int			i;

          /* if nothing after the device name - will never happen */
          if(count < 1)
            {
              fprintf(stderr, "Error : too few arguments.\n");
              return(-1);
            }
#if 0
          for (i=0; i < count; i++)
              fprintf(stdout,"%s: args[%i] = %s\n", __func__, i, args[i]);
#endif
          /* The other args on the line specify options to be set... */
          for(i = 0; i < count; i++)
          {

              /* ---------- Set frequency / channel ---------- */
              if((!strncmp(args[i], "freq", 4)) ||
                 (!strcmp(args[i], "channel")))
                {
                  double		freq;

                  if(++i >= count)
                    ABORT_ARG_NUM("Set Frequency", SIOCSIWFREQ);
                  if(!strcasecmp(args[i], "auto"))
                    {
                      wrq.u.freq.m = -1;
                      wrq.u.freq.e = 0;
                      wrq.u.freq.flags = 0;
                    }
                  else
                    {
                      if(!strcasecmp(args[i], "fixed"))
                        {
                          /* Get old bitrate */
                          IW_GET_EXT_ERR(skfd, ifname, SIOCGIWFREQ, &wrq,
                                         "Set Bit Rate");
                          wrq.u.freq.flags = IW_FREQ_FIXED;
                        }
                      else			/* Should be a numeric value */
                        {
                          if(sscanf(args[i], "%lg", &(freq)) != 1)
                            ABORT_ARG_TYPE("Set Frequency", SIOCSIWFREQ, args[i]);
                          if(index(args[i], 'G')) freq *= GIGA;
                          if(index(args[i], 'M')) freq *= MEGA;
                          if(index(args[i], 'k')) freq *= KILO;

                          iw_float2freq(freq, &(wrq.u.freq));

                          wrq.u.freq.flags = IW_FREQ_FIXED;

                          /* Check for an additional argument */
                          if(((i+1) < count) &&
                             (!strcasecmp(args[i+1], "auto")))
                            {
                              wrq.u.freq.flags = 0;
                              ++i;
                            }
                          if(((i+1) < count) &&
                             (!strcasecmp(args[i+1], "fixed")))
                            {
                              wrq.u.freq.flags = IW_FREQ_FIXED;
                              ++i;
                            }
                        }
                    }

                  IW_SET_EXT_ERR(skfd, ifname, SIOCSIWFREQ, &wrq,
                                 "Set Frequency");
                  continue;
                }

              /* ---------- Set encryption stuff ---------- */
              if((!strncmp(args[i], "enc", 3)) ||
                 (!strcmp(args[i], "key")))
                {
                  unsigned char	key[IW_ENCODING_TOKEN_MAX];

                  if(++i >= count)
                    ABORT_ARG_NUM("Set Encode", SIOCSIWENCODE);
#if 0
                  if(!strcasecmp(args[i], "on"))
                    {
                      /* Get old encryption information */
                      wrq.u.data.pointer = (caddr_t) key;
                      wrq.u.data.length = IW_ENCODING_TOKEN_MAX;
                      wrq.u.data.flags = 0;
                      IW_GET_EXT_ERR(skfd, ifname, SIOCGIWENCODE, &wrq,
                                     "Set Encode");
                      wrq.u.data.flags &= ~IW_ENCODE_DISABLED;	/* Enable */
                    }
                  else
#endif
                  {
                      int	gotone = 0;
                      int	oldone;
                      int	keylen;
                      int	temp;

                      wrq.u.data.pointer = (caddr_t) NULL;
                      wrq.u.data.flags = 0;
                      wrq.u.data.length = 0;

                      /* Allow arguments in any order (it's safe) */
                      do {
                          oldone = gotone;

                          /* -- Check for the key -- */
                          if(i < count)
                          {
                             keylen = iw_in_key_full(skfd, ifname,
                                                     args[i], key, &wrq.u.data.flags);
                             if(keylen > 0)
                             {
                                  wrq.u.data.length = keylen;
                                  wrq.u.data.pointer = (caddr_t) key;
                                  ++i;
                                  gotone++;
                             }
                          }

                          /* -- Check for token index -- */
                          if((i < count) &&
                             (sscanf(args[i], "[%i]", &temp) == 1) &&
                             (temp > 0) && (temp < IW_ENCODE_INDEX))
                            {
                                 wrq.u.encoding.flags |= temp;
                                 ++i;
                                 gotone++;
                            }
#if 0
                          /* -- Check the various flags -- */
                          if((i < count) && (!strcasecmp(args[i], "off")))
                            {
                              wrq.u.data.flags |= IW_ENCODE_DISABLED;
                              ++i;
                              gotone++;
                            }
                          if((i < count) && (!strcasecmp(args[i], "open")))
                            {
                              wrq.u.data.flags |= IW_ENCODE_OPEN;
                              ++i;
                              gotone++;
                            }
                          if((i < count) && (!strncasecmp(args[i], "restricted", 5)))
                            {
                              wrq.u.data.flags |= IW_ENCODE_RESTRICTED;
                              ++i;
                              gotone++;
                            }
                          if((i < count) && (!strncasecmp(args[i], "temporary", 4)))
                            {
                              wrq.u.data.flags |= IW_ENCODE_TEMP;
                              ++i;
                              gotone++;
                            }
#endif
                      } while(gotone != oldone);

                      /* Pointer is absent in new API */
                      if(wrq.u.data.pointer == NULL)
                          wrq.u.data.flags |= IW_ENCODE_NOKEY;

                      /* Check if we have any invalid argument */
                      if(!gotone)
                          ABORT_ARG_TYPE("Set Encode", SIOCSIWENCODE, args[i]);
                      /* Get back to last processed argument */
                      --i;
                  }

                  IW_SET_EXT_ERR(skfd, ifname, SIOCSIWENCODE, &wrq,
                                 "Set Encode");
                  continue;
                }

              /* ---------- Set ESSID ---------- */
              if(!strcasecmp(args[i], "essid"))
              {
                  char		essid[IW_ESSID_MAX_SIZE + 1];
                  int		we_kernel_version;

                  i++;
                  if(i >= count)
                    ABORT_ARG_NUM("Set ESSID", SIOCSIWESSID);
#if 0
                  if((!strcasecmp(args[i], "off")) ||
                     (!strcasecmp(args[i], "any")))
                    {
                      wrq.u.essid.flags = 0;
                      essid[0] = '\0';
                    }
                  else
                    if(!strcasecmp(args[i], "on"))
                      {
                        /* Get old essid */
                        memset(essid, '\0', sizeof(essid));
                        wrq.u.essid.pointer = (caddr_t) essid;
                        wrq.u.essid.length = IW_ESSID_MAX_SIZE + 1;
                        wrq.u.essid.flags = 0;
                        IW_GET_EXT_ERR(skfd, ifname, SIOCGIWESSID, &wrq,
                                       "Set ESSID");
                        wrq.u.essid.flags = 1;
                      }
                    else
#endif
                      {
                        /* '-' or '--' allow to escape the ESSID string, allowing
                         * to set it to the string "any" or "off".
                         * This is a big ugly, but it will do for now */
                        if((!strcmp(args[i], "-")) || (!strcmp(args[i], "--")))
                          {
                            i++;
                            if(i >= count)
                              ABORT_ARG_NUM("Set ESSID", SIOCSIWESSID);
                          }

                        /* Check the size of what the user passed us to avoid
                         * buffer overflows */
                        if(strlen(args[i]) > IW_ESSID_MAX_SIZE)
                          ABORT_ARG_SIZE("Set ESSID", SIOCSIWESSID, IW_ESSID_MAX_SIZE);
                        else
                          {
                            int		temp;
                            int space_len = 0;     /*used to count number of preceding spaces in essid*/

                            wrq.u.essid.flags = 1;

                            while(*(args[i] + space_len) == ' ')
                              space_len ++;

                            strcpy(essid, (args[i] + space_len));	/* Size checked, all clear */

                            /* Check for ESSID index */
                            if(((i+1) < count) &&
                               (sscanf(args[i+1], "[%i]", &temp) == 1) &&
                               (temp > 0) && (temp < IW_ENCODE_INDEX))
                              {
                                wrq.u.essid.flags = temp;
                                ++i;
                              }
                          }
                      }

                  /* Get version from kernel, device may not have range... */
                  we_kernel_version = iw_get_kernel_we_version();

                  /* Finally set the ESSID value */
                  wrq.u.essid.pointer = (caddr_t) essid;
                  wrq.u.essid.length = strlen(essid) + 1;
                  if(we_kernel_version > 20)
                    wrq.u.essid.length--;
                  IW_SET_EXT_ERR(skfd, ifname, SIOCSIWESSID, &wrq,
                                 "Set ESSID");
                  continue;
                }

#if 0
              /* ---------- Set AP address ---------- */
              if(!strcasecmp(args[i], "ap"))
              {
                  if(++i >= count)
                    ABORT_ARG_NUM("Set AP Address", SIOCSIWAP);
                  if((!strcasecmp(args[i], "auto")) ||
                     (!strcasecmp(args[i], "any")))
                  {
                      /* Send a broadcast address */
                      iw_broad_ether(&(wrq.u.ap_addr));
                   }
                  else
                  {
                      if(!strcasecmp(args[i], "off"))
                        {
                          /* Send a NULL address */
                          iw_null_ether(&(wrq.u.ap_addr));
                        }
                      else
                        {
                          /* Get the address and check if the interface supports it */
                          if(iw_in_addr(skfd, ifname, args[i++], &(wrq.u.ap_addr)) < 0)
                            ABORT_ARG_TYPE("Set AP Address", SIOCSIWAP, args[i-1]);
                        }
                  }
                  /* Get the address and check if the interface supports it */
                  if(iw_in_addr(skfd, ifname, args[i++], &(wrq.u.ap_addr)) < 0)
                      ABORT_ARG_TYPE("Set AP Address", SIOCSIWAP, args[i-1]);

                  IW_SET_EXT_ERR(skfd, ifname, SIOCSIWAP, &wrq,
                                 "Set AP Address");
                  continue;
              }
#endif
              /* ---------- Set Transmit-Power ---------- */
              if(!strncmp(args[i], "txpower", 3))
              {
                  struct iw_range	range;

                  if(++i >= count)
                    ABORT_ARG_NUM("Set Tx Power", SIOCSIWTXPOW);

                  /* Extract range info */
                  if(iw_get_range_info(skfd, ifname, &range) < 0)
                    memset(&range, 0, sizeof(range));

                  /* Prepare the request */
                  wrq.u.txpower.value = -1;
                  wrq.u.txpower.fixed = 1;
                  wrq.u.txpower.disabled = 0;
                  wrq.u.txpower.flags = IW_TXPOW_DBM;
#if 0
                  if(!strcasecmp(args[i], "off"))
                    wrq.u.txpower.disabled = 1;	/* i.e. turn radio off */
                  else
                    if(!strcasecmp(args[i], "auto"))
                      wrq.u.txpower.fixed = 0;	/* i.e. use power control */
                    else
                      {
                        if(!strcasecmp(args[i], "on"))
                          {
                            /* Get old tx-power */
                            IW_GET_EXT_ERR(skfd, ifname, SIOCGIWTXPOW, &wrq,
                                           "Set Tx Power");
                            wrq.u.txpower.disabled = 0;
                          }
                        else
                          {
                            if(!strcasecmp(args[i], "fixed"))
                              {
                                /* Get old tx-power */
                                IW_GET_EXT_ERR(skfd, ifname, SIOCGIWTXPOW, &wrq,
                                               "Set Tx Power");
                                wrq.u.txpower.fixed = 1;
                                wrq.u.txpower.disabled = 0;
                              }
                            else			/* Should be a numeric value */
                              {
                                int		power;
                                int		ismwatt = 0;

                                /* Get the value */
                                if(sscanf(args[i], "%i", &(power)) != 1)
                                  ABORT_ARG_TYPE("Set Tx Power", SIOCSIWTXPOW,
                                                 args[i]);

                                /* Check if milliWatt
                                 * We authorise a single 'm' as a shorthand for 'mW',
                                 * on the other hand a 'd' probably means 'dBm'... */
                                ismwatt = ((index(args[i], 'm') != NULL)
                                           && (index(args[i], 'd') == NULL));

                                /* We could check 'W' alone... Another time... */

                                /* Convert */
                                if(range.txpower_capa & IW_TXPOW_RELATIVE)
                                  {
                                    /* Can't convert */
                                    if(ismwatt)
                                      ABORT_ARG_TYPE("Set Tx Power",
                                                     SIOCSIWTXPOW,
                                                     args[i]);
                                  }
                                else
                                  if(range.txpower_capa & IW_TXPOW_MWATT)
                                    {
                                      if(!ismwatt)
                                        power = iw_dbm2mwatt(power);
                                      wrq.u.txpower.flags = IW_TXPOW_MWATT;
                                    }
                                  else
                                    {
                                      if(ismwatt)
                                        power = iw_mwatt2dbm(power);
                                      wrq.u.txpower.flags = IW_TXPOW_DBM;
                                    }
                                wrq.u.txpower.value = power;

                                /* Check for an additional argument */
                                if(((i+1) < count) &&
                                   (!strcasecmp(args[i+1], "auto")))
                                  {
                                    wrq.u.txpower.fixed = 0;
                                    ++i;
                                  }
                                if(((i+1) < count) &&
                                   (!strcasecmp(args[i+1], "fixed")))
                                  {
                                    wrq.u.txpower.fixed = 1;
                                    ++i;
                                  }
                              }
                          }
                      }
#endif
                  {
                        int		power;
                        int		ismwatt = 0;

                        /* Get the value */
                        if(sscanf(args[i], "%i", &(power)) != 1)
                          ABORT_ARG_TYPE("Set Tx Power", SIOCSIWTXPOW,
                                         args[i]);

                        /* Check if milliWatt
                         * We authorise a single 'm' as a shorthand for 'mW',
                         * on the other hand a 'd' probably means 'dBm'... */
                        ismwatt = ((index(args[i], 'm') != NULL)
                                   && (index(args[i], 'd') == NULL));

                        /* We could check 'W' alone... Another time... */

                        /* Convert */
                        if(range.txpower_capa & IW_TXPOW_RELATIVE)
                        {
                            /* Can't convert */
                            if(ismwatt)
                              ABORT_ARG_TYPE("Set Tx Power",
                                             SIOCSIWTXPOW,
                                             args[i]);
                        }
                        else if(range.txpower_capa & IW_TXPOW_MWATT) {
                              if(!ismwatt)
                                power = iw_dbm2mwatt(power);
                              wrq.u.txpower.flags = IW_TXPOW_MWATT;
                        }
                        else {
                              if(ismwatt)
                                power = iw_mwatt2dbm(power);
                              wrq.u.txpower.flags = IW_TXPOW_DBM;
                        }
                        wrq.u.txpower.value = power;
                  }

                  IW_SET_EXT_ERR(skfd, ifname, SIOCSIWTXPOW, &wrq,
                                 "Set Tx Power");
                  continue;
              }

              /* ---------- Set RTS threshold ---------- */
              if(!strncasecmp(args[i], "rts", 3))
              {
                  i++;
                  if(i >= count)
                    ABORT_ARG_NUM("Set RTS Threshold", SIOCSIWRTS);
                  wrq.u.rts.value = -1;
                  wrq.u.rts.fixed = 1;
                  wrq.u.rts.disabled = 0;
                  if(!strcasecmp(args[i], "off"))
                    wrq.u.rts.disabled = 1;	/* i.e. max size */
                  else
                    if(!strcasecmp(args[i], "auto"))
                      wrq.u.rts.fixed = 0;
                    else
                      {
                        if(!strcasecmp(args[i], "fixed"))
                          {
                            /* Get old RTS threshold */
                            IW_GET_EXT_ERR(skfd, ifname, SIOCGIWRTS, &wrq,
                                           "Set RTS Threshold");
                            wrq.u.rts.fixed = 1;
                          }
                        else			/* Should be a numeric value */
                          if(sscanf(args[i], "%li", (unsigned long *) &(wrq.u.rts.value))
                             != 1)
                            ABORT_ARG_TYPE("Set RTS Threshold", SIOCSIWRTS, args[i]);
                          if(wrq.u.rts.value < 256)
                            ABORT_ARG_TYPE("Minimum RTS Threshold should be 256 bytes", SIOCSIWRTS, args[i]);   
                    }

                  IW_SET_EXT_ERR(skfd, ifname, SIOCSIWRTS, &wrq,
                                 "Set RTS Threshold");
                  continue;
              }
              /* ---------- Set fragmentation threshold ---------- */
              if(!strncmp(args[i], "frag", 4))
                {
                  i++;
                  if(i >= count)
                    ABORT_ARG_NUM("Set Fragmentation Threshold", SIOCSIWFRAG);
                  wrq.u.frag.value = -1;
                  wrq.u.frag.fixed = 1;
                  wrq.u.frag.disabled = 0;
                  if(!strcasecmp(args[i], "off"))
                    wrq.u.frag.disabled = 1;	/* i.e. max size */
                  else
                    if(!strcasecmp(args[i], "auto"))
                      wrq.u.frag.fixed = 0;
                    else
                      {
                        if(!strcasecmp(args[i], "fixed"))
                          {
                            /* Get old fragmentation threshold */
                            IW_GET_EXT_ERR(skfd, ifname, SIOCGIWFRAG, &wrq,
                                           "Set Fragmentation Threshold");
                            wrq.u.frag.fixed = 1;
                          }
                        else			/* Should be a numeric value */
                          if(sscanf(args[i], "%li",
                                    (unsigned long *) &(wrq.u.frag.value))
                             != 1)
                            ABORT_ARG_TYPE("Set Fragmentation Threshold", SIOCSIWFRAG,
                                           args[i]);
                    }

                  IW_SET_EXT_ERR(skfd, ifname, SIOCSIWFRAG, &wrq,
                                 "Set Fragmentation Threshold");
                  continue;
                }
#if 0
              /* ---------- Set sensitivity ---------- */
              if(!strncmp(args[i], "sens", 4))
                {
                  if(++i >= count)
                    ABORT_ARG_NUM("Set Sensitivity", SIOCSIWSENS);
                  if(sscanf(args[i], "%i", &(wrq.u.sens.value)) != 1)
                    ABORT_ARG_TYPE("Set Sensitivity", SIOCSIWSENS, args[i]);

                  IW_SET_EXT_ERR(skfd, ifname, SIOCSIWSENS, &wrq,
                                 "Set Sensitivity");
                  continue;
                }

              /* ---------- Set operation mode ---------- */
              if(!strcmp(args[i], "mode"))
                {
                  int	k;

                  i++;
                  if(i >= count)
                    ABORT_ARG_NUM("Set Mode", SIOCSIWMODE);

                  if(sscanf(args[i], "%i", &k) != 1)
                    {
                      k = 0;
                      while((k < IW_NUM_OPER_MODE) &&
                            strncasecmp(args[i], iw_operation_mode[k], 3))
                        k++;
                    }
                  if((k >= IW_NUM_OPER_MODE) || (k < 0))
                    ABORT_ARG_TYPE("Set Mode", SIOCSIWMODE, args[i]);

                  wrq.u.mode = k;
                  IW_SET_EXT_ERR(skfd, ifname, SIOCSIWMODE, &wrq,
                                 "Set Mode");
                  continue;
                }

#endif
            }
        return 0;
}

static int athcfg_ifconfig_ops(const char *ifname, char *cmd, char *args)
{
    struct ifreq ifr;
    struct sockaddr_in sai;
    struct sockaddr sa;
    int sockfd = -1, ret = -1;
    int op;
    char buf[50];

    sai.sin_family = AF_INET;
    sai.sin_port = 0;

    sockfd = athcfg_sock_init(__func__);

    if (sockfd == -1)
        return -1;

    strncpy(ifr.ifr_name, ifname, IFNAMSIZ);

    if (!strcmp(cmd,"ipaddr")) {

        if (inet_aton(args, &sai.sin_addr) == 0)
        {
            sprintf(buf,"%s: inet_aton error", __func__);
            perror(buf);
            goto athcfg_ifconfig_ops_cleanup;
        }

        memcpy(&ifr.ifr_addr, &sai, sizeof(sockaddr));
        op = SIOCSIFADDR;

    }
    else if (!strcmp(cmd,"netmask")) {

        if (inet_aton(args, &sai.sin_addr) == 0)
        {
            sprintf(buf,"%s: inet_aton error", __func__);
            perror(buf);
            goto athcfg_ifconfig_ops_cleanup;
        }

        memcpy(&ifr.ifr_netmask, &sai, sizeof(sockaddr));
        op = SIOCSIFNETMASK;
    }
    else if (!strcmp(cmd,"ifstate")) {

        if (ioctl(sockfd, SIOCGIFFLAGS, &ifr) < 0) {
                sprintf(buf,"%s: SIOCGIFFLAGS error", __func__);
                perror(buf);
                goto athcfg_ifconfig_ops_cleanup;
        } else {

                if (!strncmp(args,"up",2)) {
                        ifr.ifr_flags |= (IFF_UP | IFF_RUNNING);
                } else if (!strncmp(args,"down",4)){
                        ifr.ifr_flags &= ~(IFF_UP);
                } else {
                    printf("%s: invalid status %s \n",__func__, args);
                    goto athcfg_ifconfig_ops_cleanup;
                }
                op = SIOCSIFFLAGS;
        }
    }
    else if (!strcmp(cmd,"txqueuelen"))
    {
        ifr.ifr_qlen = atoi(args);
        op = SIOCSIFTXQLEN;
    }

    ret = ioctl(sockfd, op, &ifr);

    if (ret < 0) {
        sprintf(buf, "%s: ioctl err op = %d\n", __func__, op);
        perror(buf);
    }


athcfg_ifconfig_ops_cleanup:

   athcfg_sock_deinit(sockfd);

   return ret;

}

int athcfg_sock_init(const char *caller)
{
    int sockfd;
    char buf[50];

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        sprintf(buf,"%s: socket err caller: %s", __func__, caller);
        perror(buf);
    }

    return sockfd;
}

int athcfg_sock_deinit(int skfd)
{
    close(skfd);
}

static unsigned long __tv_to_jiffies(struct timeval *tv)
{
    unsigned long long jif;

    jif = 1000000ULL * tv->tv_sec + tv->tv_usec;

    return (HZ * jif) / 1000000;
}

static void __jiffies_to_tv(struct timeval *tv, unsigned long jiffies)
{
    unsigned long long tvusec;

    tvusec = (1000000ULL * jiffies) / HZ;
    tv->tv_sec = tvusec / 1000000;
    tv->tv_usec = tvusec - 1000000 * tv->tv_sec;
}

static void __bridge_info_copy(struct bridge_info *info,
                                                           struct __bridge_info *i)
{
    memcpy(&info->designated_root, &i->designated_root, 8);
    memcpy(&info->bridge_id, &i->bridge_id, 8);
    info->root_path_cost = i->root_path_cost;
    info->topology_change = i->topology_change;
    info->topology_change_detected = i->topology_change_detected;
    info->root_port = i->root_port;
    info->stp_enabled = i->stp_enabled;
    __jiffies_to_tv(&info->max_age, i->max_age);
    __jiffies_to_tv(&info->hello_time, i->hello_time);
    __jiffies_to_tv(&info->forward_delay, i->forward_delay);
    __jiffies_to_tv(&info->bridge_max_age, i->bridge_max_age);
    __jiffies_to_tv(&info->bridge_hello_time, i->bridge_hello_time);
    __jiffies_to_tv(&info->bridge_forward_delay, i->bridge_forward_delay);
    __jiffies_to_tv(&info->ageing_time, i->ageing_time);
    __jiffies_to_tv(&info->gc_interval, i->gc_interval);
    __jiffies_to_tv(&info->hello_timer_value, i->hello_timer_value);
    __jiffies_to_tv(&info->tcn_timer_value, i->tcn_timer_value);
    __jiffies_to_tv(&info->topology_change_timer_value,
                                    i->topology_change_timer_value);
    __jiffies_to_tv(&info->gc_timer_value, i->gc_timer_value);
}

static void __port_info_copy(struct port_info *info, struct __port_info *i)
{
    memcpy(&info->designated_root, &i->designated_root, 8);
    memcpy(&info->designated_bridge, &i->designated_bridge, 8);
    info->port_id = i->port_id;
    info->designated_port = i->designated_port;
    info->path_cost = i->path_cost;
    info->designated_cost = i->designated_cost;
    info->state = i->state;
    info->top_change_ack = i->top_change_ack;
    info->config_pending = i->config_pending;
    __jiffies_to_tv(&info->message_age_timer_value,
                                    i->message_age_timer_value);
    __jiffies_to_tv(&info->forward_delay_timer_value,
                                    i->forward_delay_timer_value);
    __jiffies_to_tv(&info->hold_timer_value, i->hold_timer_value);
}

static int br_device_ioctl(struct bridge *br, unsigned long arg0,
                                          unsigned long arg1, unsigned long arg2,
                                          unsigned long arg3)
{
    unsigned long args[4];
    struct ifreq ifr;

    args[0] = arg0;
    args[1] = arg1;
    args[2] = arg2;
    args[3] = arg3;

    memcpy(ifr.ifr_name, br->ifname, IFNAMSIZ);
    ((unsigned long *) (&ifr.ifr_data))[0] = (unsigned long) args;

    return ioctl(br_socket_fd, SIOCDEVPRIVATE, &ifr);
}

static int br_read_info(struct bridge *br)
{
    struct __bridge_info i;

    if (if_indextoname(br->ifindex, br->ifname) == NULL)
            return 1;

    if (br_device_ioctl(br, BRCTL_GET_BRIDGE_INFO,
                                            (unsigned long) &i, 0, 0) < 0)
            return 1;

    __bridge_info_copy(&br->info, &i);
    return 0;
}

static int br_read_port_info(struct port *p)
{
    struct __port_info i;

    if (br_device_ioctl(p->parent, BRCTL_GET_PORT_INFO,
                                            (unsigned long) &i, p->index, 0) < 0)
            return errno;

    __port_info_copy(&p->info, &i);
    return 0;
}

static void br_nuke_bridge(struct bridge *b)
{
    struct port *p;

    p = b->firstport; 
    while (p != NULL) { 
            struct port *pnext;

            pnext = p->next;
            free(p);
            p = pnext;
    }

    free(b);
}

static int br_make_port_list(struct bridge *br)
{
    int err;
    int i;
    int ifindices[256];

    if (br_device_ioctl(br, BRCTL_GET_PORT_LIST, (unsigned long) ifindices,
                                            0, 0) < 0)
            return errno;

    for (i = 255; i >= 0; i--) {
            struct port *p;

            if (!ifindices[i])
                    continue;

            p = malloc(sizeof(struct port));
            p->index = i;
            p->ifindex = ifindices[i];
            p->parent = br;
            br->ports[i] = p;
            p->next = br->firstport;
            br->firstport = p;
            if ((err = br_read_port_info(p)) != 0)
                    goto error_out;
    }

    return 0;

error_out:
    while (++i < 256)
            free(br->ports[i]);

    return err;
}

static int br_ioctl(unsigned long arg0, unsigned long arg1,
                                          unsigned long arg2)
{
    unsigned long arg[3];

    arg[0] = arg0;
    arg[1] = arg1;
    arg[2] = arg2;

    return ioctl(br_socket_fd, SIOCGIFBR, arg);
}

struct bridge *br_find_bridge(char *brname)
{
    struct bridge *b;

    b = bridge_list;
    while (b != NULL) {
            if (!strcmp(b->ifname, brname))
                    return b;

            b = b->next;
    }

    return NULL; 
}

static int br_make_bridge_list()
{
    int err;
    int i;
    int ifindices[32];
    int num;

    num = br_ioctl(BRCTL_GET_BRIDGES, (unsigned long) ifindices, 32);
    if (num < 0)
            return errno;

    bridge_list = NULL;
    for (i = 0; i < num; i++) {
            struct bridge *br;

            br = malloc(sizeof(struct bridge));
            memset(br, 0, sizeof(struct bridge));
            br->ifindex = ifindices[i];
            br->firstport = NULL;
            br->next = bridge_list;
            bridge_list = br;
            if ((err = br_read_info(br)) != 0)
                    goto error_out;
            if ((err = br_make_port_list(br)) != 0)
                    goto error_out;
    }

    return 0;

error_out:
    while (bridge_list != NULL) {
            struct bridge *nxt;

            nxt = bridge_list->next;
            br_nuke_bridge(bridge_list);
            bridge_list = nxt;
    }

    return err;
}

static int br_get_version()
{
    return br_ioctl(BRCTL_GET_VERSION, 0, 0);
}

static int br_add_interface(struct bridge *br, int ifindex)
{
    if (br_device_ioctl(br, BRCTL_ADD_IF, ifindex, 0, 0) < 0)
            return errno;

    return 0;
}

static int br_del_interface(struct bridge *br, int ifindex)
{
    if (br_device_ioctl(br, BRCTL_DEL_IF, ifindex, 0, 0) < 0)
            return errno;

    return 0;
}

int br_set_bridge_forward_delay(struct bridge *br)
{
    unsigned long jif = 0;

    if (br_device_ioctl(br, BRCTL_SET_BRIDGE_FORWARD_DELAY, jif, 0, 0) < 0)
        return -1;

    return 0;
}

static int br_add_bridge(char *brname)
{
    char _br[IFNAMSIZ];

    memcpy(_br, brname, IFNAMSIZ);
    if (br_ioctl(BRCTL_ADD_BRIDGE, (unsigned long) _br, 0) < 0)
            return errno;

    return 0;
}

static int br_del_bridge(char *brname)
{
    char _br[IFNAMSIZ];

    memcpy(_br, brname, IFNAMSIZ);
    if (br_ioctl(BRCTL_DEL_BRIDGE, (unsigned long) _br, 0) < 0)
            return errno;

    return 0;
}

static int bridge_add_del_interface(char *, char *, int);

static int br_cmd_addbr(char *brname)
{
    int err;

    if ((err = br_add_bridge(brname)) == 0) {
        /* set the forwarding delay to zero */
        err = bridge_add_del_interface(brname, NULL, 3);
        return 0;
    }

    switch (err) {
    case EEXIST:
            fprintf(stderr, "device %s already exists; can't create "
                            "bridge with the same name\n", brname);
            break;

    default:
            perror("br_add_bridge");
            break;
    }

    return -1;
}

static int br_cmd_delbr(char *brname)
{
    int err;

    if ((err = br_del_bridge(brname)) == 0)
            return 0;

    switch (err) {
    case ENXIO:
            fprintf(stderr, "bridge %s doesn't exist; can't delete it\n", brname);
            return -1;

    case EBUSY:
            fprintf(stderr, "bridge %s is still up; can't delete it\n", brname);
            return 0;

    default:
            perror("br_del_bridge");
            return -1;
    }
}

static int  br_cmd_addif(struct bridge *br, char *ifname)
{
    int err;
    int ifindex;

    ifindex = if_nametoindex(ifname);
    if (!ifindex) {
            fprintf(stderr, "interface %s does not exist!\n", ifname);
            return -1;
    }

    if ((err = br_add_interface(br, ifindex)) == 0) 
            return 0;

    switch (err) {
    case EBUSY:
            fprintf(stderr, "device %s is already a member of a bridge; "
                            "can't enslave it to bridge %s.\n", ifname, br->ifname);
            break;

    case ELOOP:
            fprintf(stderr, "device %s is a bridge device itself; "
                            "can't enslave a bridge device to a bridge device.\n",
                            ifname);
            break;

    default:
            perror("br_add_interface");
            break;
    }

    return -1;
}

static int br_cmd_delif(struct bridge *br, char *ifname)
{
    int err;
    int ifindex;

    ifindex = if_nametoindex(ifname);
    if (!ifindex) {
            fprintf(stderr, "interface %s does not exist!\n", ifname);
            return;
    }

    if ((err = br_del_interface(br, ifindex)) == 0)
            return 0;

    switch (err) {
    case EINVAL:
            fprintf(stderr, "device %s is not a slave of %s\n",
                            ifname, br->ifname);
            break;

    default:
            perror("br_del_interface");
            break;
    }

    return -1;

}

static int ath_br_init()
{
    if (br_socket_fd == -1) {
        if ((br_socket_fd = athcfg_sock_init(__func__)) < 0) {
            return -1;
        }
    }
    return 0;
}

static int ath_br_deinit()
{
    if (br_socket_fd != -1) {
        athcfg_sock_deinit(br_socket_fd);
        br_socket_fd = -1;
    }
    return 0;

}

static int ath_get_brlist(void)
{
    int err;

    if (br_get_version() != BRCTL_VERSION)
            return -1;

    if ((err = br_make_bridge_list()) != 0)
            return err;

    return 0;
}

/*
 * Execute a private command on the interface
 */
static int
iwpriv_set_private_cmd(
                int		skfd,		/* Socket */
		char 		args[][30],		/* Command line args */
		int		count,		/* Args count */
		char *		ifname,		/* Dev name */
		char *		cmdname,	/* Command name */
		iwprivargs *	priv,		/* Private ioctl description */
		int		priv_num)	/* Number of descriptions */
{
  struct iwreq	wrq;
  u_char	buffer[4096];	/* Only that big in v25 and later */
  int		i = 0;		/* Start with first command arg */
  int		k;		/* Index in private description table */
  int		temp;
  int		subcmd = 0;	/* sub-ioctl index */
  int		offset = 0;	/* Space for sub-ioctl index */

  /* Check if we have a token index.
   * Do it now so that sub-ioctl takes precedence, and so that we
   * don't have to bother with it later on... */
  if((count >= 1) && (sscanf(args[0], "[%i]", &temp) == 1))
    {
      subcmd = temp;
      args++;
      count--;
    }

  /* Search the correct ioctl */
  k = -1;
  while((++k < priv_num) && strcmp(priv[k].name, cmdname));

  /* If not found... */
  if(k == priv_num)
    {
      fprintf(stderr, "Invalid command : %s\n", cmdname);
      return(-1);
    }

  /* Watch out for sub-ioctls ! */
  if(priv[k].cmd < SIOCDEVPRIVATE)
    {
      int	j = -1;

      /* Find the matching *real* ioctl */
      while((++j < priv_num) && ((priv[j].name[0] != '\0') ||
				 (priv[j].set_args != priv[k].set_args) ||
				 (priv[j].get_args != priv[k].get_args)));

      /* If not found... */
      if(j == priv_num)
	{
	  fprintf(stderr, "Invalid private ioctl definition for : %s\n",
		  cmdname);
	  return(-1);
	}

      /* Save sub-ioctl number */
      subcmd = priv[k].cmd;
      /* Reserve one int (simplify alignment issues) */
      offset = sizeof(__u32);
      /* Use real ioctl definition from now on */
      k = j;

#if 0
      printf("<mapping sub-ioctl %s to cmd 0x%X-%d>\n", cmdname,
	     priv[k].cmd, subcmd);
#endif
    }

  /* If we have to set some data */
  if((priv[k].set_args & IW_PRIV_TYPE_MASK) &&
     (priv[k].set_args & IW_PRIV_SIZE_MASK))
    {
      switch(priv[k].set_args & IW_PRIV_TYPE_MASK)
	{
	case IW_PRIV_TYPE_BYTE:
	  /* Number of args to fetch */
	  wrq.u.data.length = count;
	  if(wrq.u.data.length > (priv[k].set_args & IW_PRIV_SIZE_MASK))
	    wrq.u.data.length = priv[k].set_args & IW_PRIV_SIZE_MASK;

	  /* Fetch args */
	  for(; i < wrq.u.data.length; i++) {
	    sscanf(args[i], "%i", &temp);
	    buffer[i] = (char) temp;
	  }
	  break;

	case IW_PRIV_TYPE_INT:
	  /* Number of args to fetch */
	  wrq.u.data.length = count;
	  if(wrq.u.data.length > (priv[k].set_args & IW_PRIV_SIZE_MASK))
	    wrq.u.data.length = priv[k].set_args & IW_PRIV_SIZE_MASK;

	  /* Fetch args */
	  for(; i < wrq.u.data.length; i++) {
	    sscanf(args[i], "%i", &temp);
	    ((__s32 *) buffer)[i] = (__s32) temp;
	  }
	  break;

	case IW_PRIV_TYPE_CHAR:
	  if(i < count)
	    {
	      /* Size of the string to fetch */
	      wrq.u.data.length = strlen(args[i]) + 1;
	      if(wrq.u.data.length > (priv[k].set_args & IW_PRIV_SIZE_MASK))
		wrq.u.data.length = priv[k].set_args & IW_PRIV_SIZE_MASK;

	      /* Fetch string */
	      memcpy(buffer, args[i], wrq.u.data.length);
	      buffer[sizeof(buffer) - 1] = '\0';
	      i++;
	    }
	  else
	    {
	      wrq.u.data.length = 1;
	      buffer[0] = '\0';
	    }
	  break;

	case IW_PRIV_TYPE_FLOAT:
	  /* Number of args to fetch */
	  wrq.u.data.length = count;
	  if(wrq.u.data.length > (priv[k].set_args & IW_PRIV_SIZE_MASK))
	    wrq.u.data.length = priv[k].set_args & IW_PRIV_SIZE_MASK;

	  /* Fetch args */
	  for(; i < wrq.u.data.length; i++) {
	    double		freq;
	    if(sscanf(args[i], "%lg", &(freq)) != 1)
	      {
		printf("Invalid float [%s]...\n", args[i]);
		return(-1);
	      }
	    if(index(args[i], 'G')) freq *= GIGA;
	    if(index(args[i], 'M')) freq *= MEGA;
	    if(index(args[i], 'k')) freq *= KILO;
	    sscanf(args[i], "%i", &temp);
	    iw_float2freq(freq, ((struct iw_freq *) buffer) + i);
	  }
	  break;

	case IW_PRIV_TYPE_ADDR:
	  /* Number of args to fetch */
	  wrq.u.data.length = count;
	  if(wrq.u.data.length > (priv[k].set_args & IW_PRIV_SIZE_MASK))
	    wrq.u.data.length = priv[k].set_args & IW_PRIV_SIZE_MASK;

	  /* Fetch args */
	  for(; i < wrq.u.data.length; i++) {
	    if(iw_in_addr(skfd, ifname, args[i],
			  ((struct sockaddr *) buffer) + i) < 0)
	      {
		printf("Invalid address [%s]...\n", args[i]);
		return(-1);
	      }
	  }
	  break;

	default:
	  fprintf(stderr, "Not implemented...\n");
	  return(-1);
	}

      if((priv[k].set_args & IW_PRIV_SIZE_FIXED) &&
	 (wrq.u.data.length != (priv[k].set_args & IW_PRIV_SIZE_MASK)))
	{
	  printf("The command %s needs exactly %d argument(s)...\n",
		 cmdname, priv[k].set_args & IW_PRIV_SIZE_MASK);
	  return(-1);
	}
    }	/* if args to set */
  else
    {
      wrq.u.data.length = 0L;
    }

  strncpy(wrq.ifr_name, ifname, IFNAMSIZ);

  /* Those two tests are important. They define how the driver
   * will have to handle the data */
  if((priv[k].set_args & IW_PRIV_SIZE_FIXED) &&
      ((iw_get_priv_size(priv[k].set_args) + offset) <= IFNAMSIZ))
    {
      /* First case : all SET args fit within wrq */
      if(offset)
	wrq.u.mode = subcmd;
      memcpy(wrq.u.name + offset, buffer, IFNAMSIZ - offset);
    }
  else
    {
      if((priv[k].set_args == 0) &&
	 (priv[k].get_args & IW_PRIV_SIZE_FIXED) &&
	 (iw_get_priv_size(priv[k].get_args) <= IFNAMSIZ))
	{
	  /* Second case : no SET args, GET args fit within wrq */
	  if(offset)
	    wrq.u.mode = subcmd;
	}
      else
	{
	  /* Third case : args won't fit in wrq, or variable number of args */
	  wrq.u.data.pointer = (caddr_t) buffer;
	  wrq.u.data.flags = subcmd;
	}
    }

  /* Perform the private ioctl */
  if(ioctl(skfd, priv[k].cmd, &wrq) < 0)
    {
      fprintf(stderr, "Interface doesn't accept private ioctl...\n");
      fprintf(stderr, "%s (%X): %s\n", cmdname, priv[k].cmd, strerror(errno));
      return(-1);
    }

  return(0);
}

/*------------------------------------------------------------------*/
/*
 * Execute a private command on the interface
 */

static iwpriv_init(int skfd,  const char *ifname)
{

    int iwpriv_cmds = 0;
    iwprivargs **iwpriv_list;

    if (!strcmp(ifname,"wifi0"))
      iwpriv_list = &wlan_hw_if_privcmd_list;
    else
      iwpriv_list = &wlan_vap_if_privcmd_list;

    /* Read the private ioctls */
    iwpriv_cmds = iw_get_priv_info(skfd, ifname, iwpriv_list);

    /* Is there any ? */
    if(iwpriv_cmds <= 0) {
      /* Should I skip this message ? */
      fprintf(stderr, "%-8.16s  no private ioctls.\n\n",
              ifname);
      free(*iwpriv_list);
      return(-1);
    }

    if (!strcmp(ifname,"wifi0"))
      iwpriv_hw_if_cmd_numbers = iwpriv_cmds;
    else
      iwpriv_vap_if_cmd_numbers = iwpriv_cmds;

    return 0;

}

static void iwpriv_deinit()
{
    if (wlan_hw_if_privcmd_list != NULL)
        free(wlan_hw_if_privcmd_list);

    if (wlan_vap_if_privcmd_list != NULL)
        free(wlan_vap_if_privcmd_list);
}

static inline int
iwpriv_set_private(
            int		skfd,		/* Socket */
	    char 	args[][30],		/* Command line args */
	    int		count,		/* Args count */
	    char *	ifname)		/* Dev name */
{
    int		ret, iwpriv_cmd_numbers = 0;
    iwprivargs    *iwpriv_cmd_list;

    if (!strcmp(ifname,"wifi0")) {
      iwpriv_cmd_list = wlan_hw_if_privcmd_list;
      iwpriv_cmd_numbers = iwpriv_hw_if_cmd_numbers;
    }
    else {
      iwpriv_cmd_list = wlan_vap_if_privcmd_list;
      iwpriv_cmd_numbers = iwpriv_vap_if_cmd_numbers;
    }

    /* Do it */
    ret = iwpriv_set_private_cmd(skfd,
                               args + 1,
                               count - 1,
                               ifname,
                               args[0],
                               iwpriv_cmd_list,
                               iwpriv_cmd_numbers);

    return(ret);
}

static int bridge_add_del_interface(char *brname, char *ifname, int addif)
{
    int ret = -1;
    struct bridge *br;

    if (ath_br_init() != 0)
        return -1;

    br = NULL;

    if (ath_get_brlist() != 0) {
         fprintf(stderr,"%s: cannot get the bridge list \
                 %s\n", __func__, brname);
         goto bradd_cleanup;
    }

    br =  br_find_bridge(brname);

    if (br == NULL) {
       fprintf(stderr,"%s: cannot find the bridge \
               %s\n", __func__, brname);
       goto bradd_cleanup;
    }

    if (addif == 1)
        ret = br_cmd_addif(br, ifname);
    else
    {
        if (addif == 2)
            ret = br_cmd_delif(br, ifname);
        else
            ret = br_set_bridge_forward_delay(br);
    }

    if (ret == -1) {
       fprintf(stderr,"%s: cannot %s %s the bridge\
               %s\n", __func__, ifname, \
               (addif == 1 ? "add to": \
               (addif == 2 ? "delete from" : "set fd on")),\
               brname);
    }

    ret=0;

bradd_cleanup:
    ath_br_deinit();
    return ret;
}

struct iwpriv_cmds_list
{
    char iwpriv_varname[30];
    char nvram_varname[20];
    char default_value[20];
    char apname[10];
    int  dont_set_if_empty;
};

static struct iwpriv_cmds_list iwpriv_list1[] = {
    { "bgscan", "NULL", "0", "custom", 0 },
    //{ "HALDbg", "HALDEBUG", "0x0", "wifi0", 0 },
    { "ATHDebug" , "ATHDEBUG", "0x0", "wifi0", 0 },
    { "dbgLVL", "DEBUGMODE", "0x100","custom", 0 },
    { "shortgi", "SHORTGI", "NULL" , "custom", 0 },
    { "noedgech", "NO_EDGE_CH", "NULL", "custom", 1},
    { "mode", "AP_CHMODE", "NULL", "custom", 0 },
    { "set_swapled", "SWAP_LED", "NULL", "wifi0", 1 },
    { "set_ledcustom", "LED_CUSTOM", "NULL", "wifi0", 1 },
    { "disablecoext", "NULL", "1", "custom", 0 }
};

static struct iwpriv_cmds_list iwpriv_list2[] = {
    { "AMPDU", "AMPDUENABLE", "NULL", "wifi0", 0 },
    { "AMPDUFrames", "AMPDUFRAMES", "NULL", "wifi0", 0 },
    { "AMPDULim", "AMPDULIMIT",  "NULL", "wifi0", 0 },
    { "pureg", "PUREG", "NULL", "custom", 0 },
    { "puren", "PUREN", "NULL", "custom", 0 }
};

#define ATH_IWPRIV_SET(skfd, iwpriv_args, iwpriv_arg_count, ifname)\
{\
    /* fprintf(stdout, "%s: setting iwpriv var : %s value : %s ap : %s\n", __func__, \
            iwpriv_args[0], iwpriv_args[1], ifname); */ \
    ret = iwpriv_set_private(skfd, iwpriv_args, iwpriv_arg_count, ifname);\
    if (ret != 0) {\
        fprintf(stderr,"%s: Error setting iwpriv var : %s value : %s ap : %s\n", \
                        __func__, iwpriv_args[0], iwpriv_args[1], ifname); \
        return -1;\
    }\
}

static int iwpriv_set_default_commands(int skfd, char *apname, struct iwpriv_cmds_list iwlist[], int count)
{
    char buf[50];
    char iwpriv_args[2][30];
    char *iwapname;
    int i=0, ret=0;

    for (i=0; i < count; i++) {


        if (strcmp(iwlist[i].default_value,"NULL") == 0) {

            CFG_get_by_name(iwlist[i].nvram_varname, buf);

            if (strlen(buf) == 0 && iwlist[i].dont_set_if_empty)
                continue;

            strcpy(iwpriv_args[1], buf);

        }
        else
            strcpy(iwpriv_args[1],iwlist[i].default_value);

        strcpy(iwpriv_args[0], iwlist[i].iwpriv_varname);

        iwapname = iwlist[i].apname;

        if (!strcmp(iwapname,"custom"))
            strcpy(iwapname, apname);

        ATH_IWPRIV_SET(skfd, iwpriv_args, 2, iwapname);

    }

    return 0;
}

static int iwpriv_set_custom_commands_set_1(int skfd, char *apname)
{
    char buf[50];
    char iwpriv_args[2][30];
    int ret=0;


    CFG_get_by_name("AP_CHMODE",buf);

    if (strlen(buf) && strstr(buf,"11NG")) {
        /* for ani processing */
        strcpy(iwpriv_args[0],"ForBiasAuto");
        strcpy(iwpriv_args[1],"1");
        ATH_IWPRIV_SET(skfd,iwpriv_args, 2, "wifi0");
    }

    strcpy(iwpriv_args[0],"extoffset");

    if (strlen(buf) && strstr(buf,"PLUS")) {
        strcpy(iwpriv_args[1],"1");
        ATH_IWPRIV_SET(skfd,iwpriv_args, 2, apname);
    } else if (strlen(buf) && strstr(buf,"MINUS")) {
        strcpy(iwpriv_args[1],"-1");
        ATH_IWPRIV_SET(skfd,iwpriv_args, 2, apname);
    }

    strcpy(iwpriv_args[0],"cwmmode");

    if ((!strncmp(buf,"11NGHT20", 8)) ||
        (!strncmp(buf,"11NAHT20", 8))) {
        strcpy(iwpriv_args[1],"0");
    } else {
        CFG_get_by_name("CWMMODE",buf);

        if (strlen(buf))
               strcpy(iwpriv_args[1],buf);
        else
               strcpy(iwpriv_args[1],"1");
    }

    ATH_IWPRIV_SET(skfd, iwpriv_args, 2, apname);

    return 0;
}

static int iwpriv_set_custom_commands_set_2(int skfd, char *apname)
{
    char buf[50];
    char iwpriv_args[2][30];
    int ret=0;

    CFG_get_by_name("RATECTL", buf);

    if (strlen(buf) && strncmp(buf, "auto", 4)) {

        CFG_get_by_name("MANRATE",buf);

        if (strlen(buf)) {
            strcpy(iwpriv_args[0],"set11NRates");
            strcpy(iwpriv_args[1],buf);
            ATH_IWPRIV_SET(skfd,iwpriv_args, 2, apname);
        }

        CFG_get_by_name("MANRETRIES",buf);

        if (strlen(buf)) {
            strcpy(iwpriv_args[0],"set11NRetries");
            strcpy(iwpriv_args[1],CFG_get_by_name("MANRETRIES",buf));
            ATH_IWPRIV_SET(skfd,iwpriv_args, 2, apname);
        }

    }

    CFG_get_by_name("TX_CHAINMASK", buf);

    if (strlen(buf) && strcmp(buf,"0")) {
        strcpy(iwpriv_args[0],"txchainmask");
        strcpy(iwpriv_args[1],buf);
        ATH_IWPRIV_SET(skfd,iwpriv_args, 2, "wifi0");
    }

    CFG_get_by_name("RX_CHAINMASK",buf);

    if (strlen(buf) && strcmp(buf,"0")) {
        strcpy(iwpriv_args[0],"rxchainmask");
        strcpy(iwpriv_args[1],buf);
        ATH_IWPRIV_SET(skfd, iwpriv_args, 2, "wifi0");
    }

    CFG_get_by_name("BEACONINT", buf);

    if (strlen(buf)) {
        strcpy(iwpriv_args[0],"bintval");
        strcpy(iwpriv_args[1],buf);
        ATH_IWPRIV_SET(skfd, iwpriv_args, 2, apname);
    }

    return 0;

}

static int ath_wlan_set_country_code(int skfd)
{
    char iwpriv_args[2][30];
    char buf[20];
    int ret = 0;

    CFG_get_by_name("ATH_countrycode", buf);

    if (strlen(buf)) {

        strcpy(iwpriv_args[0],"setCountryID");
        strcpy(iwpriv_args[1], buf);

        ret = iwpriv_set_private(skfd, iwpriv_args, 2, "wifi0");

        if (ret != 0) {
            fprintf(stderr,"%s: Error setting iwpriv var : %s \
                                value : %s ap : %s\n", \
                                __func__, iwpriv_args[0], \
                                 iwpriv_args[1], "wifi0");
        }
        else
            fprintf(stdout,"%s: country code %s set successfully\n", __func__, buf);

    }
    return ret;
}

static int makeVAP(char *vapmode, char *ifmode, char *apname, int staflag)
{
        char buf[100], essid[100], channel[20];
        char iwconfig_args[6][150];
        int  iwconfig_args_count  = 0;
        int  skfd = -1, ret, list_count;
        unsigned int flags = O_NONBLOCK|O_EXCL;

        if ((skfd = athcfg_sock_init(__func__)) < 0)
        {
          goto makevap_cleanup;

        }

        if (iwpriv_init(skfd, "wifi0") != 0)
            goto makevap_cleanup;

        if (ath_wlan_set_country_code(skfd) != 0)
            goto makevap_cleanup;

        ret = wlan_create_interface(skfd, apname, "wifi0", vapmode, staflag);

        if (ret == -1)
            goto makevap_cleanup;

        CFG_get_by_name("TXQUEUELEN", buf);

        if (!strlen(buf))
            strcpy(buf,"1000");

        ret = athcfg_ifconfig_ops("wifi0", "txqueuelen", buf);

        if (ret == -1)
            goto makevap_cleanup;

        ret = athcfg_ifconfig_ops(apname, "txqueuelen", buf);

        if (ret == -1)
            goto makevap_cleanup;


        if (iwpriv_init(skfd, apname) != 0)
            goto makevap_cleanup;

        //fprintf(stdout,"%s: iwpriv_init for wifi0 and %s success\n", __func__, apname);

        list_count = (sizeof(iwpriv_list1)/sizeof(iwpriv_list1[0]));

        if (iwpriv_set_default_commands(skfd, apname, iwpriv_list1, list_count) != 0) {
            fprintf(stdout,"%s: iwpriv default set list1 failed\n", __func__);
            goto makevap_cleanup;
        }

        if (iwpriv_set_custom_commands_set_1(skfd, apname) != 0) {
            fprintf(stdout,"%s: iwpriv custom cmd set_1 failed\n", __func__);
            goto makevap_cleanup;
        }

        list_count = (sizeof(iwpriv_list2)/sizeof(iwpriv_list2[0]));

        if (iwpriv_set_default_commands(skfd, apname, iwpriv_list2, list_count) != 0) {
            fprintf(stdout,"%s: iwpriv default set list2 failed\n", __func__);
            goto makevap_cleanup;
        }

        CFG_get_by_name("AP_SSID",essid);

        if (strlen(essid)) {
            if (strncmp(essid,"_any", 4)) {
                strcpy(iwconfig_args[iwconfig_args_count],"essid");
                strcpy(iwconfig_args[++iwconfig_args_count],essid);
            }
        }
        else
        {
            fprintf(stderr, "%s: AP_SSID not set, switching to default essid Atheros_XSpan_2G\n", __func__);
            strcpy(iwconfig_args[iwconfig_args_count],"essid");
            strcpy(iwconfig_args[++iwconfig_args_count],"Atheros_XSpan_2G");
        }

        if (iwconfig_args_count != 0)
              iwconfig_args_count ++;

        strcpy(iwconfig_args[iwconfig_args_count],"mode");
        strcpy(iwconfig_args[++iwconfig_args_count], ifmode);

        CFG_get_by_name("AP_PRIMARY_CH", buf);

        /* check for auto channel */
        if (strncmp(buf,"11ng", 4) && strncmp(buf,"11na", 4)) {
            strcpy(iwconfig_args[++iwconfig_args_count], "freq");
            strcpy(iwconfig_args[++iwconfig_args_count],
                    CFG_get_by_name("AP_PRIMARY_CH", channel));
        }/* else {
            strcpy(iwconfig_args[iwconfig_args_count], "freq");
            strcpy(iwconfig_args[++iwconfig_args_count], "auto");
        } */

        iwconfig_args_count ++;

        if (iwconfig_set_info(skfd, iwconfig_args, iwconfig_args_count, apname) != 0) {
                fprintf(stderr, "%s: iwconfig error\n", __func__);
                wlan_destroy_interface(skfd, "ath0");
                goto makevap_cleanup;
        }

        if (iwpriv_set_custom_commands_set_2(skfd, apname) != 0) {
            fprintf(stdout,"%s: iwpriv custom cmd set_2 failed\n", __func__);
            goto makevap_cleanup;
        }

        /* An extra IE is provided for Intel interop */
        system("echo 1 > /proc/sys/dev/ath/htdupieenable");

        fprintf(stdout,"makeVaP for %s successful\n", apname);
        athcfg_sock_deinit(skfd);
        return 0;

makevap_cleanup:
        if (skfd != -1)
           athcfg_sock_deinit(skfd);
        fprintf(stderr,"%s: error in create wlan interfaces\n", __func__);
        iwpriv_deinit();

        syscall(__NR_delete_module, "ath_hal", flags);
        syscall(__NR_delete_module, "ath_pci", flags);
        return (-1);

}

static int ath_generate_sec_file(char *fname, char *destfile)
{
    char            Name[32];
    char            Value[64];
    FILE            *f1, *f2;

    /*
    ** Code Begins.
    ** Input the parameter cache for processing
    */

    f1 = fopen(fname,"r");

    if ( !f1 )
    {
        sprintf(Value,"file %s fopen",fname);
        perror(Value);
        return (-1);
    }

    f2 = fopen(destfile,"w");

    if ( !f2 )
    {
        sprintf(Value,"file %s fopen",destfile);
        perror(Value);
        fclose(f1);
        return (-1);
    }

    ModeFlag = 1;

    /*
    ** Read the file, one line at a time.  If the line is aborted, then
    ** dump the line and continue
    */

    while(!feof(f1))
    {
        cmdLine[0] = 0;
        fgets(cmdLine,1024,f1);
        expandLine(cmdLine,opBuff);

        if (!AbortFlag)
            fprintf(f2,"%s",opBuff);
        else
        {
            //printf("%s: abortflag set\n", __func__);
            AbortFlag = 0;
        }

        opBuff[0] = 0;  // clear the buffer for the next cycle

    }

    fclose ( f1 );
    fclose ( f2 );

    return (0);
}

static int ath_generate_toplogy_cfg_file(char *apname, 
                                         char *bridge_name, 
                                         char *secfile)
{
    char buf[50];
    FILE *fp;


    fp = fopen("/var/run/topology.conf","w");


    if (!fp)
    {
        fprintf(stderr,"%s: cannot open file %s for security configuration\n", __func__, buf);
        return -1;
    }

    if (!strlen(bridge_name)) 
        fprintf(fp,"bridge none\n");
    else {
            fprintf(fp,"bridge %s\n", bridge_name);

            fprintf(fp,"{\n"); 


            fprintf(fp,"\tipaddress %s\n",CFG_get_by_name("AP_IPADDR", buf));

            fprintf(fp,"\tipmask %s\n", CFG_get_by_name("AP_NETMASK", buf));

            fprintf(fp, "\tinterface %s\n", apname);
            fprintf(fp, "\tinterface eth0\n");
            fprintf(fp, "\tinterface eth1\n");

            fprintf(fp,"}\n");
    }

    fprintf(fp,"radio wifi0\n");
    fprintf(fp,"{\n");
    fprintf(fp,"\tap\n");
    fprintf(fp,"\t{\n");
    fprintf(fp,"\t\tbss %s\n", apname);
    fprintf(fp,"\t\t{\n");
    fprintf(fp,"\t\t\tconfig %s\n", secfile);
    fprintf(fp,"\t\t}\n");
    fprintf(fp,"\t}\n");
    fprintf(fp,"}\n");

    fclose(fp);
    return 0;
}

int athcfg_prepare_nvram(FILE *fp, const char *name, int save_wps_cfg)
{
    int fd = 0;
    struct mtd_info_user mtdInfo;
    struct erase_info_user mtdEraseInfo;
    int ret;
    char *ptr;
    char buf[50];

    if (save_wps_cfg) {

        bzero(opBuff, sizeof(opBuff) - 1 );

        fseek(fp, ATHCFG_NVRAM_WPS_OFFSET, SEEK_SET);

        ret = fread(opBuff, ATHCFG_NVRAM_WPS_CFG_SIZE, 1, fp);

        if (ferror(fp) || ret != 1) {
            sprintf(buf,"\n%s: fread wps cfg from %s error",__func__, name);
            perror(buf);
            return -1;
        }

        opBuff[ATHCFG_NVRAM_WPS_CFG_SIZE]='\0';

        if ((ptr = strstr(opBuff,"WSC_")) == NULL) {
            fprintf(stdout,"%s: nvram doesn't have wps cfg data\n", __func__);
            strcpy(opBuff,"NO_WPS_CFG");
        }
    }

    /*
     * Proceed with erasing the nvram sector using
     * mtd layer ioctl
     */

    fd = fileno(fp);

    if(ioctl(fd, MEMGETINFO, &mtdInfo)) {
        fprintf(stderr, "%s: Could not get MTD device info from %s\n", __func__, name);
        return -1;
    }

    mtdEraseInfo.start = 0;
    mtdEraseInfo.length = mtdInfo.erasesize;

    if (ioctl (fd, MEMERASE, &mtdEraseInfo) < 0) {
        fprintf(stderr, "%s: Erasing mtd failed: %s errno = %d\n", __func__, name, errno);
        return -1;
    }

    fprintf(stdout,"%s: %s %d bytes erased successfully\n", __func__, name, mtdInfo.erasesize);

    rewind(fp);
    return 0;
}

int athcfg_save_wps_cfg_to_nvram(char *fname, FILE *f, int wps_write_from_buffer)
{
    FILE *fp1, *fp2;
    char buf[100], buf1[50];
    char *nvram="/dev/nvram";
    int wps_configured;
    struct stat st;
    int ret=-1;

    /*
     * If needed, just write the wps cfg alone to the
     * nvram and return.
     */

    if (wps_write_from_buffer) {

        if (strlen(opBuff) && !strcmp(opBuff,"NO_WPS_CFG"))
            return 0;

        fseek(f, ATHCFG_NVRAM_WPS_OFFSET, SEEK_SET);

        ret = fwrite(opBuff, ATHCFG_NVRAM_WPS_CFG_SIZE, 1, f);

        if (ferror(f) || ret != 1) {
            sprintf(buf,"%s: fwrite wps cfg to /dev/nvram error",__func__);
            perror(buf);
            return -1;
        }
        else
            return 0;
    }

    fp1 = fopen(nvram, "r+");

    /*
     * Open the wps config file
     */

    fp2 = fopen(fname, "r");

    if (!fp1 || !fp2)
    {
        sprintf(buf,"%s: fopen %s error",__func__, (fp1 == NULL ? nvram : fname));
        perror(buf);
        return -1;
    }

    if (fstat(fileno(fp2), &st) != 0) {
        sprintf(buf,"%s: fstat %s error\n", __func__, fname);
        goto athcfg_save_wps_cfg_cleanup;
    }

    bzero(opBuff, sizeof(opBuff)-1);

    /*
     * Read the existing wlan cfg data from nvram
     */

    ret = fread(opBuff, ATHCFG_NVRAM_WLAN_CFG_SIZE, 1, fp1);

    if (ferror(fp1) || ret != 1) {
        sprintf(buf,"\n%s: fread wlan cfg from %s error",__func__, nvram);
        perror(buf);
        ret = -1;
        goto athcfg_save_wps_cfg_cleanup;
    }

    if (athcfg_prepare_nvram(fp1, nvram, 0) != 0) {
        goto athcfg_save_wps_cfg_cleanup;
    }

    rewind(fp1);

    /*
     * Now that the nvram sector is erased, restore
     * the wlan cfg data alone
     */

    ret = fwrite(opBuff, ATHCFG_NVRAM_WLAN_CFG_SIZE, 1, fp1);

    if (ferror(fp1) || ret != 1) {
        sprintf(buf,"%s: fwrite wlan cfg to /dev/nvram error",__func__);
        perror(buf);
        ret = -1;
        goto athcfg_save_wps_cfg_cleanup;
    }

    /*
     * Prepare the nvram for WPS cfg data
     */

    fseek(fp1, ATHCFG_NVRAM_WPS_OFFSET, SEEK_SET);

    fprintf(fp1,"%s=%d\n", "/tmp/WSC_ath0.conf", st.st_size);

    bzero(opBuff, sizeof(opBuff) - 1);


    /*
     * Read the entire wps cfg data from wps config
     * file in one shot
     */

    ret = fread(opBuff, st.st_size, 1, fp2);

    if (ferror(fp2) || ret != 1) {
        sprintf(buf,"\n%s: fread wps cfg from %s error",__func__, fname);
        perror(buf);
        ret = -1;
        goto athcfg_save_wps_cfg_cleanup;
    }

    /*
     * Write the wps cfg data to nvram
     */

    ret = fwrite(opBuff, st.st_size, 1, fp1);

    if (ferror(fp1) || ret != 1) {
        sprintf(buf,"%s: fwrite wps cfg to /dev/nvram error",__func__);
        perror(buf);
        ret = -1;
        goto athcfg_save_wps_cfg_cleanup;
    }

    /* Finally a good fsync */

    fsync(fileno(fp1));

    ret = 0;

athcfg_save_wps_cfg_cleanup:

    fclose(fp1);
    fclose(fp2);

    return ret;

}

static int athcfg_check_nvram_wps(char *fname, char *apname)
{
    FILE *fp1, *fp2;
    char buf[100], buf1[50], *ptr, *ptr1;
    int wps_configured;
    int  wps_cfg_len, ret=-1;

    fp1 = NULL;
    fp2 = NULL;

    fp1 = fopen("/dev/nvram", "r");

    if (!fp1) {
        sprintf(buf,"%s: fopen %s error",__func__, (fp1 == NULL ? "/dev/nvram" : fname));
        perror(buf);
        return -1;
    }

    fseek(fp1, ATHCFG_NVRAM_WPS_OFFSET, SEEK_SET);

    if (fgets(buf,sizeof(buf), fp1) != NULL) {

        /*
         * Check if the nvram has the wps cfg data.
         * There won't be any, if a factory reset has been 
         * issued.
         */

        if ((ptr = strstr(buf,apname)) == NULL) {
            fprintf(stdout,"%s: nvram doesn't have wps cfg data\n", __func__);
            ret = 0;
            goto athcfg_check_nvram_wps_cleanup;
        }
        else
        {
            /*
             * Get the wps cfg size in the nvram
             */

            ptr = strchr(buf,'=');

            if (ptr) {

                ptr ++;

                if (sscanf(ptr,"%d\n", &wps_cfg_len) != 1) {
                    fprintf(stdout,"%s:%d wps cfg data in nvram is corrupted\n",\
                    __func__, __LINE__);
                    ret = 0;
                    goto athcfg_check_nvram_wps_cleanup;
                }

                if (wps_cfg_len <= 0 || wps_cfg_len > 2500) {
                    fprintf(stdout,"%s:%d wps cfg data in nvram is corrupted: \
                    invalid wps_cfg_len\n", __func__, __LINE__);
                    ret = 0;
                    goto athcfg_check_nvram_wps_cleanup;
                }

            }
            else
            {
                fprintf(stdout,"%s:%d wps cfg data in nvram is corrupted\n", \
                __func__, __LINE__);
                ret = 0;
                goto athcfg_check_nvram_wps_cleanup;
            }

            bzero(opBuff, sizeof(opBuff)-1);

            //fprintf(stdout, "%s: wps_cfg_len = %d\n", __func__, wps_cfg_len);

            /*
             * Read the entire wps cfg from the nvram to the array
             */

            ret = fread(opBuff, wps_cfg_len, 1, fp1);

            if (ferror(fp1) || ret != 1) {
                sprintf(buf,"\n%s: fread wps cfg from %s error",__func__, fname);
                perror(buf);
                ret = -1;
                goto athcfg_check_nvram_wps_cleanup;
            }

            /*
             * NVRAM has valid wps cfg data, now check for the
             * value of wps_configured variable
             */

            if ((ptr=strstr(opBuff,"wps_configured=")) != NULL) {

                ptr1 = strchr(ptr, '=');

                if (!ptr1) {
                    fprintf(stdout,"%s-%d: wps cfg data in nvram is corrupted\n", __func__, __LINE__);
                    ret = 0;
                    goto athcfg_check_nvram_wps_cleanup;
                }

                ptr1 ++;

                if (sscanf(ptr1,"%d\n", &wps_configured) != 1) {
                    fprintf(stdout,"%s-%d: wps cfg data wps_configured read from i\
                    nvram error\n", __func__, __LINE__);
                    ret = 0;
                    goto athcfg_check_nvram_wps_cleanup;
                }
                else
                {
                    if (wps_configured < 0 || wps_configured > 1) {
                        fprintf(stdout,"%s-%d: invalid wps_configured value from nvram\n", \
                        __func__, __LINE__);
                        ret = 0;
                        goto athcfg_check_nvram_wps_cleanup;
                    }
                }

                //printf("%s: wps_configured = %d\n", __func__, wps_configured);

                /*
                 * if wps_configured is 0, then just return 0
                 * so that a fresh wps cfg will be generated
                 */

                if (wps_configured == 0) {
                    ret = 0;
                    goto athcfg_check_nvram_wps_cleanup;
                }

                /*
                 * Now the dirty work of restoring the wps cfg
                 * from nvram starts
                 */

                fp2 = fopen(fname,"w");

                if (!fp2)
                {
                    sprintf(buf,"%s: fopen %s error",__func__, fname);
                    perror(buf);
                    goto athcfg_check_nvram_wps_cleanup;
                }

                /*
                 * Now that wps cfg data has been read from nvram,
                 * write the same into the /tmp/WSC_ath0.conf
                 */

                ret = fwrite(opBuff, wps_cfg_len, 1, fp2);

                if (ferror(fp2) || ret != 1) {
                    sprintf(buf,"%s: fwrite wps cfg to %s error",__func__, fname);
                    perror(buf);
                    ret = -1;
                    goto athcfg_check_nvram_wps_cleanup;
                }

                fsync(fileno(fp2));

                //fprintf(stdout,"%s: wps cfg data from nvram written to %s\n", __func__, fname);

                ret = 1;

            } /* end of if */
        } /* end of else */
    }
    else  /* fgets check */
        ret = -1;


athcfg_check_nvram_wps_cleanup:

    if (fp1 != NULL)
        fclose(fp1);
    if (fp2 != NULL)
        fclose(fp2);

    return ret;
}

static int activateVAP(char *apname, char *bridge_name, char *mode)
{
      int ret, wephex, i , skfd = -1;
      struct bridge *br;
      char buf[50], wepkey[150], iwconfig_args[3][150], wephexyes[10];
      char secfile[20], secfiletrans[50];
      char iwpriv_args[2][30];

      if ((skfd = athcfg_sock_init(__func__)) < 0)
      {
          fprintf(stderr, "%s: socket error: %d\n", __func__, errno);
          return -1;
      }

      if (athcfg_ifconfig_ops(apname, "ifstate", "up") != 0) {
         fprintf(stderr,"%s: cannot bring up the interface : %s error: %d", __func__, apname, errno);
         athcfg_sock_deinit(skfd);
         return -1;;
      }

      if (bridge_add_del_interface(bridge_name, apname, 1) != 0) {
          fprintf(stderr,"%s: cannot add %s to bridge %s\n", __func__, apname, bridge_name);   
          goto actvap_cleanup;
      }

      sprintf(cmdLine,"arping -U -c 1 -I %s %s", bridge_name, CFG_get_by_name("AP_IPADDR", buf)); 
      system(cmdLine);

      CFG_get_by_name("WPS_ENABLE",buf);

      if (strlen(buf) && !strncmp(buf,"1",1)) {
         CFG_get_by_name("AP_SECFILE",buf);

         if (strlen(buf) && !strncmp(buf,"EAP", 3)) {
             fprintf(stderr,"%s: eap is not supported in wps mode\n",__func__);
             br_cmd_delif(br, apname);
             goto actvap_cleanup;
         }

         sprintf(buf,"/tmp/WSC_%s.conf", apname);

         /*
          * Before generating the wps cfg file, check
          * the nvram for the presence of wps cfg data
          * and if present wps_configured variable should
          * be set to 1. If one this conditions fail,
          * then we'll go with generating a fresh wps
          * cfg file
          */

         ret = athcfg_check_nvram_wps(buf, apname);

         if (ret >= 0) {

             fprintf(stdout,"%s: wps_configured = %d in nvram\n", __func__, ret);

             /*
              * Either there is no wps cfg data in the nvram
              * else wps_configured is 0, so proceed with the
              * generation of new WSC_ath0.conf file
              */

             if (ret == 0) {
                 if (ath_generate_sec_file("/etc/ath/WSC.conf", buf) != 0)
                 {
                     fprintf(stderr,"%s: WSC.conf translation failed \n",__func__);
                     goto actvap_cleanup;
                     return -1;
                 }
             }

             /*
              * Generate the topology.conf file from the
              * ap, bridge, ethernet intf details
              */

             if (ath_generate_toplogy_cfg_file(apname, bridge_name, buf) != 0)
             {
                 fprintf(stderr,"%s: WPS topology.conf generation failed \n",__func__);
                 goto actvap_cleanup;
             }

             if (ret == 0) {

                 /*
                  * Save the generated WSC_ath0.conf file to nvram
                  */

                 if (athcfg_save_wps_cfg_to_nvram(buf, NULL, 0) != 0) {
                     fprintf(stderr,"%s: saving wps cfg %s to /dev/nvram failed \n",__func__);
                     goto actvap_cleanup;
                 }
                 else
                     fprintf(stdout,"%s: wps cfg saved to /dev/nvram\n", __func__);
             }
             else if (ret == 1) 
             {
                 fprintf(stdout,"%s: wps cfg data from nvram restored successfully\n", __func__);
                 ret = 0;
             }
         }
         else if (ret == -1)
         {
             goto actvap_cleanup;
         }
      } else {

         CFG_get_by_name("AP_SECMODE",buf);

         if (!strncmp(buf,"None",4)) {
            fprintf(stdout,"%s: ap %s activated\n",__func__, apname);
            return 0;
         } else
            if (!strncmp(buf,"WPA",3)) {
                 sprintf(buf,"/tmp/sec%s", apname);

                 sprintf(secfiletrans,"/etc/ath/%s.ap_bss", 
                         CFG_get_by_name("AP_SECFILE",secfile));

                 if (ath_generate_sec_file(secfiletrans, buf) != 0)
                 {
                     fprintf(stderr,"%s: WSC.conf translation failed \n",__func__);
                     goto actvap_cleanup;
                 }

                 if (ath_generate_toplogy_cfg_file(apname, bridge_name, buf) != 0)
                 {
                     fprintf(stderr,"%s: WPA topology.conf generation failed \n",__func__);
                     goto actvap_cleanup;
                 }
         } else
             if (!strncmp(buf,"WEP",3)) {

                CFG_get_by_name("AP_WEP_MODE", buf);

                if (strlen(buf)) {
                    strcpy(iwpriv_args[0],"authmode");
                    strcpy(iwpriv_args[1],buf);

                    ret = iwpriv_set_private(skfd, iwpriv_args, 2, apname);

                    if (ret != 0) {
                        fprintf(stderr,"%s: Error setting iwpriv var : %s \
                                            value : %s ap : %s\n", \
                                            __func__, iwpriv_args[0],  \
                                             iwpriv_args[1], apname);
                        goto actvap_cleanup;
                    }
                }

                for(i=1; i < 5; i ++)
                {
                    sprintf(buf,"WEPKEY_%d", i);

                    CFG_get_by_name(buf, wepkey);

                    if (strlen(wepkey))
                    {
                        ModeFlag = 1;
                        wephex = isKeyHex(wepkey, KEY_IS_WEP);

                        if (wephex == -1)
                            goto actvap_cleanup;

                        strcpy(iwconfig_args[0],"enc");

                        if (wephex == 1)
                                strcpy(iwconfig_args[1], wepkey);
                        else if (wephex == 0)
                                sprintf(iwconfig_args[1],"s:%s", wepkey);
                        else {
                            fprintf(stderr,"%s: unsupported wep key type\n");
                            goto actvap_cleanup;
                        }

                        sprintf(iwconfig_args[2],"[%d]",i);

                        if (iwconfig_set_info(skfd, iwconfig_args, 3, apname) != 0) {
                            fprintf(stderr,"%s: not able to set wep keys in ap %s\n", __func__, apname);
                            goto actvap_cleanup;
                        }
                    }
                }

                CFG_get_by_name("AP_PRIMARY_KEY",wepkey);

                strcpy(iwconfig_args[0],"enc");

                if (strlen(wepkey)) {
                    sprintf(iwconfig_args[1],"[%s]",wepkey);
                }
                else
                {
                    fprintf(stderr,"%s: ap primary key not set.. defaulting to 1\n");
                    strcpy(iwconfig_args[1],"[1]");
                }

                if (iwconfig_set_info(skfd, iwconfig_args, 2, apname) != 0) {
                    fprintf(stderr,"%s: not able to set primary wep key in ap %s\n", __func__, apname);
                    goto actvap_cleanup;
                }


                return 0;
          }
          else {
             fprintf(stderr,"%s: unsupported security mode %s\n",__func__, buf);
             br_cmd_delif(br, apname);
             return -1;
         }
      }

      /* start the hostapd */
      system("hostapd /var/run/topology.conf &");
      athcfg_sock_deinit(skfd);
      iwpriv_deinit();
      return 0;

actvap_cleanup:

    if (skfd != -1)
        athcfg_sock_deinit(skfd);

    bridge_add_del_interface(bridge_name, apname, 2);

    athcfg_ifconfig_ops(apname, "ifstate", "down");

    iwpriv_deinit();

    return -1;

}


static int killVAP(char *apname, char *bridge_name)
{
      struct bridge *br;
      int skfd = -1;
      char buf[20];

      if (ath_br_init() != 0)
          return -1;

      if ((skfd = athcfg_sock_init(__func__)) < 0)
      {
          fprintf(stderr, "%s: socket error: %d\n", __func__, errno);
          return -1;

      }

#if 0
      CFG_get_by_name("AP_SECMODE", buf);

      if ((strlen(buf) != 0) && (strcmp(buf,"None")))
#endif
      /*
       * There might be a case user did cfg wlan up; cfg -x;
       * If we bring down the wlan in this scenario
       * and based on the security settings, hostapd won't be
       * killed, so lets skip the check (similar
       * to what the scripts do)
       */

      /* kill hostapd */
      system("killall hostapd");

      br = NULL;

      if (ath_get_brlist() != 0) {
         fprintf(stderr,"%s: cannot get the bridge list %s\n", __func__, bridge_name);
         goto killvap_cleanup;
      }

      br = NULL;

      br =  br_find_bridge(bridge_name);

      if (br == NULL) {
         fprintf(stderr,"%s: cannot find the bridge %s\n", __func__, bridge_name);
      }
      else {

          if (br_cmd_delif(br, apname) != 0) {
             fprintf(stderr,"%s: cannot delete %s from the bridge %s\n", __func__, apname, bridge_name);
             //goto killvap_cleanup;
          }
      }

      if (athcfg_ifconfig_ops(apname, "ifstate", "down") != 0) {
         fprintf(stderr,"%s: cannot bring down the interface : %s error: %d", __func__, apname, errno);
         //goto killvap_cleanup;
      }

      if (wlan_destroy_interface(skfd, apname) != 0) {
         fprintf(stderr,"%s: cannot destroy interface : %s error: %d", __func__, apname, errno);
         //goto killvap_cleanup;
      }

killvap_cleanup:

      if (skfd != -1)
         athcfg_sock_deinit(skfd);

      ath_br_deinit();

      return 0 ;

}


struct ath_cfg_table
{
    char varname[20];
    char varvalue[30];
} const static ath_def_cfg_table[] = {
    { "AP_IPADDR", "192.168.1.2" },
    { "AP_NETMASK", "255.255.255.0" } ,
    { "WAN_MODE", "bridged" },
    { "WAN_IPADDR", "192.168.2.1" },
    { "WAN_NETMASK", "255.255.255.0" },
    { "WLAN_ON_BOOT", "n" },
    { "AP_STARTMODE", "standard" },
    { "AP_PRIMARY_CH", "6" },
    { "AP_CHMODE", "11NGHT20" },
    { "PUREG", "0" },
    { "PUREN", "0" },
    { "TXQUEUELEN", "1000" },
    { "SHORTGI", "1" },
    { "AMPDUENABLE", "1" },
    { "AMPDUFRAMES", "32" },
    { "AMPDULIMIT", "50000" },
    { "AMPDUMIN", "32768" },
    { "CWMMODE", "1" },
    { "RATECTL", "auto" },
    { "MANRATE", "0x8c8c8c8c" },
    { "MANRETRIES", "0x04040404" },
    { "RX_CHAINMASK","1" },
    { "TX_CHAINMASK","1" },
    { "SWAP_LED", "1" },
    { "LED_CUSTOM", "3" },
    { "AP_SSID", "Atheros_XSpan_2G" },
    { "AP_MODE","ap"},
#if 0
    { "AP_MODE_2","ap"},
    { "AP_MODE_3","ap"},
    { "AP_MODE_4","ap"},
    { "AP_MODE_5","ap"},
    { "AP_MODE_6","ap"},
    { "AP_MODE_7","ap"},
    { "AP_MODE_8","ap"},
#endif
    { "AP_SECMODE", "None" },
#if 0
    { "AP_SECMODE_2", "None" },
    { "AP_SECMODE_3", "None" },
    { "AP_SECMODE_4", "None" },
    { "AP_SECMODE_5", "None" },
    { "AP_SECMODE_6", "None" },
    { "AP_SECMODE_7", "None" },
    { "AP_SECMODE_8", "None" },
#endif

    { "AP_SECFILE", "PSK" },
#if 0
    { "AP_SECFILE_2", "PSK" },
    { "AP_SECFILE_2", "PSK" },
    { "AP_SECFILE_2", "PSK" },
    { "AP_SECFILE_2", "PSK" },
    { "AP_SECFILE_2", "PSK" },
    { "AP_SECFILE_2", "PSK" },
    { "AP_SECFILE_2", "PSK" },
#endif
    { "WPS_ENABLE", "0" },
#if 0
    { "WPS_ENABLE_2", "0" },
    { "WPS_ENABLE_3", "0" },
    { "WPS_ENABLE_4", "0" },
    { "WPS_ENABLE_5", "0" },
    { "WPS_ENABLE_6", "0" },
    { "WPS_ENABLE_7", "0" },
    { "WPS_ENABLE_8", "0" },
#endif
    { "WEP_IS_HEX1", "0" },
    { "WEP_IS_HEX2", "0" },
    { "WEP_IS_HEX3", "0" },
    { "WEP_IS_HEX4", "0" },
#if 0
    { "WEP_IS_HEX5", "0" },
    { "WEP_IS_HEX6", "0" },
    { "WEP_IS_HEX7", "0" },
    { "WEP_IS_HEX8", "0" },
#endif
    { "WPA_IS_HEX", "0" },
#if 0
    { "WPA_IS_HEX_2", "0" },
    { "WPA_IS_HEX_3", "0" },
    { "WPA_IS_HEX_4", "0" },
    { "WPA_IS_HEX_5", "0" },
    { "WPA_IS_HEX_6", "0" },
    { "WPA_IS_HEX_7", "0" },
    { "WPA_IS_HEX_8", "0" },
#endif
    { "ATH_use_eeprom", "0" },
    { "AP_NO_A_BAND", "1" },
    { "WAN_IF", "eth0" },
    { "LAN_IF", "eth1" },
    { "DEBUGMODE", "0x100" },
    { "HALDEBUG", "0x0" },
    { "ATHDEBUG", "0x0" }
};

void athcfg_set_default_config_values()
{
    int i;
    char buf[80];

    for (i=0;i < (sizeof(ath_def_cfg_table)/sizeof(ath_def_cfg_table[0])); \
         i++) {
                CFG_get_by_name(((char *)(ath_def_cfg_table[i].varname)), buf);
                if (strlen(buf) == 0)
                    CFG_set_by_name(((char *)(ath_def_cfg_table[i].varname)),
                                    ((char *)(ath_def_cfg_table[i].varvalue)));
    }

}

static int athcfg_get_wlan_status(int down)
{
    FILE *fp;
    char buf[100];
    int i=0;
    fp = fopen("/proc/modules","r");

    if (!fp)
    {
        perror("fopen");
        return -2;
    }

    while (!feof(fp))
    {
        fgets(buf,sizeof(buf),fp);
        if (down) {

            if (i == 2)
                return 0;

            if (!strncmp(buf,"ath_hal", 7)) {
                i++;
                continue;
            }

            if (!strncmp(buf,"ath_pci", 7))
                i++;

        }
        else {
            if ((!strncmp(buf,"ath_hal", 7)) ||
                 (!strncmp(buf,"ath_pci", 7))) {
                return -1;
            }
        }
    }

    fclose(fp);

    if (down && i == 0)
        return -1;

    return  0;
}

static int athcfg_wlan_up()
{
    int ret, staflag = 0, apwds = 0, stawds = 0, athind = 0, stafwd = 0;
    unsigned int flags = O_NONBLOCK|O_EXCL;
    char buf[50], mode[50], apname[IFNAMSIZ];

    ret = athcfg_get_wlan_status(0);

    if (ret == -2)
    {
        fprintf(stderr,"%s: error in getting the wlan  \
                module status\n", __func__);
        return -1;
    }

    if (ret == -1) {
        fprintf(stderr,"%s: wlan modules already loaded\n",
                __func__);
        return -1;
    }

    if (system("insmod \
               /lib/modules/2.6.15/net/ath_hal.ko 2>/dev/null")
                 == -1) {
        fprintf(stderr, "athcfg: error in inserting wlan module ath_hal\n");
        return -2;
    }


    if (system("insmod \
               /lib/modules/2.6.15/net/ath_pci.ko 2>/dev/null")
                == -1) {
        syscall(__NR_delete_module, "ath_hal", flags);
        fprintf(stderr,"athcfg: error in inserting wlan module ath_pci\n");
        return -2;
    }

    if (ath_br_init() != 0) {
        fprintf(stderr,"athcfg: error initializing bridge\n");
        goto ath_unload_modules;
    }

    CFG_get_by_name("AP_STARTMODE", buf);

    if (strlen(buf) && !strncmp(buf,"standard", 8)) {

        strcpy(buf,"ap");
        strcpy(mode,"master");
        strcpy(apname, "ath");

        ret = makeVAP(buf, mode, apname, 0);

        if (ret == -1) {
            fprintf(stderr,"%s: standard mode makeVAP error\n",__func__);
            goto ath_unload_modules;
        }

        ret = activateVAP("ath0", "br0", mode);

        if (ret == -1) {
            fprintf(stderr,"%s: standard mode activateVAP error\n",__func__);
            goto ath_unload_modules;
        }

        return 0;

    } else
        fprintf(stderr,"cfg: only standard mode is supported");

#if 0
    else if (!strncmp(CFG_get_by_name("AP_STARTMODE",buf),"sta", 3)) {

        strcpy(buf,"sta");
        strcpy(mode,"mode managed");
        staflag = 1;
    } else if (!strncmp(CFG_get_by_name("AP_STARTMODE",buf),"rootap", 6)) {
        strcpy(buf,"ap");
        apwds = 1;
    } else if (!strncmp(CFG_get_by_name("AP_STARTMODE",buf),"repeater", 8)) {
        strcpy(buf,"ap");
        apwds = 1;
        stawds = 1;
    } else if (!strncmp(CFG_get_by_name("AP_STARTMODE",buf),"repeater-ind", 12)) {
        strcpy(buf,"ap");
        apwds = 1;
        stawds = 1;
        athind = 1;
    } else if (!strncmp(CFG_get_by_name("AP_STARTMODE",buf),"sta-fwd-ind", 12)) {
        strcpy(buf,"sta");
        stafwd = 1;
    }  else if (!strncmp(CFG_get_by_name("AP_STARTMODE",buf),"multi", 5)) {


    }
#endif

ath_unload_modules:

syscall(__NR_delete_module, "ath_hal", flags);
syscall(__NR_delete_module, "ath_pci", flags);
return (-1);

}

static int athcfg_wlan_down()
{
    int ret;
    unsigned int flags = O_NONBLOCK|O_EXCL;

    ret = athcfg_get_wlan_status(1);

    if (ret == -1)
    {
        printf("ath_cfg: wlan modules already unloaded\n");
        exit(0);
    }

    if (killVAP("ath0","br0") != 0) 
        fprintf(stderr,"%s: killVap failed.. trying to unload the modules\n", __func__);

    if (syscall(__NR_delete_module, "ath_pci", flags) < 0) 
        perror("ath_cfg: delete_module failed for ath_pci\n");

    if (syscall(__NR_delete_module, "ath_hal", flags) < 0) 
        perror("ath_cfg: delete_module failed for ath_hal\n");


    return 0;

}

extern int mount(__const char *__special_file, __const char *__dir,
                 __const char *__fstype, unsigned long int __rwflag,
                 __const void *__data);

static int mountall()
{
    FILE *fp = 0;
    char buf[80];
    struct mntent *m=NULL;
    int ret;

    fp = setmntent("/etc/fstab", "r");

    if (!fp)
    {
            sprintf(buf,"%s: fopen", __func__);
            perror(buf);
            return -1;
    }

    while ((m = getmntent(fp)) != NULL)
    {
            if (!strcmp(m->mnt_fsname,"devpts"))
                    strcpy(buf,"mode=0622");
            else 
                    buf[0] = (char)NULL;

            ret = mount(m->mnt_fsname, m->mnt_dir, m->mnt_type, 0, (void *)buf);

            if (ret != 0)
            {
                    sprintf(buf,"%s: mount fs : %s dir : %s type=%s error" ,
                                            __func__, m->mnt_fsname, m->mnt_dir, m->mnt_type);
                    perror(buf);
                    endmntent(fp);
                    return -1;
            }
    }

    printf("%s: all filesystems mounted successfully\n",__func__);
    endmntent(fp);
    return 0;
}

static char *find_real_root_device_name(char *blockdev)
{
    DIR *dir;
    struct dirent *entry;
    struct stat statBuf, rootStat;
    char *fileName = NULL;
    dev_t dev;

    if (stat("/", &rootStat) != 0) {
        perror("could not stat '/'");
                strcpy(blockdev,"/dev/root");
                return blockdev;
        }
    else {
        /* This check is here in case they pass in /dev name */
        if ((rootStat.st_mode & S_IFMT) == S_IFBLK)
            dev = rootStat.st_rdev;
        else
            dev = rootStat.st_dev;

        dir = opendir("/dev");
        if (!dir)
            perror("could not open '/dev'");
        else {
            while((entry = readdir(dir)) != NULL) {
                const char *myname = entry->d_name;
                /* Must skip ".." since that is "/", and so we
                 * would get a false positive on ".."  */
                if (myname[0] == '.' && myname[1] == '.' && !myname[2])
                    continue;
#ifdef CONFIG_FEATURE_DEVFS
                /* if there is a link named /dev/root skip that too */
                if (strcmp(myname, "root")==0)
                    continue;
#endif
                strcat(blockdev,"/dev"); 
                fileName = strcat(blockdev, myname);

                /* Some char devices have the same dev_t as block
                 * devices, so make sure this is a block device */
                if (stat(fileName, &statBuf) == 0 &&
                        S_ISBLK(statBuf.st_mode)!=0 &&
                        statBuf.st_rdev == dev)
                        break;
                fileName = NULL;
            }
            closedir(dir);
        }
    }

    if(fileName == NULL)
        strcpy(blockdev,"/dev/root");

    return blockdev;
}

void show_mounts()
{
    FILE *mountTable = setmntent("/proc/mounts","r");
    char blockdev[40];

    if (mountTable) {
        struct mntent *m;

        while ((m = getmntent(mountTable)) != 0) {
            char *blockDevice = m->mnt_fsname;

            if (strcmp(blockDevice, "rootfs") == 0) {
                continue;
            } else if (strcmp(blockDevice, "/dev/root") == 0) {
                blockDevice = find_real_root_device_name(blockdev);
            }
            printf("%s on %s type %s (%s)\n", blockDevice, m->mnt_dir,
                       m->mnt_type, m->mnt_opts);
        }
        endmntent(mountTable);
    } else {
        sprintf(blockdev,"%s: setmntent",__func__);
        perror(blockdev);
    }
}

void usage(char *progname)
{
    char us_buf[] = {
                        "\nUsage:  cfg [addbr|delbr] bridge_name\n"
                        "\t    [braddif|brdelif] <bridge_name> <interface>\n"
                        "\t    ipaddr <interface> <ipaddress>\n"
                        "\t    netmask <interface> <netmask>\n"
                        "\t    ifstate <interface> [up|down]\n"
                        "\t    wlan [up|down|status]\n"
                        "\t    ath0 list sta\n"
                        "\t    ath0 list chan\n"
                        "\t    ath0 maccmd [0|1|2|3|4]\n"
                        "\t    ath0 addmac <macaddr>\n"
                        "\t    ath0 delmac <macaddr>\n"
                        "\t    ath0 kickmac <macaddr>\n"
                        "\t    ath0 beacon-interval <interval in millsecons>\n"
                        "\t    ath0 hide_ssid\n"
                        "\t    ath0 essid <essid>\n"
                        "\t    ath0 txpower <txpower>[dBm]\n"
                        "\t    ath0 rts <rts threshold>\n"
                        "\t    ath0 frag <frag value>\n"
                        "\t    ath0 pureg [1|0]\n"
                        "\t    ath0 dtim_period <dp>\n"
                        "\t    ath0 doth <value>\n"
                        "\t    ath0 wmm <value>\n"
                        "\t    ath0 setaddbaoper [1|0]\n"
                        "\t    boardinfo\n"
    };

    fprintf(stderr,"%s\n",us_buf);
    exit(-1);

}

static int 
athcfg_wlan_set_iwpriv_acl(const char *cmd, const char *value, int *fd)
{
    int ret, skfd;
    char iwpriv_args[2][30];

    if ((skfd = athcfg_sock_init(__func__)) < 0)
    {
        fprintf(stderr, "%s: socket error: %d\n", __func__, errno);
        return -1;
    }

    *fd = skfd;

    if (iwpriv_init(skfd, "ath0") != 0)
        return -1;

    strcpy(iwpriv_args[0], cmd);
    strcpy(iwpriv_args[1], value);

    ret = iwpriv_set_private(skfd, iwpriv_args, 2, "ath0");

    iwpriv_deinit();

    if (ret != 0) {
        fprintf(stderr,"%s: Error setting iwpriv var : %s \
                            value : %s ap : %s\n",\
                            __func__, iwpriv_args[0],\
                            iwpriv_args[1], "ath0");
        return -1;
    }

    return 0;
}


/************************* DISPLAY ROUTINES **************************/

/*------------------------------------------------------------------*/
/*
 * Get wireless informations & config from the device driver
 * We will call all the classical wireless ioctl on the driver through
 * the socket to know what is supported and to get the settings...
 */
static int
get_info(int			skfd,
	 char *			ifname,
	 struct wireless_info *	info)
{
    struct iwreq		wrq;

    memset((char *) info, 0, sizeof(struct wireless_info));

    /* Get basic information */
    if(iw_get_basic_config(skfd, ifname, &(info->b)) < 0)
    {
      /* If no wireless name : no wireless extensions */
      /* But let's check if the interface exists at all */
      struct ifreq ifr;

      strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
      if(ioctl(skfd, SIOCGIFFLAGS, &ifr) < 0)
        return(-ENODEV);
      else
        return(-ENOTSUP);
    }

    /* Get ranges */
    if(iw_get_range_info(skfd, ifname, &(info->range)) >= 0)
      info->has_range = 1;

    /* Get sensitivity */
    if(iw_get_ext(skfd, ifname, SIOCGIWSENS, &wrq) >= 0)
    {
      info->has_sens = 1;
      memcpy(&(info->sens), &(wrq.u.sens), sizeof(iwparam));
    }

    /* Get AP address */
    if(iw_get_ext(skfd, ifname, SIOCGIWAP, &wrq) >= 0)
    {
      info->has_ap_addr = 1;
      memcpy(&(info->ap_addr), &(wrq.u.ap_addr), sizeof (sockaddr));
    }

    /* Get NickName */
    wrq.u.essid.pointer = (caddr_t) info->nickname;
    wrq.u.essid.length = IW_ESSID_MAX_SIZE + 1;
    wrq.u.essid.flags = 0;
    if(iw_get_ext(skfd, ifname, SIOCGIWNICKN, &wrq) >= 0)
    if(wrq.u.data.length > 1)
      info->has_nickname = 1;

    /* Get bit rate */
    if(iw_get_ext(skfd, ifname, SIOCGIWRATE, &wrq) >= 0)
    {
      info->has_bitrate = 1;
      memcpy(&(info->bitrate), &(wrq.u.bitrate), sizeof(iwparam));
    }

    /* Get RTS threshold */
    if(iw_get_ext(skfd, ifname, SIOCGIWRTS, &wrq) >= 0)
    {
      info->has_rts = 1;
      memcpy(&(info->rts), &(wrq.u.rts), sizeof(iwparam));
    }

    /* Get fragmentation threshold */
    if(iw_get_ext(skfd, ifname, SIOCGIWFRAG, &wrq) >= 0)
    {
      info->has_frag = 1;
      memcpy(&(info->frag), &(wrq.u.frag), sizeof(iwparam));
    }

    /* Get Power Management settings */
    wrq.u.power.flags = 0;
    if(iw_get_ext(skfd, ifname, SIOCGIWPOWER, &wrq) >= 0)
    {
      info->has_power = 1;
      memcpy(&(info->power), &(wrq.u.power), sizeof(iwparam));
    }

    if((info->has_range) && (info->range.we_version_compiled > 9))
    {
      /* Get Transmit Power */
      if(iw_get_ext(skfd, ifname, SIOCGIWTXPOW, &wrq) >= 0)
        {
          info->has_txpower = 1;
          memcpy(&(info->txpower), &(wrq.u.txpower), sizeof(iwparam));
        }
    }

    if((info->has_range) && (info->range.we_version_compiled > 10))
    {
      /* Get retry limit/lifetime */
      if(iw_get_ext(skfd, ifname, SIOCGIWRETRY, &wrq) >= 0)
        {
          info->has_retry = 1;
          memcpy(&(info->retry), &(wrq.u.retry), sizeof(iwparam));
        }
    }

    /* Get stats */
    if(iw_get_stats(skfd, ifname, &(info->stats),
                  &info->range, info->has_range) >= 0)
    {
      info->has_stats = 1;
    }

    #ifdef DISPLAY_WPA
    /* Note : currently disabled to not bloat iwconfig output. Also,
    * if does not make total sense to display parameters that we
    * don't allow (yet) to configure.
    * For now, use iwlist instead... Jean II */

    /* Get WPA/802.1x/802.11i security parameters */
    if((info->has_range) && (info->range.we_version_compiled > 17))
    {
      wrq.u.param.flags = IW_AUTH_KEY_MGMT;
      if(iw_get_ext(skfd, ifname, SIOCGIWAUTH, &wrq) >= 0)
        {
          info->has_auth_key_mgmt = 1;
          info->auth_key_mgmt = wrq.u.param.value;
        }

      wrq.u.param.flags = IW_AUTH_CIPHER_PAIRWISE;
      if(iw_get_ext(skfd, ifname, SIOCGIWAUTH, &wrq) >= 0)
        {
          info->has_auth_cipher_pairwise = 1;
          info->auth_cipher_pairwise = wrq.u.param.value;
        }

      wrq.u.param.flags = IW_AUTH_CIPHER_GROUP;
      if(iw_get_ext(skfd, ifname, SIOCGIWAUTH, &wrq) >= 0)
        {
          info->has_auth_cipher_group = 1;
          info->auth_cipher_group = wrq.u.param.value;
        }
    }
    #endif

    return(0);
}

/*------------------------------------------------------------------*/
/*
 * Print on the screen in a neat fashion all the info we have collected
 * on a device.
 */
static void
display_info(struct wireless_info *	info,
	     char *			ifname)
{
    char		buffer[128];	/* Temporary buffer */

    /* One token is more of less 5 characters, 14 tokens per line */
    int	tokens = 3;	/* For name */

    /* Display device name and wireless name (name of the protocol used) */
    printf("%-8.16s  %s  ", ifname, info->b.name);

    /* Display ESSID (extended network), if any */
    if(info->b.has_essid)
    {
      if(info->b.essid_on)
        {
          /* Does it have an ESSID index ? */
          if((info->b.essid_on & IW_ENCODE_INDEX) > 1)
            printf("ESSID:\"%s\" [%d]  ", info->b.essid,
                   (info->b.essid_on & IW_ENCODE_INDEX));
          else
            printf("ESSID:\"%s\"  ", info->b.essid);
        }
      else
        printf("ESSID:off/any  ");
    }

    /* Display NickName (station name), if any */
    if(info->has_nickname)
    printf("Nickname:\"%s\"", info->nickname);

    /* Formatting */
    if(info->b.has_essid || info->has_nickname)
    {
      printf("\n          ");
      tokens = 0;
    }

    /* Display Network ID */
    if(info->b.has_nwid)
    {
      /* Note : should display proper number of digits according to info
       * in range structure */
      if(info->b.nwid.disabled)
        printf("NWID:off/any  ");
      else
        printf("NWID:%X  ", info->b.nwid.value);
      tokens +=2;
    }

    /* Display the current mode of operation */
    if(info->b.has_mode)
    {
      printf("Mode:%s  ", iw_operation_mode[info->b.mode]);
      tokens +=3;
    }

    /* Display frequency / channel */
    if(info->b.has_freq)
    {
      double		freq = info->b.freq;	/* Frequency/channel */
      int		channel = -1;		/* Converted to channel */
      /* Some drivers insist of returning channel instead of frequency.
       * This fixes them up. Note that, driver should still return
       * frequency, because other tools depend on it. */
      if(info->has_range && (freq < KILO))
        channel = iw_channel_to_freq((int) freq, &freq, &info->range);
      /* Display */
      iw_print_freq(buffer, sizeof(buffer), freq, -1, info->b.freq_flags);
      printf("%s  ", buffer);
      tokens +=4;
    }

    /* Display the address of the current Access Point */
    if(info->has_ap_addr)
    {
      /* A bit of clever formatting */
      if(tokens > 8)
        {
          printf("\n          ");
          tokens = 0;
        }
      tokens +=6;

      /* Oups ! No Access Point in Ad-Hoc mode */
      if((info->b.has_mode) && (info->b.mode == IW_MODE_ADHOC))
        printf("Cell:");
      else
        printf("Access Point:");
      printf(" %s   ", iw_sawap_ntop(&info->ap_addr, buffer));
    }

    /* Display the currently used/set bit-rate */
    if(info->has_bitrate)
    {
      /* A bit of clever formatting */
      if(tokens > 11)
        {
          printf("\n          ");
          tokens = 0;
        }
      tokens +=3;

      /* Display it */
      iw_print_bitrate(buffer, sizeof(buffer), info->bitrate.value);
      printf("Bit Rate%c%s   ", (info->bitrate.fixed ? '=' : ':'), buffer);
    }

    /* Display the Transmit Power */
    if(info->has_txpower)
    {
      /* A bit of clever formatting */
      if(tokens > 11)
        {
          printf("\n          ");
          tokens = 0;
        }
      tokens +=3;

      /* Display it */
      iw_print_txpower(buffer, sizeof(buffer), &info->txpower);
      printf("Tx-Power%c%s   ", (info->txpower.fixed ? '=' : ':'), buffer);
    }

    /* Display sensitivity */
    if(info->has_sens)
    {
      /* A bit of clever formatting */
      if(tokens > 10)
        {
          printf("\n          ");
          tokens = 0;
        }
      tokens +=4;

      /* Fixed ? */
      if(info->sens.fixed)
        printf("Sensitivity=");
      else
        printf("Sensitivity:");

      if(info->has_range)
        /* Display in dBm ? */
        if(info->sens.value < 0)
          printf("%d dBm  ", info->sens.value);
        else
          printf("%d/%d  ", info->sens.value, info->range.sensitivity);
      else
        printf("%d  ", info->sens.value);
    }

    printf("\n          ");
    tokens = 0;

    /* Display retry limit/lifetime information */
    if(info->has_retry)
    { 
      printf("Retry");
      /* Disabled ? */
      if(info->retry.disabled)
        printf(":off");
      else
        {
          /* Let's check the value and its type */
          if(info->retry.flags & IW_RETRY_TYPE)
            {
              iw_print_retry_value(buffer, sizeof(buffer),
                                   info->retry.value, info->retry.flags);
              printf("%s", buffer);
            }

          /* Let's check if nothing (simply on) */
          if(info->retry.flags == IW_RETRY_ON)
            printf(":on");
        }
      printf("   ");
      tokens += 5;	/* Between 3 and 5, depend on flags */
    }

    /* Display the RTS threshold */
    if(info->has_rts)
    {
      /* Disabled ? */
      if(info->rts.disabled)
        printf("RTS thr:off   ");
      else
        {
          /* Fixed ? */
          if(info->rts.fixed)
            printf("RTS thr=");
          else
            printf("RTS thr:");

          printf("%d B   ", info->rts.value);
        }
      tokens += 3;
    }

    /* Display the fragmentation threshold */
    if(info->has_frag)
    {
      /* A bit of clever formatting */
      if(tokens > 10)
        {
          printf("\n          ");
          tokens = 0;
        }
      tokens +=4;

      /* Disabled ? */
      if(info->frag.disabled)
        printf("Fragment thr:off");
      else
        {
          /* Fixed ? */
          if(info->frag.fixed)
            printf("Fragment thr=");
          else
            printf("Fragment thr:");

          printf("%d B   ", info->frag.value);
        }
    }

    /* Formating */
    if(tokens > 0)
    printf("\n          ");

    /* Display encryption information */
    /* Note : we display only the "current" key, use iwlist to list all keys */
    if(info->b.has_key)
    {
      printf("Encryption key:");
      if((info->b.key_flags & IW_ENCODE_DISABLED) || (info->b.key_size == 0))
        printf("off");
      else
        {
          /* Display the key */
          iw_print_key(buffer, sizeof(buffer),
                       info->b.key, info->b.key_size, info->b.key_flags);
          printf("%s", buffer);

          /* Other info... */
          if((info->b.key_flags & IW_ENCODE_INDEX) > 1)
            printf(" [%d]", info->b.key_flags & IW_ENCODE_INDEX);
          if(info->b.key_flags & IW_ENCODE_RESTRICTED)
            printf("   Security mode:restricted");
          if(info->b.key_flags & IW_ENCODE_OPEN)
            printf("   Security mode:open");
        }
      printf("\n          ");
    }

    #ifdef DISPLAY_WPA
    /* Display WPA/802.1x/802.11i security parameters */
    if(info->has_auth_key_mgmt || info->has_auth_cipher_pairwise ||
     info->has_auth_cipher_group)
    {
      printf("Auth params:");
      if(info->has_auth_key_mgmt)
        printf(" key_mgmt:0x%X ", info->auth_key_mgmt);
      if(info->has_auth_cipher_pairwise)
        printf(" cipher_pairwise:0x%X ", info->auth_cipher_pairwise);
      if(info->has_auth_cipher_group)
        printf(" cipher_group:0x%X ", info->auth_cipher_group);
      printf("\n          ");
    }
    #endif

    /* Display Power Management information */
    /* Note : we display only one parameter, period or timeout. If a device
    * (such as HiperLan) has both, the user need to use iwlist... */
    if(info->has_power)	/* I hope the device has power ;-) */
    { 
      printf("Power Management");
      /* Disabled ? */
      if(info->power.disabled)
        printf(":off");
      else
        {
          /* Let's check the value and its type */
          if(info->power.flags & IW_POWER_TYPE)
            {
              iw_print_pm_value(buffer, sizeof(buffer),
                                info->power.value, info->power.flags);
              printf("%s  ", buffer);
            }

          /* Let's check the mode */
          iw_print_pm_mode(buffer, sizeof(buffer), info->power.flags);
          printf("%s", buffer);

          /* Let's check if nothing (simply on) */
          if(info->power.flags == IW_POWER_ON)
            printf(":on");
        }
      printf("\n          ");
    }

    /* Display statistics */
    if(info->has_stats)
    {
      iw_print_stats(buffer, sizeof(buffer),
                     &info->stats.qual, &info->range, info->has_range);
      printf("Link %s\n", buffer);

      if(info->range.we_version_compiled > 11)
        printf("          Rx invalid nwid:%d  Rx invalid crypt:%d  Rx invalid frag:%d\n          Tx excessive retries:%d  Invalid misc:%d   Missed beacon:%d\n",
               info->stats.discard.nwid,
               info->stats.discard.code,
               info->stats.discard.fragment,
               info->stats.discard.retries,
               info->stats.discard.misc,
               info->stats.miss.beacon);
      else
        printf("          Rx invalid nwid:%d  invalid crypt:%d  invalid misc:%d\n",
               info->stats.discard.nwid,
               info->stats.discard.code,
               info->stats.discard.misc);
    }

    printf("\n");
}

static int athcfg_display_wlan_status()
{
    int ret, skfd;
    char iwpriv_args[2][30];
    struct wireless_info wlconfig;

    if ((skfd = athcfg_sock_init(__func__)) < 0)
    {
        fprintf(stderr, "%s: socket error: %d\n", __func__, errno);
        return -1;
    }

    if (get_info(skfd, "ath0", &wlconfig) != 0) {
        athcfg_sock_deinit(skfd);
        return -1;
    }

    display_info(&wlconfig, "ath0");

    athcfg_sock_deinit(skfd);

    return 0;
}


static void
print_chaninfo(const struct ieee80211_channel *c)
{
#define	IEEE80211_IS_CHAN_PASSIVE(_c) \
	(((_c)->ic_flags & IEEE80211_CHAN_PASSIVE))
#define IEEE80211_IS_CHAN_DFS(_c) \
    (((_c)->ic_flagext & 0x02))
    char buf[14];

    buf[0] = '\0';
    if (IEEE80211_IS_CHAN_FHSS(c))
        strlcat(buf, " FHSS", sizeof(buf));
    if (IEEE80211_IS_CHAN_11NA(c))
        strlcat(buf, " 11na", sizeof(buf));
    else if (IEEE80211_IS_CHAN_A(c))
        strlcat(buf, " 11a", sizeof(buf));
    else if (IEEE80211_IS_CHAN_11NG(c))
        strlcat(buf, " 11ng", sizeof(buf));
    /* XXX 11g schizophrenia */
    else if (IEEE80211_IS_CHAN_G(c) || IEEE80211_IS_CHAN_PUREG(c))
        strlcat(buf, " 11g", sizeof(buf));
    else if (IEEE80211_IS_CHAN_B(c))
        strlcat(buf, " 11b", sizeof(buf));
    if (IEEE80211_IS_CHAN_TURBO(c))
        strlcat(buf, " Turbo", sizeof(buf));
    if(IEEE80211_IS_CHAN_11N_CTL_CAPABLE(c))
        strlcat(buf, " C", sizeof(buf));
    if(IEEE80211_IS_CHAN_11N_CTL_U_CAPABLE(c))
        strlcat(buf, " CU", sizeof(buf));
    if(IEEE80211_IS_CHAN_11N_CTL_L_CAPABLE(c))
        strlcat(buf, " CL", sizeof(buf));
    printf("Channel %3u : %u%c%c Mhz%-14.14s",
	    ieee80211_mhz2ieee(c->ic_freq), c->ic_freq,
	    IEEE80211_IS_CHAN_PASSIVE(c) ? '*' : ' ',IEEE80211_IS_CHAN_DFS(c) ?'~':' ', buf);
#undef IEEE80211_IS_CHAN_PASSIVE
#undef IEEE80211_IS_CHAN_DFS
}

static int ath_wlan_list_channels(const char *ifname)
{
    struct iwreq iwrq, *iwr;
    int skfd;
    struct ieee80211req_chaninfo chans;
    const struct ieee80211_channel *c;
    int i, half, len, ret;

    if ((skfd = athcfg_sock_init(__func__)) < 0)
    {
        fprintf(stderr, "%s: socket error: %d\n", __func__, errno);
        return -1;
    }

    iwr = &iwrq;

    memset(iwr, 0, sizeof(iwr));

    strncpy(iwr->ifr_name, ifname, IFNAMSIZ);

    iwr->u.data.pointer =  &chans;
    iwr->u.data.length = sizeof(chans);

    ret = ioctl(skfd, IEEE80211_IOCTL_GETCHANINFO, iwr);

    if (ret != 0) {
        fprintf(stderr,"%s: IEEE80211_IOCTL_GETCHANINFO error : %d for ap %s\n", __func__,
                errno, ifname);
        athcfg_sock_deinit(skfd);
        return -1;
    }

    half = chans.ic_nchans / 2;

    if (chans.ic_nchans % 2)
            half++;

    for (i = 0; i < chans.ic_nchans / 2; i++) {
            print_chaninfo(&chans.ic_chans[i]);
            print_chaninfo(&chans.ic_chans[half+i]);
            printf("\n");
    }

    if (chans.ic_nchans % 2) {
            print_chaninfo(&chans.ic_chans[i]);
            printf("\n");
    }


    athcfg_sock_deinit(skfd);
    return 0;

}

static int ath_check_valid_num(char *arg, const char *cmd)
{

    char *endptr, *str;
    long val;
    int ret = -1, i;

    for(i=0;i < strlen(arg); i++)
    {
        if (!isdigit(arg[i])) {
            fprintf(stderr, "%s: cmd: %s, invalid digit given\n", __func__, cmd);
            return ret;
        }
    }

    errno = 0;
    str = arg;
    val = strtol(str, &endptr, 10);

    if ((errno == ERANGE && (val == LONG_MAX || val == LONG_MIN))
           || (errno != 0 && val == 0)) {
       fprintf(stderr,"%s: cmd: %s strtol error\n", __func__, cmd);
    }
    else if (endptr == str) {
       fprintf(stderr, "\n%s: cmd: %s No digits were found\n", __func__, cmd);
    }
    else
        ret = 0;

    return ret;
}

static int athcfg_wlan_set_config(char *ifname, const char *cmd, char *arg)
{

    char buf[100], essid[100], channel[20];
    char iwconfig_args[2][150];
    int skfd, ret = -1;
    char *endptr, *str;
    long val;

    skfd = athcfg_sock_init(__func__);

    if (skfd < 0)
        return -1;

    if (!strcmp(cmd,"essid")) {
        if (strlen(arg) > 32) {
            fprintf(stderr,"%s: invalid ssid length, length should be <= 32\n",__func__);
            goto ath_wlan_set_cfg_cleanup;
        }
    }
#if 0
    else if (!strcmp(cmd,"ap"))
    {
        if (strlen(arg) != 17) {
            fprintf(stderr,"%s: invalid mac-addr length, length should be 17\n",__func__);
            goto ath_wlan_set_cfg_cleanup;
        }
    }
#endif
    else if (!strcmp(cmd,"txpower"))
    {
        if (strlen(arg) > 5) {
            fprintf(stderr,"%s: invalid txpower length, length should be <= 5\n",__func__);
            goto ath_wlan_set_cfg_cleanup;
        }

    }
    else if (!strcmp(cmd,"rts")|| !strcmp(cmd,"frag"))
    {
        if (strlen(arg) > 5) {
            fprintf(stderr,"%s: invalid %s length, length should be <= 5\n",__func__, cmd);
            goto ath_wlan_set_cfg_cleanup;
        }

        if (ath_check_valid_num(arg, cmd) != 0)
            goto ath_wlan_set_cfg_cleanup;
    }


    strcpy(iwconfig_args[0],cmd);
    strcpy(iwconfig_args[1],arg);

    if (iwconfig_set_info(skfd, iwconfig_args, 2, ifname) != 0) {
            fprintf(stderr, "%s: iwconfig error\n", __func__);
            goto ath_wlan_set_cfg_cleanup;
    }

    ret = 0;

ath_wlan_set_cfg_cleanup:
    athcfg_sock_deinit(skfd);

    return ret;

}

static void athcfg_cat(char *fname)
{
        FILE *fp;
        fp = fopen(fname,"r");

        if (!fp)
        {
            sprintf(opBuff,"%s: fopen %s error",__func__, fname);
            perror(opBuff);
            exit(-1);
        }

        while (!feof(fp)) {
            if (fgets(opBuff, 80, fp) != NULL)
                    fprintf(stdout,"%s",opBuff);
        }

        fclose(fp);
}

void athcfg_process_commandline(int argc, char **argv)
{
    char  valBuff[128];
    int ret = -1;
    int fd = -1;

    if (!strcmp(argv[1],"wlan"))
    {
            if (argc < 3 || argc > 4) {
                fprintf(stderr,"\n%s: invalid wlan options\n", argv[0]);
                usage(argv[0]);
            }

            athcfg_set_default_config_values();

            if (!strcmp(argv[2],"up")) {
                if (athcfg_wlan_up() != 0)
                    exit(-1);
                else {
                    fprintf(stdout,"\n%s: wlan bringup successful\n", argv[0]);
                    exit(0);
                }
            }
            else if (!strcmp(argv[2],"down")) {
                athcfg_wlan_down();
                fprintf(stdout,"\n%s: wlan shutdown successful\n", argv[0]);
                exit(0);
            }

            ret = athcfg_get_wlan_status(1);

            if (ret == -1)
            {
                fprintf(stderr, "\n%s: wlan not active\n", argv[0]);
                exit(0);
            }

            if (!strcmp(argv[2],"status")) {
                ret = athcfg_display_wlan_status();
                exit(ret);
            }
            else
            {
                fprintf(stderr,"\n%s: invalid wlan options\n", argv[0]);
                usage(argv[0]);
            }

    }
    else if (!strcmp(argv[1],"ath0"))
    {
            if (argc < 3 || argc > 4) {
                fprintf(stderr,"\n%s: invalid ath0 options\n", argv[0]);
                usage(argv[0]);
            }

            ret = athcfg_get_wlan_status(1);

            if (ret != 0)
            {
                fprintf(stderr, "\n%s: wlan not active\n\n", argv[0]);
                exit(0);
            }
            else if (!strcmp(argv[2],"list"))
            {
                if (argc != 4)
                {
                    fprintf(stderr,"\n%s: invalid ath0 options\n",argv[0]);
                    usage(argv[0]);
                }
                else if (!strcmp(argv[3],"sta"))
                    ret = athcfg_wlan_list_stations(argv[1]);
                else if (!strcmp(argv[3],"chan"))
                    ret = ath_wlan_list_channels(argv[1]);
                else
                {
                    fprintf(stderr,"\n%s: invalid ath0 options\n",argv[0]);
                    usage(argv[0]);
                }
                exit(ret);
            }
            else if (!strcmp(argv[2],"beacon-interval")||
                     !strcmp(argv[2],"dtim_period") ||
                     !strcmp(argv[2],"doth") ||
                     !strcmp(argv[2],"setaddbaoper") ||
                     !strcmp(argv[2],"wmm") ||
                     !strcmp(argv[2],"pureg")) {


                    if (argc != 4) {
                        fprintf(stderr,"\n%s: invalid ath0 options\n",argv[0]);
                        usage(argv[0]);
                    }

                    if (ath_check_valid_num(argv[3], argv[2]) != 0) {
                        exit(EXIT_FAILURE);
                    }

                    if (strstr(argv[2],"beacon")) {
                        strcpy(argv[2],"bintval");
                    }

                    ret = athcfg_wlan_set_iwpriv_acl(argv[2], argv[3], &fd);

                    if (fd != -1)
                        close(fd);

                    exit(ret);
            }
            else if (!strcmp(argv[2],"maccmd") ||
                     !strcmp(argv[2],"hide_ssid"))
            {
                    if (argc == 4) {

                        if (ath_check_valid_num(argv[3], argv[2]) != 0) {
                           exit(EXIT_FAILURE);
                        }

                        strcpy(valBuff,argv[3]);
                    }
                    else if (strstr(argv[2],"hide"))
                            strcpy(valBuff,"1");
                    else if (strstr(argv[2], "mac") && argc != 4)
                            strcpy(valBuff,"0");

                    ret = athcfg_wlan_set_iwpriv_acl(argv[2], valBuff, &fd);

                    if (fd != -1)
                        close(fd);

                    exit(ret);
            }
            else if (!strcmp(argv[2],"addmac") ||
                     !strcmp(argv[2],"delmac") ||
                     !strcmp(argv[2],"kickmac")) {

                    int fd = -1;

                    if (argc != 4) {
                        fprintf(stderr,"\n%s: invalid wlan options\n", argv[0]);
                        usage(argv[0]);
                    }

                    if (strlen(argv[3]) != 17) {
                        fprintf(stderr,"\n%s: invalid mac address length \n", argv[0]);
                        usage(argv[0]);
                    }

                    ret = athcfg_wlan_set_iwpriv_acl(argv[2], argv[3], &fd);

                    if (fd != -1)
                        close(fd);

                    exit(ret);
            }
            else if (!strcmp(argv[2],"txpower")||
                     !strcmp(argv[2],"essid")  ||
                     !strcmp(argv[2],"rts")    ||
                     !strcmp(argv[2],"frag")) {

                    if (argc != 4) {
                        fprintf(stderr,"\n%s: invalid wlan options\n", argv[0]);
                        usage(argv[0]);
                    }
                    ret = athcfg_wlan_set_config("ath0",argv[2], argv[3]);
                    exit(ret);
            }
            else
            {
                    fprintf(stderr,"\n%s: invalid ath0 options\n", argv[0]);
                    usage(argv[0]);
            }
    }
    else if (!strcmp(argv[1],"addbr") ||
             !strcmp(argv[1],"delbr"))
    {
            int bradd=0;

            if (argc != 3) {
                fprintf(stderr,"\n%s: invalid bridge options\n", argv[0]);
                usage(argv[0]);
            }

            if (strlen(argv[2]) > 5) {
                fprintf(stderr,"\n%s: invalid bridge options\n", argv[0]);
                usage(argv[0]);
            }

            if (ath_br_init() != 0)
                exit(-1);

            if (!strcmp(argv[1],"addbr")) {
                bradd=1;
                ret = br_cmd_addbr(argv[2]);
            } else {

                ret = br_cmd_delbr(argv[2]);
            }

            ath_br_deinit();

            if (ret != 0)
               exit(-1);
            else
               fprintf(stdout,"\n%s: Bridge %s %s successfully\n", argv[0],\
                       argv[1], bradd ? "created" : "deleted");

            exit(0);
    }
    else if (!strcmp(argv[1],"braddif") ||
             !strcmp(argv[1],"brdelif"))
    {
            int addif=0;

            if (argc != 4) {
                fprintf(stderr,"\n%s: invalid bridge options\n", argv[0]);
                usage(argv[0]);
            }

            if ((strlen(argv[2]) > 5) ||
                (strlen(argv[3]) > IFNAMSIZ)) {
                fprintf(stderr,"\n%s: invalid bridge options\n", argv[0]);
                usage(argv[0]);
            }

            if (!strcmp(argv[1],"braddif"))
                addif = 1;
            else
                addif = 2;

            if (bridge_add_del_interface(argv[2], argv[3], addif) != 0)
                exit(-1);
            else
                exit(0);
    }
    else if (!strcmp(argv[1],"ipaddr"))
    {
            if (argc != 4) {
                fprintf(stderr,"\n%s: invalid ipaddr options\n", argv[0]);
                usage(argv[0]);
            }

            if (strlen(argv[2]) > IFNAMSIZ) {
                fprintf(stderr,"\n%s: invalid interface length\n", argv[0]);
                usage(argv[0]);
            }

            if (athcfg_ifconfig_ops(argv[2], "ipaddr", argv[3]) != 0)
                    exit(-1);
            else
                fprintf(stdout,"\n%s: address %s assigned to %s\n", argv[0], \
                        argv[3],argv[2]);
            exit(0);

    }
    else if (!strcmp(argv[1],"netmask"))
    {
            if (argc != 4) {
                fprintf(stderr,"\n%s: invalid netmask options\n", argv[0]);
                usage(argv[0]);
            }

            if (strlen(argv[2]) > IFNAMSIZ) {
                fprintf(stderr,"\n%s: invalid interface length\n", argv[0]);
                usage(argv[0]);
            }

            if (athcfg_ifconfig_ops(argv[2], "netmask", argv[3]) != 0)
                    exit(-1);
            else
                fprintf(stdout,"\n%s: netmask %s assigned to %s\n", argv[0], \
                        argv[3],argv[2]);
            exit(0);
    }
    else if (!strcmp(argv[1],"ifstate"))
    {
            if (argc != 4) {
                fprintf(stderr,"\n%s: invalid ifstate options\n", argv[0]);
                usage(argv[0]);
            }

            if (!strcmp(argv[3],"up") ||
                !strcmp(argv[3],"down")) {

                if (strlen(argv[2]) > IFNAMSIZ) {
                    fprintf(stderr,"\n%s: invalid interface length\n",\
                            argv[0]);
                    usage(argv[0]);
                }

                if (athcfg_ifconfig_ops(argv[2],"ifstate", argv[3]) != 0) {
                        exit(-1);
                }
                exit(0);
            } else {
                fprintf(stderr,"\n%s: invalid ifstate options\n", argv[0]);
                usage(argv[0]);
            }
    }
    else if (!strcmp(argv[1],"-?"))
    {
        usage(argv[0]);
    }
    else if (!strcmp(argv[1],"mountall"))
    {
        ret = mountall();
        exit(ret);
    }
    else if (!strcmp(argv[1],"mount"))
    {
        show_mounts();
        exit(0);
    }
    else if (!strcmp(argv[1],"boardinfo"))
    {
        athcfg_cat("/proc/version");
        athcfg_cat("/version");
        exit(0);
    }
    else if (!strcmp(argv[1],"cat"))
    {
        if (argc == 3)
            athcfg_cat(argv[2]);
        exit(0);
    }
    else if (!strcmp(argv[1],"wpssave"))
    {
        /*
         * This option is currently used by hostapd
         * once a wps transaction is complete and
         * wps_configured is set to 1
         */

        if (argc != 3) {
            fprintf(stderr,"%s: no wps conf file name given");
            exit(-1);
        }

        //fprintf(stdout, "%s: wpssave file name: %s\n", __func__, argv[2]);

        ret = athcfg_save_wps_cfg_to_nvram(argv[2], NULL, 0);

        exit(ret);
    }
    else
    {
        fprintf(stderr,"\n%s: invalid wlan options\n", argv[0]);
        usage(argv[0]);
    }
}

