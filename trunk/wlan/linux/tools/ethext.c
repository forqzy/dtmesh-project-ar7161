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
#include <net/if.h>

#include "athrs_ioctl.h"

#define ETH_ALEN        6
#define ETH_FORMATTED_MAC_LEN   17

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

int ethlanctl_do_ioctl(ETHCFGPARAMS *eqcfg)
{
        struct ifreq ifr;
        char *ifname = "eth0";
        int s;

        s = socket(AF_INET, SOCK_DGRAM, 0);
        if (s < 0)
               return ETH_LANCTLEXT_ERROR_1;
        strcpy(ifr.ifr_name, ifname);
        ifr.ifr_data = (void *)eqcfg;
        if (ioctl(s,eqcfg->cmd, &ifr) < 0)
                return ETH_LANCTLEXT_ERROR_2;
        /* close (s)*/
        return ETH_LANCTLEXT_NOERROR;
}

int ethlanctl_do_brioct(unsigned long cmd,unsigned long arg1, unsigned long arg2,unsigned long arg3)
{
    char *brname="br0";
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
    ret = br_device_ioctl(br,cmd,arg1,arg2,arg3);

bradd_cleanup:
    ath_br_deinit();
    return ret;
}


int  ethlanctl_setmaxlimit(uint32_t portnum,uint32_t limit)
{
	//printf("SET MACLIMIT %d,%d\n",portnum,limit);
        if (portnum < 0 || portnum > ETH_MAX_LANPORTS){
                       printf("invalid port num argument\n");
                       return ETH_LANCTLEXT_ERROR_2;
        }
	//return ethlanctl_do_brioct(BRCTL_SET_MAC_LIMIT,portnum,limit,0);
	return ethlanctl_do_brioct(BRCTL_SET_ATHR_PHY_PORT_MAC_ADDR_LIMIT,
                        portnum,limit,0);
}

int  ethlanctl_getmaxlimit(uint32_t portnum,uint32_t *limit)
{
	//printf("GET MACLIMIT %d\n",portnum);
        if (portnum < 0 || portnum > ETH_MAX_LANPORTS){
                       printf("invalid port num argument\n");
                       return ETH_LANCTLEXT_ERROR_2;
        }
	//return ethlanctl_do_brioct(BRCTL_GET_MAC_LIMIT,portnum,(unsigned long)limit,0);
	return ethlanctl_do_brioct(BRCTL_GET_ATHR_PHY_PORT_MAC_ADDR_LIMIT,
                                   portnum,(unsigned long)limit,0);
}

int  ethlanctl_setaging(char *macaddr,uint32_t aging)
{
	//printf("SET AGING %s,%d\n",macaddr,aging);
        //return ethlanctl_do_brioct(BRCTL_SET_MAC_AGING,(unsigned long)macaddr,aging,0);
        return ethlanctl_do_brioct(BRCTL_SET_ATHR_PHY_PORT_MAC_AGEING_INTERVAL,
                                   (unsigned long)macaddr,aging,0);
}

int ethlanctl_getaging(char *macaddr, uint32_t *aging)
{
	//printf("GET AGING %s\n",macaddr);
        //return ethlanctl_do_brioct(BRCTL_GET_MAC_AGING,(unsigned long)macaddr,(unsigned long)aging,0);
        return ethlanctl_do_brioct(BRCTL_GET_ATHR_PHY_PORT_MAC_AGEING_INTERVAL,
                                   (unsigned long)macaddr,(unsigned long)aging,0);
}


int ethlanctl_setstormctl(uint32_t portnum,uint32_t enabled,ETH_STORMTYPE type,uint32_t rate)
{
	ETHCFGPARAMS eqcfg;
	ETHCFG_STOMCTL *pstomctl;
	int err;

	printf("set stom %d,%d,%d,%d\n",portnum,enabled,type,rate);
        if (portnum <= 0 || portnum > ETH_MAX_LANPORTS){
                       printf("invalid port num argument\n");
                       return ETH_LANCTLEXT_ERROR_2;
        }
	eqcfg.cmd=ETH_STORMCTL_CFG;
	pstomctl=&eqcfg.cmdparams.stomctl;
	pstomctl->portnum=portnum;
	pstomctl->enabled=enabled;
	pstomctl->storm_type=type;
	pstomctl->storm_rate=(rate * 1000);
	err=ethlanctl_do_ioctl(&eqcfg);
	printf("SET STORMCRL Status [%x]\n",err);
	return err;
}

int ethlanctl_getstormctl(uint32_t portnum,ETHCFG_PORTSTOMCTL *portstomctl)
{
        ETHCFGPARAMS eqcfg;
	ETHCFG_PORTSTOMCTL *pstomctl;
        int err;

        if (portnum <= 0 || portnum > ETH_MAX_LANPORTS){
              printf("invalid port num argument\n");
              return ETH_LANCTLEXT_ERROR_2;
        }
        eqcfg.cmd=ETH_STORMCTL_SHOW;
        pstomctl=&eqcfg.cmdparams.portstomctl;
        pstomctl->portnum=portnum;
        err=ethlanctl_do_ioctl(&eqcfg);
       	printf("GET STORMCRL Status [%x] \n",err);
	if ( err ==  ETH_LANCTLEXT_NOERROR){
        	printf(" Broadcast enabled:%d rate:%d\n",pstomctl->broadcast.enabled,
				(pstomctl->broadcast.storm_rate / 1000));
        	printf(" Multicast enabled:%d rate:%d\n",pstomctl->multicast.enabled,
				(pstomctl->multicast.storm_rate / 1000));
        	printf(" Unicast enabled:%d rate:%d\n",pstomctl->unicast.enabled,
				(pstomctl->unicast.storm_rate / 1000));
	}
        return err;
}

int ethlanctl_process_setmaxlimit(int argc,char *argv[])
{
	int portnum,limit;
       if (argc < 5){
                printf("invalid commandline argument\n");
                return ETH_LANCTLEXT_ERROR_4;
        }
        if(sscanf(argv[3], "%i", &(portnum)) != 1){
                printf("invalid port num argument\n");
                return ETH_LANCTLEXT_ERROR_6;
        }
        if(sscanf(argv[4], "%d", &(limit)) != 1){
                printf("invalid port num argument\n");
                return ETH_LANCTLEXT_ERROR_6;
        }
        if (ethlanctl_setmaxlimit(portnum,limit) != ETH_LANCTLEXT_NOERROR){
                printf("ioctl error\n");
                return ETH_LANCTLEXT_ERROR_3;
	}
	return ETH_LANCTLEXT_NOERROR;
}


int ethlanctl_process_getmaxlimit(int argc,char *argv[])
{
       int portnum=0,limit=0;

        if (argc < 4){
                printf("invalid commandline argument\n");
                return ETH_LANCTLEXT_ERROR_4;
        }

        if(sscanf(argv[3], "%i", &(portnum)) != 1){
                printf("invalid port num argument\n");
                return ETH_LANCTLEXT_ERROR_6;
        }

        if (portnum <= 0 || portnum > ETH_MAX_LANPORTS){
              printf("invalid port num argument\n");
              return ETH_LANCTLEXT_ERROR_2;
        }

        if (ethlanctl_getmaxlimit(portnum, &limit) != ETH_LANCTLEXT_NOERROR){
                printf("ioctl error\n");
                return ETH_LANCTLEXT_ERROR_3;
        }

        printf("%s: getmaxlimit = %d\n", __func__, limit);

        return ETH_LANCTLEXT_NOERROR;
}

int eth_verify_and_get_mac_address(char *mac_fmtted, char *mac_addr)
{
        int i=0, j=0;

        if (strlen(mac_fmtted) != ETH_FORMATTED_MAC_LEN)  {
                printf("Invalid mac address length\n");
                return -1;
        }

        for(j=0; j < ETH_ALEN; j++) {
                if (sscanf(mac_fmtted + i, "%hhx", mac_addr + j) != 1) {
                        printf("Invalid mac address format\n");
                        return -1;
                }
                i = i+3;
        }

        return 0;
}

int ethlanctl_process_setmaxaging(int argc ,char *argv[])
{
       int aging;
       char mac_addr[ETH_ALEN];
       char *mac_ptr;

        if (argc < 5){
                printf("invalid commandline argument\n");
                return ETH_LANCTLEXT_ERROR_4;
        }

        if(sscanf(argv[4], "%d", &(aging)) != 1){
                printf("invalid port num argument\n");
                return ETH_LANCTLEXT_ERROR_6;
        }


        if (eth_verify_and_get_mac_address(argv[3], mac_addr) != 0) {
                return ETH_LANCTLEXT_ERROR_4;
        }
#if 0
        mac_ptr = argv[3];

        sscanf(mac_ptr,"%hhx",&(mac_addr[0]));
        sscanf(mac_ptr + 3,"%hhx",&(mac_addr[1]));
        sscanf(mac_ptr + 6,"%hhx",&(mac_addr[2]));
        sscanf(mac_ptr + 9,"%hhx",&(mac_addr[3]));
        sscanf(mac_ptr + 12,"%hhx",&(mac_addr[4]));
        sscanf(mac_ptr + 15,"%hhx",&(mac_addr[5]));
#endif
        if (ethlanctl_setaging(mac_addr, aging) != ETH_LANCTLEXT_NOERROR){
                printf("mac addr %s not found\n", argv[3]);
                return ETH_LANCTLEXT_ERROR_3;
        }

        printf("mac %s set ageing to %d seconds success \n", argv[3], aging);

        return ETH_LANCTLEXT_NOERROR;
}

int ethlanctl_process_getmaxaging(int argc,char *argv[])
{
        int aging;
        char mac_addr[ETH_ALEN];
        const char *mac_ptr;

        if (argc < 4){
                printf("invalid commandline argument\n");
                return ETH_LANCTLEXT_ERROR_4;
        }

        if (eth_verify_and_get_mac_address(argv[3], mac_addr) != 0) {
                return ETH_LANCTLEXT_ERROR_4;
        }
#if 0

        mac_ptr = (const char *)argv[3];

        sscanf(mac_ptr,"%hhx",&(mac_addr[0]));
        sscanf(mac_ptr + 3,"%hhx",&(mac_addr[1]));
        sscanf(mac_ptr + 6,"%hhx",&(mac_addr[2]));
        sscanf(mac_ptr + 9,"%hhx",&(mac_addr[3]));
        sscanf(mac_ptr + 12,"%hhx",&(mac_addr[4]));
        sscanf(mac_ptr + 15,"%hhx",&(mac_addr[5]));
#endif
        if (ethlanctl_getaging(mac_addr, &aging) != ETH_LANCTLEXT_NOERROR){
                printf("mac addr %s not found\n", argv[3]);
                return ETH_LANCTLEXT_ERROR_3;
        }

        printf("mac %s ageing = %d \n", argv[3], aging);

        return ETH_LANCTLEXT_NOERROR;
}

static char *athr_format_mac_addr(unsigned char *mac, char *mac_fmt)
{
        unsigned char *ptr;
        int i, cnt;

        ptr = mac_fmt;

        for (i = 0; i < ETH_ALEN; i++)  {
                cnt = sprintf(ptr, "%02x%c", mac[i], i == ETH_ALEN - 1 ? ' ':':');
                ptr += cnt;
        }

        *(ptr + 1)='\0';

        return mac_fmt;

}

int ethlanctl_process_getportmactable(int argc,char *argv[])
{
        int portnum,limit, ret, i = 0;
        ETHCFG_MACAGING *mac_table, *ptr;
        char mac_fmt[30];

        if (argc < 4){
                printf("invalid commandline argument\n");
                return ETH_LANCTLEXT_ERROR_4;
        }

        if(sscanf(argv[3], "%i", &(portnum)) != 1){
                printf("invalid port num argument\n");
                return ETH_LANCTLEXT_ERROR_6;
        }

        if (portnum <= 0 || portnum > ETH_MAX_LANPORTS){
              printf("invalid port num argument\n");
              return ETH_LANCTLEXT_ERROR_2;
        }

        if (ethlanctl_getmaxlimit(portnum, &limit) != ETH_LANCTLEXT_NOERROR){
                printf("ioctl error\n");
                return ETH_LANCTLEXT_ERROR_3;
        }


        mac_table = (ETHCFG_MACAGING *) calloc (limit, sizeof(ETHCFG_MACAGING));


        if (!mac_table) {
                perror("malloc");
                return ETH_LANCTLEXT_ERROR_3;
        }

        ret = ethlanctl_do_brioct(BRCTL_GET_ATHR_PHY_PORT_MAC_TABLE, portnum,
                                         (unsigned long)mac_table, 0);

        if (ret < 0) {
                printf("%s: get port mac table ioctl error\n",__func__);
                free(mac_table);
                return ETH_LANCTLEXT_ERROR_3;
        }

        ptr = mac_table;

        printf("\n");
        printf("Mac Table entries for port:%d Max Entries:%d\n", portnum, limit);
        printf("\n");
        printf("--------------------------------------------\n");
        printf("Mac Address         MaxAge        CurrentAge\n");
        printf("--------------------------------------------\n");


        if (ret == 0) {
                printf("\n*** Mac port table for port %d is empty *** \n\n", portnum);
        }
        else
        {
                for(i=0; i < ret; i++) {
                        printf("%s  %d(secs)      %d(secs)\n",
                               athr_format_mac_addr(mac_table->mac_addr, mac_fmt),
                               mac_table->maxagingtime, mac_table->curagingtime);

                        mac_table ++;
                }
                printf("\n\n");
        }

        free(ptr);

	return ETH_LANCTLEXT_NOERROR;
}

int ethlanctl_process_setstorm(int argc,char *argv[])
{
        char *type_options[ETH_LANCTLEXT_STROM_TYPE_MAX]={"broadcast","multicast","unicast"};
        char *enable_options[ETH_LANCTLEXT_STROM_ENABLE_MAX]={"off","on"};
        int portnum,type,enabled,rate;

       if (argc < 7){
                printf("invalid commandline argument\n");
                return ETH_LANCTLEXT_ERROR_4;
        }
        for (type=0;type<ETH_LANCTLEXT_STROM_TYPE_MAX;type++){
                if (!strcmp(argv[3],type_options[type])) {
                        break;
                }
        }
        if (type == ETH_LANCTLEXT_STROM_TYPE_MAX){
                printf("invalid option\n");
                return ETH_LANCTLEXT_ERROR_4;
       }

       for (enabled=0;enabled<ETH_LANCTLEXT_STROM_ENABLE_MAX;enabled++){
                if (!strcmp(argv[4],enable_options[enabled])) {
                        break;
                }
        }
        if ( enabled == ETH_LANCTLEXT_STROM_ENABLE_MAX){
                printf("invalid on/off\n");
                return ETH_LANCTLEXT_ERROR_5;
        }
        if(sscanf(argv[5], "%i", &(portnum)) != 1){
                printf("invalid port num argument\n");
                return ETH_LANCTLEXT_ERROR_6;
        }
        if(sscanf(argv[6], "%d", &(rate)) != 1){
                printf("invalid rate\n");
                return ETH_LANCTLEXT_ERROR_6;
        }
        if (ethlanctl_setstormctl(portnum,enabled,type,rate) != ETH_LANCTLEXT_NOERROR){
                printf("ioctl error\n");
                return ETH_LANCTLEXT_ERROR_3;
        }

   return ETH_LANCTLEXT_NOERROR;
}

int ethlanctl_process_getstorm(int argc,char *argv[])
{
       unsigned int portnum;
       ETHCFG_PORTSTOMCTL portstomctl;

       if (argc < 4){
                printf("invalid commandline argument\n");
                return ETH_LANCTLEXT_ERROR_4;
        }
        /* Get the value */
        if(sscanf(argv[3], "%i", &(portnum)) != 1){
                        printf("invalid port num argument\n");
                        return ETH_LANCTLEXT_ERROR_1;
        }
        if (ethlanctl_getstormctl(portnum,&portstomctl) != ETH_LANCTLEXT_NOERROR){
                        printf("ioctl error\n");
                        return ETH_LANCTLEXT_ERROR_3;
        }
     	return ETH_LANCTLEXT_NOERROR;
}

void eth_ext_usage(void)
{
    char us_buf[] = {
                        "\nUsage:  cfg lanctl set-maclimit <portno> <limit>\n"
                        "\t    get-maclimit <portno> <limit>\n"
                        "\t    get-macaging <mac-address>\n"
                        "\t    set-macaging <mac-address> <ageing interval>\n"
                        "\t    get-portmactable <portno>\n"
    };

    fprintf(stderr,"%s\n",us_buf);
    exit(-1);

}

int ethlanctl_process_opt(int argc,char *argv[])
{
	int type=0;
        char *cmdargs[7]={"set-maclimit","get-maclimit","set-macaging","get-macaging","get-portmactable","set-storm","get-storm"};
	ETHLANCMDPROC cmdfunc[ETH_LANCTLEXT_MAX_CMDS]={ethlanctl_process_setmaxlimit,
		ethlanctl_process_getmaxlimit,ethlanctl_process_setmaxaging,ethlanctl_process_getmaxaging,
		ethlanctl_process_getportmactable,ethlanctl_process_setstorm,ethlanctl_process_getstorm};

        if (argc > 3) {
                for (type=0;type < ETH_LANCTLEXT_MAX_CMDS; type++){
                        if (!strcmp(argv[2],cmdargs[type])) {
                                return cmdfunc[type](argc,argv);
                        }
                }
        }
	printf("Invalid LANCTL option \n");
        eth_ext_usage();
	return ETH_LANCTLEXT_ERROR_3;
}
