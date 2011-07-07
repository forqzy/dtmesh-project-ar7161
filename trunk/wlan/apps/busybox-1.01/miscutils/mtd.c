/*
 * mtd - simple memory technology device manipulation tool
 *
 * Copyright (C) 2005 Waldemar Brodkorb <wbx@dass-it.de>,
 *	                  Felix Fietkau <nbd@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * $Id: mtd.c 7159 2007-05-10 08:54:37Z mbm $
 *
 * The code is based on the linux-mtd examples.
 */

#include <stdio.h>
#include <sys/types.h>
#include <ctype.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <linux/reboot.h>
#include <string.h>
#include <signal.h>
#include "busybox.h"

#include "mtd.h"

#define MAX_ARGS 8

#define DEBUG

static struct globals {
	unsigned int out_full, out_part, in_full, in_part;
} G;
/* We have to zero it out because of NOEXEC */
#define INIT_G() memset(&G, 0, sizeof(G))
int quiet;

static void mtd_output_status(int cur_signal)
{
	/* Deliberately using %u, not %d */
	fprintf(stderr, "%ul+%ul records in\n"
			"%ul+%ul records out\n",
			G.in_full, G.in_part,
			G.out_full, G.out_part);
}

static int mtd_open(const char *mtd, int flags)
{
	FILE *fp;
	char dev[PATH_MAX];
	int i;
	int ret;

	if ((fp = fopen("/proc/mtd", "r"))) {
		while (fgets(dev, sizeof(dev), fp)) {
			if (sscanf(dev, "mtd%d:", &i) && strstr(dev, mtd)) {
				snprintf(dev, sizeof(dev), "/dev/mtd/%d", i);
				if ((ret=open(dev, flags))<0) {
					snprintf(dev, sizeof(dev), "/dev/mtd%d", i);
					ret=open(dev, flags);
				}
				fclose(fp);
				return ret;
			}
		}
		fclose(fp);
	}

	return open(mtd, flags);
}

int set_fis_len(const char *mtd, unsigned int len)
{
	FILE *fp;
	char fis_mtd[PATH_MAX], buf[PATH_MAX];
    char mtd_label[16], *mtd_name, *fis, *fis_entry;
	int fd, i, ret;
    unsigned int old_len;
	struct mtd_info_user mtdInfo;
	struct erase_info_user mtdEraseInfo;
#define FIS_IMG_DESC_SZ 256
#define FIS_LEN_OFFSET 32

    mtd_name = strrchr(mtd,'/') + 1;

    if(mtd_name == 1)
        return -1;
    
	if ((fp = fopen("/proc/mtd", "r"))) {
		while (fgets(buf, sizeof(buf), fp)) {
            if(sscanf(buf, "mtd%d:", &i)) {
                if (strstr(buf, "FIS directory")) {
                    snprintf(fis_mtd, sizeof(fis_mtd), "/dev/mtd%d", i);
                }
                else if(strstr(buf, mtd_name)) {
                    *strrchr(buf,'\"') = '\0';
                    strncpy(mtd_label, strchr(buf,'"') + 1, 15);
                }
			}
		}
		fclose(fp);
	}

	fd = mtd_open(fis_mtd, O_RDWR | O_SYNC);
	if(fd < 0) {
		fprintf(stderr, "Could not open FIS directory mtd device: %s\n", fis_mtd);
		return fd;
	}

	if((ret = ioctl(fd, MEMGETINFO, &mtdInfo)) < 0) {
		fprintf(stderr, "Could not get MTD device info from %s\n", fis_mtd);
		close(fd);
		return ret;
	}

	fis = xmalloc(mtdInfo.erasesize);	
    if(fis == NULL) {
        ret = -1;
        goto error;
    }
    
    ret = read(fd, fis, mtdInfo.erasesize);
    
    if(ret != mtdInfo.erasesize)
        goto error;
    
    for(fis_entry = fis; isprint(*fis_entry); fis_entry += FIS_IMG_DESC_SZ) {
        ret = strcmp(mtd_label, fis_entry);
        if(!ret)
            break;
    }

    if(ret) {
		fprintf(stderr, "Could not find %s partition\n", mtd_label);
        ret = -1;
        goto error;
    } 

    old_len = *((unsigned int *) (fis_entry + FIS_LEN_OFFSET));
    
    if(old_len != len) {
        if (!quiet)
            fprintf(stderr, "Changing %s image descriptor length from %u to %u\n", 
                    mtd_label, old_len, len);

        *((unsigned int *) (fis_entry + FIS_LEN_OFFSET)) = len;
        
        mtdEraseInfo.start = 0;
        mtdEraseInfo.length = mtdInfo.erasesize;
        
        if ((ret = ioctl (fd, MEMERASE, &mtdEraseInfo)) < 0) {
            fprintf(stderr, "Erasing FIS directory failed\n");
            goto error;
        }
        
        /* reset file position from read above */
        if((ret = lseek(fd, 0, SEEK_SET)))
            goto error;

        if ((ret = write(fd, fis, mtdInfo.erasesize)) < mtdInfo.erasesize) {
            fprintf(stderr, "Writing FIS directory failed\n");
            goto error;
        }
    }
    else {
        if (!quiet)
            fprintf(stderr, "%s image descriptor length is unchanged\n", mtd_label);
    }
    
    ret = 0;
error:
    close(fd);
    if(fis)
        free(fis);
    return ret;
}

static int mtd_check(char *mtd)
{
	struct mtd_info_user mtdInfo;
	int fd;

	fd = mtd_open(mtd, O_RDWR | O_SYNC);
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", mtd);
		return 0;
	}

	if(ioctl(fd, MEMGETINFO, &mtdInfo)) {
		fprintf(stderr, "Could not get MTD device info from %s\n", mtd);
		close(fd);
		return 0;
	}

	close(fd);
	return 1;
}

static int mtd_unlock(const char *mtd)
{
	int fd;
	struct mtd_info_user mtdInfo;
	struct erase_info_user mtdLockInfo;

	fd = mtd_open(mtd, O_RDWR | O_SYNC);
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", mtd);
		exit(1);
	}

	if(ioctl(fd, MEMGETINFO, &mtdInfo)) {
		fprintf(stderr, "Could not get MTD device info from %s\n", mtd);
		close(fd);
		exit(1);
	}

	mtdLockInfo.start = 0;
	mtdLockInfo.length = mtdInfo.size;
	if(ioctl(fd, MEMUNLOCK, &mtdLockInfo)) {
		close(fd);
		return 0;
	}
		
	close(fd);
	return 0;
}

static int mtd_erase(const char *mtd)
{
	int fd;
	struct mtd_info_user mtdInfo;
	struct erase_info_user mtdEraseInfo;

	fd = mtd_open(mtd, O_RDWR | O_SYNC);
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", mtd);
		exit(1);
	}

	if(ioctl(fd, MEMGETINFO, &mtdInfo)) {
		fprintf(stderr, "Could not get MTD device info from %s\n", mtd);
		close(fd);
		exit(1);
	}

	mtdEraseInfo.length = mtdInfo.erasesize;

	for (mtdEraseInfo.start = 0;
		 mtdEraseInfo.start < mtdInfo.size;
		 mtdEraseInfo.start += mtdInfo.erasesize) {
		
		ioctl(fd, MEMUNLOCK, &mtdEraseInfo);
		if(ioctl(fd, MEMERASE, &mtdEraseInfo))
			fprintf(stderr, "Failed to erase block on %s at 0x%x\n", mtd, mtdEraseInfo.start);
		else
			G.out_full++;		
	}		

	close(fd);
	return 0;

}

static int mtd_write(int imagefd, const char *mtd, int update_fis)
{
	int fd, result;
	size_t r, w, e;
	struct mtd_info_user mtdInfo;
	struct erase_info_user mtdEraseInfo;
	char *buf;

	fd = mtd_open(mtd, O_RDWR | O_SYNC);
	if(fd < 0) {
		fprintf(stderr, "Could not open mtd device: %s\n", mtd);
		exit(1);
	}

	if(ioctl(fd, MEMGETINFO, &mtdInfo)) {
		fprintf(stderr, "Could not get MTD device info from %s\n", mtd);
		close(fd);
		exit(1);
	}
	buf = xmalloc(mtdInfo.erasesize);	
	r = w = e = 0;
	if (!quiet)
		fprintf(stderr, " [ ]");

	for (;;) {
		r = read(imagefd, buf, mtdInfo.erasesize);
		w += r;

		/* EOF */
		if (r <= 0) break;

		if ( r < mtdInfo.erasesize )
			G.in_part++;
		else
			G.in_full++;

		/* need to erase the next block before writing data to it */
		while (w > e) {
			mtdEraseInfo.start = e;
			mtdEraseInfo.length = mtdInfo.erasesize;

			if (!quiet)
				fprintf(stderr, "\b\b\b[e]");
			/* erase the chunk */
			if (ioctl (fd,MEMERASE,&mtdEraseInfo) < 0) {
				fprintf(stderr, "Erasing mtd failed: %s\n", mtd);
				break;
			}
			e += mtdInfo.erasesize;
		}
		
		if(w > e)
		     break;

		if (!quiet)
			fprintf(stderr, "\b\b\b[w]");
		
		if ((result = write(fd, buf, r)) < r) {
			if (result < 0) {
				fprintf(stderr, "Error writing image.\n");
				break;
			} else {
				fprintf(stderr, "Insufficient space.\n");
				break;
			}
		}
		if( r < mtdInfo.erasesize )
		        G.out_part++;
		else
			G.out_full++;
	}
	if (!quiet)
		fprintf(stderr, "\b\b\b\b");

	free(buf);
	close(fd);

    if(update_fis)
        set_fis_len(mtd, w);

	return 0;
}

int mtd_main (int argc, char **argv)
{
	int ch, i, boot, imagefd=0, unlocked, update_fis;
	char *erase[MAX_ARGS], *device, *imagefile=NULL;
	struct sigaction new_action;
	enum {
		CMD_ERASE,
		CMD_WRITE,
		CMD_UNLOCK
	} cmd;
	
	erase[0] = NULL;
	boot = 0;
	quiet = 0;
    update_fis = 0;

	while ((ch = getopt(argc, argv, "rqle:")) != -1)
		switch (ch) {
			case 'r':
				boot = 1;
				break;
			case 'q':
				quiet++;
				break;
			case 'l':
				update_fis++;
				break;
			case 'e':
				i = 0;
				while ((erase[i] != NULL) && ((i + 1) < MAX_ARGS))
					i++;
					
				erase[i++] = optarg;
				erase[i] = NULL;
				break;
			
			case '?':
			default:
				bb_show_usage();
		}
	argc -= optind;
	argv += optind;
	
	if (argc < 2)
		bb_show_usage();

	if ((strcmp(argv[0], "unlock") == 0) && (argc == 2)) {
		cmd = CMD_UNLOCK;
		device = argv[1];
	} else if ((strcmp(argv[0], "erase") == 0) && (argc == 2)) {
		cmd = CMD_ERASE;
		device = argv[1];
	} else if ((strcmp(argv[0], "write") == 0) && (argc == 3)) {
		cmd = CMD_WRITE;
		device = argv[2];
	
		if (strcmp(argv[1], "-") == 0) {
			imagefile = (char *) "<stdin>";
			imagefd = 0;
		} else {
			imagefile = argv[1];
			if ((imagefd = open(argv[1], O_RDONLY)) < 0) {
				fprintf(stderr, "Couldn't open image file: %s!\n", imagefile);
				exit(1);
			}
		}
	
		if (!mtd_check(device)) {
			fprintf(stderr, "Can't open device for writing!\n");
			exit(1);
		}
	} else {
		bb_show_usage();
	}

	sync();
	INIT_G();
	i = 0;
	unlocked = 0;

	/* Set up SIGUSR1 handler */
	sigemptyset (&new_action.sa_mask);
	new_action.sa_flags = 0;
	new_action.sa_handler = mtd_output_status;
	if (sigaction (SIGHUP, &new_action, NULL) != 0 || sigaction (SIGUSR1, &new_action, NULL) != 0 )
		exit(EXIT_FAILURE);

	while (erase[i] != NULL) {
		if (quiet < 2)
			fprintf(stderr, "Unlocking %s ...\n", erase[i]);
		mtd_unlock(erase[i]);
		if (quiet < 2)
			fprintf(stderr, "Erasing %s ...\n", erase[i]);
		mtd_erase(erase[i]);
		if (strcmp(erase[i], device) == 0)
			unlocked = 1;
		i++;
	}
	
	if (!unlocked) {
		if (quiet < 2) 
			fprintf(stderr, "Unlocking %s ...\n", device);
		mtd_unlock(device);
	}
		
	switch (cmd) {
		case CMD_UNLOCK:
			break;
		case CMD_ERASE:
			if (quiet < 2)
				fprintf(stderr, "Erasing %s ...\n", device);
			mtd_erase(device);
			break;
		case CMD_WRITE:
			if (quiet < 2)
				fprintf(stderr, "Writing from %s to %s ...\n", imagefile, device);
			mtd_write(imagefd, device, update_fis);
			if (quiet < 2)
				fprintf(stderr, "\n");
			break;
	}

	sync();
	
	if (boot) {
		fprintf(stderr, "Rebooting ...\n");
		fflush(stderr);
		syscall(SYS_reboot,LINUX_REBOOT_MAGIC1,LINUX_REBOOT_MAGIC2,LINUX_REBOOT_CMD_RESTART,NULL);
	}
	mtd_output_status(0);
	return 0;
}
