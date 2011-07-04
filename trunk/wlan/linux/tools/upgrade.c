/*
  upgrade.c
  
  Verify imgfile with md5sum and start embedded upload script. 
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/utsname.h>
#include <sys/unistd.h>

char errorpage[BUFSIZ];
char nextpage[BUFSIZ];
char imgfile[BUFSIZ];
#define LOGFILE "/tmp/upgrade.log"

/* Show the error page, with macro MESSAGE. */
void error_page(char *Message) {
    FILE *fp;
    char line[BUFSIZ];
    char buf[BUFSIZ];
    char *p1;
    
    printf("HTTP/1.0 200 OK\r\n");
    printf("Content-type: text/html\r\n\r\n");
    
    sprintf(buf,"/usr/www/%s", errorpage);

    fp = fopen(buf,"r");
    if (fp == NULL) {
	fprintf(stdout,"<html>\n<body>\r\n",Message);
	if (*errorpage != '\0') {
	    fprintf(stdout,"Error: could not open the errorpage: %s<p>\r\n",errorpage);
	    fprintf(stdout,"The original error message was:<br>\r\n");
	}
	fprintf(stdout,"%s\r\n",Message);
	fprintf(stdout,"</body>\r\n</html>\r\n",Message);
	exit(0);
    }
    
    while (fgets(line,BUFSIZ,fp) != NULL) {
	p1 = line;
	while (*p1 != '\0') {
	    if (strncasecmp(p1,"<insert message>",15) == 0) {
		*p1 = '\0';
		sprintf(buf,"%s%s%s",line,Message,p1 + 16);
		strcpy(line,buf);
	    }
	    p1++;
	}
	fprintf(stdout,"%s",line);
    }
    
    exit(EXIT_SUCCESS);
}

/* Write a line to the logging file. */
void log_write(char *line)
{
    FILE *fp;
    
    fp = fopen(LOGFILE, "a");
    if (fp == NULL)
	error_page("Could not open the logfile");
    
    fprintf(fp,"%s\n", line);
    fclose(fp);
}


void unencode(char * src, char* dest)
{
    int code;
    
    while(*src != '\0') {
        if(*src == '+') {
            *dest = ' ';
        }
        else if(*src == '%') {
            if(sscanf(src + 1, "%2x", &code) != 1) 
		code = '?';
	    *dest = code;
	    src += 2;
	}
	else
	    *dest = *src;

	dest++; src++;
    }
    *dest = '\0';
}

static const long KEY_ID = 0x55504453;	/* "UPDS" */

int main(int argc, char *argv[])
{
    char *envp;           /* pointer to environment variable. */
    long len, bytes_read;             /* the number of incoming bytes. */
    char content[BUFSIZ];
    char errmsg[BUFSIZ];
    
    FILE *fp;
    char *p1, *p2, *p3, *last;
    char buf[BUFSIZ], buf2[BUFSIZ];
    int blksz, count, code, i, refresh, fd, maxfd;
    char img_arch[_UTSNAME_LENGTH], img_md5sum[33], calc_md5sum[33];
    pid_t pid, sid;
    struct utsname uts_name;
    struct stat imgstat;
    int semid = -1;
    struct sembuf sops = { 0, 0, 0 }; 

    unlink(LOGFILE); /* start with a new file */
    log_write("Upgrade Log Start");

    envp = getenv("REQUEST_METHOD");
    if ((envp == NULL) || (*envp == '\0') || (strcasecmp(envp,"post") != 0)) {
	log_write("this program only supports METHOD=POST.");
	error_page("this program only supports METHOD=POST.");
    }
  
    /* Get the total number of bytes in the input stream from the
       CONTENT_LENGTH environment variable. */
    envp = getenv("CONTENT_LENGTH");
    if (envp == NULL) {
	log_write("Error: no CONTENT_LENGTH found.");
	error_page("Error: no CONTENT_LENGTH found.");
    }
    
    if (sscanf(envp, "%ld", &len) != 1 || (len > BUFSIZ)) {
	log_write("Error: parsing CONTENT_LENGTH failed.");
	error_page("Error: parsing CONTENT_LENGTH failed.");
    }
  
    bytes_read = fread(content, 1, len, stdin);
    if(bytes_read < len) {
	sprintf(errmsg,"Warning: POST content size [%u] does not equal CONTENT_LENGTH [%u]",
		(unsigned) bytes_read, (unsigned) len);
	log_write(errmsg);
	len = bytes_read;
    }

    last = content + len;
    
    p1 = content;

    /* get form data */
    nextpage[0] = '\0';
    errorpage[0] = '\0';
    imgfile[0] = '\0';
    refresh = 10;
    p2 = p1;
    while(p1 <= last) {
	if(*p1 == '?' || *p1 == '&' || *p1 == '\n' || *p1 == '\r' || *p1 == '\0') {
	    *p1 = '\0';
	    p3 = strchr(p2, '=');
	    if(p3) {
		*p3 = '\0';
		unencode(p2, buf2);
		p3++;
		unencode(p3, buf);
		if(!strcmp(buf2, "imgfile"))
		    strcpy(imgfile, buf);
		if(!strcmp(buf2, "errorpage"))
		    strcpy(errorpage, buf);	
		if(!strcmp(buf2, "nextpage"))
		    strcpy(nextpage, buf);	
		if(!strcmp(buf2, "refresh"))
		    refresh = strtoul(buf, NULL, 10);	
	    }
	    p2 = p1 + 1;
	}
	p1++;
    }

    if(nextpage[0] == '\0')
	log_write("Warning: post contents did not include a next page");
    
    if(errorpage[0] == '\0')
	log_write("Warning: post contents did not include an error page");
    
    if(imgfile[0] == '\0') {
	log_write("Error: post contents did not include an image file name");
	error_page("Error: post contents did not include an image file name");
    }

    log_write("Info: retrieving MD5 sum from image file");
    fp = fopen(imgfile,"r");
    
    if(fp == NULL) {
	log_write("Error: failed to open image file");
	error_page("Error: failed to open image file");
    }

    count = 0; blksz = 0; img_md5sum[0] = '\0'; img_arch[0] = '\0';
    /* image parameters within the first 20 lines */
    for (i = 0; i < 20; i++) {
	if (fgets(buf, BUFSIZ, fp) == NULL)
	    break;
	if(blksz == 0) {
	    if(sscanf(buf,"BLKSZ=%d", &blksz) == 1)
		count++;
	}
	if(img_md5sum[0] == '\0') {
	    if(sscanf(buf,"IMG_MD5SUM=\"%32[a-fA-F0-9]\"", img_md5sum) == 1)
		count++;
	}
	if(img_arch[0] == '\0') {
	    if(sscanf(buf,"IMG_ARCH=%s", img_arch) == 1)
		count++;
	}
	if(count == 3)
	    break;
    }

    fclose(fp);

    if(count != 3) {
	log_write("Error: failed to get MD5 sum from image file");
	error_page("Error: failed to get MD5 sum from image file");
    }

    log_write("Info: get machine architecture");
    if (uname(&uts_name) == -1) {
	log_write("cannot get system name");
	error_page("cannot get system name");
    }

    if(strncmp(img_arch, uts_name.machine, _UTSNAME_LENGTH)) {
	sprintf(buf, "The uploaded %s image is not for this %s platform", img_arch, uts_name.machine);
	log_write(buf);
	error_page(buf);
    }
    
    log_write("Info: calculating MD5 sum on image file");
    sprintf(buf, "dd bs=%d if=%s skip=1 | gunzip -c | md5sum", blksz, imgfile);
    fp = popen(buf, "r");

    strcpy(errmsg, "Error: cannot calculate MD5 sum");
    
    if(fp == NULL) {
	log_write(errmsg);
	error_page(errmsg);
    }
    
    if(fgets(buf, BUFSIZ, fp) == NULL) {
	pclose(fp);
	log_write(errmsg);
	error_page(errmsg);
    }
    
    pclose(fp);
    
    if(sscanf(buf, "%32[a-fA-F0-9]", &calc_md5sum) != 1) {
	log_write(errmsg);
	error_page(errmsg);
    }
    
    if(strncmp(img_md5sum, calc_md5sum, 32)) {
	log_write("Error: verify failed");
	error_page("Error: verify failed");
    }

    pid = getpid();
    sprintf(buf,"Info: current PID is %d", pid);
    log_write(buf);
    
    /* initialize semaphore to block */
    if ((semid = semget(KEY_ID, 1, IPC_CREAT | 0666)) == -1) {
	log_write("Cannot open semaphore");
	error_page("Cannot open semaphore");
    }

    semctl(semid, 0, SETVAL, 1);
    
    if(stat(imgfile, &imgstat) < 0) {
	sprintf(buf, "ERROR: cannot stat %s", imgfile);
	log_write(buf);
	error_page(buf);
    }

    pid = fork();

    if(pid < 0) {
	semctl(semid, 0, IPC_RMID); /* done with semaphore */
	log_write("Error: cannot create child process");
	error_page("Error: cannot create child process");
    }
    
    if(pid > 0) {
	semop(semid, &sops, 1); /* wait for child */
	semctl(semid, 0, IPC_RMID); /* done with semaphore */
    
	log_write("Info: Returning to httpd server");
	/* parent process: return control to httpd server */
	/* redirect after timeout to upgrade page in the image. */
	fprintf(stdout, "HTTP/1.0 200 OK\r\n");
	fprintf(stdout, "Content-type: text/html\r\n");
	fprintf(stdout, "Connection: close\r\n");
	fprintf(stdout, "Refresh: %d; url=/cgi-bin/update.cgi\r\n", refresh);
	fprintf(stdout, "\r\n");
	fprintf(stdout, "\r\n");

	if(nextpage[0] != '\0') {
	    sprintf(buf,"/usr/www/%s", nextpage);
	    fp = fopen(buf, "r");
	    if (fp != NULL) {
		while (fgets(buf, BUFSIZ, fp) != NULL)
		    fprintf(stdout, "%s", buf);
	    }
	    fclose(fp);
	}
	
	exit(EXIT_SUCCESS);
    }

   
    semctl(semid, 0, SETVAL, 0);
    /* Change the file mode mask */
    umask(0);
 
    /* Create a new SID for the child process */
    sid = setsid();
    if (sid < 0) {
	log_write("unable to create a new session");
	exit(EXIT_FAILURE);
    }
    
    /* Change the current working directory.  This prevents the current
       directory from being locked; hence not being able to remove it. */
    if ((chdir("/")) < 0) {
	log_write("unable to change to root directory");
	exit(EXIT_FAILURE);
    }
    
    /* Redirect standard files to /dev/console */
    freopen( "/dev/console", "r", stdin);
    freopen( "/dev/console", "w", stdout);
    freopen( "/dev/console", "w", stderr);

    /* close the other inherited descriptors */
    maxfd=sysconf(_SC_OPEN_MAX);
    for(fd = 3; fd < maxfd; fd++)
	close(fd);

    sprintf(buf, "Info: Starting %s", imgfile); 
    log_write(buf);

    /* let the parent close stdout and allow httpd to proceed */
    semctl(semid, 0, SETVAL, 0);

    /* child process: start upgrade image */
    chmod(imgfile, imgstat.st_mode | S_IXUSR);
    execl(imgfile, imgfile, (char *) NULL);
    /* should not get here */
    exit(EXIT_SUCCESS);
}
