#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>
#include <unistd.h>


#define BUF_SIZE 1024

int main(int argc,char **argv)
{
    char   buf[BUF_SIZE] = {0};
    FILE   *fp           = NULL;
    int    rs            = 0;
    
    fprintf(stdout,"Pragma: no-cache\n");
    fprintf(stdout,"Cache-control: no-cache\n");
    fprintf(stdout,"Content-Transfer-Encoding: binary\n");
    fprintf(stdout,"Content-Disposition: attachment; filename=\"atheros-cfg.bin\"");
    fprintf(stdout,"Content-type: application/octet-stream\n\n");

    if(NULL != (fp = fopen("/tmp/.apcfg","rb")))
    {
		while(!feof(fp))
        {
            rs = fread(buf,1,sizeof(buf),fp);
	    	fwrite(buf,rs,1,stdout);
		}
		fclose(fp);
    }
    else
    {
        printf("open /tmp/.apcfg failed!\n");
    }
        
    return 0;
}



