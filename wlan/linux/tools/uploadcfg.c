#include <stdio.h>
#include <stdlib.h>

char errorpage[BUFSIZ];
char cfgfile[BUFSIZ];
#define LOGFILE "/tmp/uploadcfg.log"

/* Show the error page, with macro MESSAGE. */
void error_page(char *Message) 
{
    FILE *fp;
    char line[BUFSIZ];
    char buf[BUFSIZ];
    char *p1;
    
    printf("HTTP/1.0 200 OK\r\n");
    printf("Content-type: text/html\r\n\r\n");
    
    sprintf(buf,"/usr/www/%s", errorpage);

    fp = fopen(buf,"rb");
    if (fp == NULL) 
    {
		fprintf(stdout,"<html>\n<body>\r\n",Message);
		if (*errorpage != '\0') 
		{
	    	fprintf(stdout,"Error: could not open the errorpage: %s<p>\r\n",errorpage);
	    	fprintf(stdout,"The original error message was:<br>\r\n");
		}
		fprintf(stdout,"%s\r\n",Message);
		fprintf(stdout,"</body>\r\n</html>\r\n",Message);
		exit(0);
    }
    
    while (fgets(line,BUFSIZ,fp) != NULL) 
    {
		p1 = line;
		while (*p1 != '\0') 
		{
	    	if (strncasecmp(p1,"<insert message>",15) == 0) 
	    	{
				*p1 = '\0';
				sprintf(buf,"%s%s%s",line,Message,p1 + 16);
				strcpy(line,buf);
	    	}
	    	p1++;
		}
		fprintf(stdout,"%s",line);
    }
    
    fclose(fp);
    exit(EXIT_SUCCESS);
}

/* Write a line to the logging file. */
void log_write(char *line)
{
    FILE *fp;
    
    fp = fopen(LOGFILE, "a");
    if (fp == NULL)
    {
		printf("Could not open the logfile");
    }
    
    fprintf(fp,"%s\n", line);
    fclose(fp);
}


void unencode(char * src, char* dest)
{
    int code;
    
    while(*src != '\0') 
    {
        if(*src == '+') 
        {
            *dest = ' ';
        }
        else if(*src == '%') 
        {
            if(sscanf(src + 1, "%2x", &code) != 1)
            { 
				code = '?';
	    	}
	    	*dest = code;
	    	src += 2;
		}
		else
		{
	    	*dest = *src;
		}
		
		dest++; 
		src++;
    }
    
    *dest = '\0';
}


int main(int argc, char *argv[])
{
    char *envp;           		   /* pointer to environment variable. */
    long len, bytes_read;             /* the number of incoming bytes. */
    char content[BUFSIZ];
    char errmsg[BUFSIZ];
    
    FILE *fp,*f;
    char *p1, *p2, *p3, *last;
    char buf[BUFSIZ], buf2[BUFSIZ];
    int  refresh;
    char cmd[4096];
    
    unlink(LOGFILE); /* start with a new file */
    log_write("Upgrade Log Start");

    envp = getenv("REQUEST_METHOD");
    if ((envp == NULL) || (*envp == '\0') || (strcasecmp(envp,"post") != 0)) 
    {
		log_write("this program only supports METHOD=POST.");
		error_page("this program only supports METHOD=POST.");
    }
  
    /* Get the total number of bytes in the input stream from the
       CONTENT_LENGTH environment variable. */
    envp = getenv("CONTENT_LENGTH");
    if (envp == NULL) 
    {
		log_write("Error: no CONTENT_LENGTH found.");
		error_page("Error: no CONTENT_LENGTH found.");
    }
    
    if (sscanf(envp, "%ld", &len) != 1 || (len > BUFSIZ)) 
    {
		log_write("Error: parsing CONTENT_LENGTH failed.");
		error_page("Error: parsing CONTENT_LENGTH failed.");
    }
  
    bytes_read = fread(content, 1, len, stdin);
    if(bytes_read < len) 
    {
		sprintf(errmsg,"Warning: POST content size [%u] does not equal CONTENT_LENGTH [%u]",(unsigned) bytes_read, (unsigned) len);
		log_write(errmsg);
		len = bytes_read;
    }

    last = content + len;
    p1 = content;

    /* get form data */
    errorpage[0] = '\0';
    cfgfile[0] = '\0';
    refresh = 10;
    p2 = p1;
    while(p1 <= last) 
    {
		if(*p1 == '?' || *p1 == '&' || *p1 == '\n' || *p1 == '\r' || *p1 == '\0') 
		{
	    	*p1 = '\0';
	    	p3 = strchr(p2, '=');
	    	if(p3) 
	    	{
				*p3 = '\0';
				unencode(p2, buf2);
				p3++;
				unencode(p3, buf);
				if(!strcmp(buf2, "cfgfile"))
				{
		    		strcpy(cfgfile, buf);
				}
				if(!strcmp(buf2, "errorpage"))
		    	{
		    		strcpy(errorpage, buf);	
				}
				if(!strcmp(buf2, "refresh"))
		    	{
		    		refresh = strtoul(buf, NULL, 10);	
	    		}
	    	}
	    	p2 = p1 + 1;
		}
		p1++;
    }

    
    if(errorpage[0] == '\0')
    {
		log_write("Warning: post contents did not include an error page");
    }
    
    if(cfgfile[0] == '\0') 
    {
		log_write("Error: post contents did not include an cfgfile name");
		error_page("Error: post contents did not include an cfgfile name");
    }
    
    
	fp = fopen(cfgfile,"rb");
    if(NULL == fp) 
    {
		log_write("Error: failed to open cfgfile");
		error_page("Error: failed to open cfgfile");
    }
    else
    {
    	/*用上传的文件替换/tmp/.apcfg文件*/
	   	snprintf(cmd, 4096, "mv %s /tmp/.apcfg", cfgfile);
	   	system(cmd);
	   	
		/*将缓存的内容提交到flash*/
	   	snprintf(cmd, 4096, "cfg -c");
	   	system(cmd);

		fprintf(stdout,
				"\
				Server: %s\n\
				Pragma: no-cache\n\
				Content-type: text/html\n",
				getenv("SERVER_SOFTWARE"));
		fprintf(stdout,"\n\
				<html>\n\
				<body> <h3> Import Settings</h3>");
    	fprintf(stdout,"cfgfile upload success!</body></html>");

		log_write("Success: uploadcfg file success"); 	
	}
	
    fclose(fp); 

	exit(EXIT_SUCCESS);
}




