/*****************************************************************************
**
** cgiMain.c
**
** This is a CGI function that is used to process the tagged HTML files that
** contain the environmental markers.  It also provides a "translation"
** function that will replace tagged parameters with their equivalent values.
**
** Copyright (c) 2009, Atheros Communications Inc.
**
** Permission to use, copy, modify, and/or distribute this software for any
** purpose with or without fee is hereby granted, provided that the above
** copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
** WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
** MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
** ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
** WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
** ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
** OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
**
******************************************************************************/

/*
** include files
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/reboot.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ctype.h>
#include <net/if.h>
#include <sys/ioctl.h>

#ifdef CONFIG_NVRAM
#define NVRAM  "/dev/nvram"
#define NVRAM_OFFSET 0
#else
#define NVRAM  "/dev/caldata"
#define NVRAM_OFFSET (32 * 1024)
#endif

#define PORT_FORWARD_CHAIN	"port_forward"
#define PROTO_NONE		0
#define PROTO_TCP		1
#define PROTO_UDP		2
#define PROTO_TCP_UDP	3
#define PROTO_ICMP		4

#define MaxRulesCount   32

#define DMZ_CHAIN       "dmz"
#define DMZ_PORT        80

#define PORT_FILTER_CHAIN    "macipport_filter"
#define ACTION_DROP    0
#define ACTION_ACCEPT  1
/*
** local definitions
*****************
** Local Types
** Structure element for parameter display
*/

typedef struct {
char    *val;
char    *label;
} t_DispLable;

typedef struct {
char    Name[32];
char    Val[80];
} t_singleParam;

#define MAX_WLAN_ELEMENT    1024

typedef struct {
int             numParams;
t_singleParam   Param[MAX_WLAN_ELEMENT];
} EnvParam_t;

/*
** Global Data
*/

extern char **environ;

/*
** Defined flags
*/

#define KEY_IS_WPA	0x00
#define KEY_IS_WEP	0x01
#define SILENT_ERROR	0x02

/*
** This bit in the sync word is used to indicate that parameters have been
** written to either cache or the flash, and that the factory default
** data has been changed
*/

#define NOT_FACTORY_DEFAULT		0x01

/*
**  Data
*/

#ifdef ATH_SINGLE_CFG
#define PRIVATE_C
#else
#define PRIVATE_C static
#endif

static  EnvParam_t      config;
static  char            rspBuff[65536];
PRIVATE_C  char         opBuff[65536];
static  unsigned int    parameterIndex = 0;
static  int		radioIndex = 0;
static  char            additionalParams[5][32];
static  unsigned int    numAdditionalParams = 0;
PRIVATE_C  unsigned     AbortFlag=0;
PRIVATE_C  unsigned     ModeFlag=0; // 0 = web page translation, 1 = cmdline
static  int		FactoryDefault = 0;  // indicates that parameters have been changed if zero

static FILE *errOut;

/*
** Internal Prototypes
*/

char *CFG_get_by_name(char *name,char *buff);
char *extractParam(char *str,char *Name,char *Val);
char *Execute_cmd(char *cmd,char *buffer);
char *expandOutput(char *inBuff, char *outBuff);
void htmlEncode(char *src, char *dest);
int	isKeyHex(char *key,int flags);

int  CFG_set_by_name(char *name,char *val);

/******************************************************************************/
/*!
**  \brief Print output to HTML or standard text
**
**  This macro will output a string based either on being in HTML mode
**  or in command line mode.  Used for Debug/Error messages that will show
**  up on the web page or on console output.
**
*/

#define modePrintf(x...) if(ModeFlag){printf(x);printf("\n");} \
                         else \
                         { printf("<p class='header2'>"); \
                           printf(x); printf("</p>"); }


/******************************************************************************/
/*!
**  This function will check if the user enter string has only numberic or nor
**
**  \param src Pointer to string
**  \return 0 if the string has only numberic character, return -1 if not
*/

static inline int isNumericOnly(char *pStr)
{
    while ( *pStr){
        if (!(*pStr >= '0' && *pStr <= '9'))
            return -1;/* non numeric */
        pStr++;
    }
    return 0;
}


/******************************************************************************/
/*!
**  \brief Fills parameter struct
**
**  This function will fill the parameter data structure with additional
**  parameters as read from either the temp file or the flash file system.
**
**  \param f File pointer to file containing name/value pairs
**  \return N/A
*/

void fillParamStruct(FILE *f)
{
    char            buff[256];
    unsigned long   syncWord;
    char            *vPtr;

    /*
    ** Read the first word.  It should be 0xfaf30020
    */

    fread(&syncWord,4,1,f);

	/*
	** For proper WPS operation, we need to know if ANY parameters have
	** been changed.  This is done by using a bit in the sync word
	** that indicates if any updates of the paramters from factory
	** conditions have occurred.  This bit will be set whenever the
	** cache or flash are updated, except for a  "cfg -x" which will
	** set everything back to Factory Default
	*/

	if(syncWord & NOT_FACTORY_DEFAULT)
		FactoryDefault = 0;
	else
		FactoryDefault = 1;

    if((syncWord & ~NOT_FACTORY_DEFAULT) == 0xfaf30020 )
    {
        /*
        ** File has been initialized.  Let's read until we find a NULL
        */

        while( !feof(f) )
        {
            /*
            ** Read one line at a time.  Seperated by line feed
            ** characters.
            */

            fgets(buff,256,f);

            if( buff[0] == 0 )
                break;

            /*
            ** We don't want to use extractParam here since the cache may contain
            ** "funny" characters for certain parameters.  Just assume the NAME=VAL
            ** format, terminated by 0x0a
            */

            if(vPtr=strchr(buff,0x0a))
                *vPtr = 0;  // extract the line feed
            else
                break;      // No line feed, bad line.

            vPtr = strchr(buff,'=');

            /*
            ** If this string doesn't have an "=" inserted, it's a malformed
            ** string and we should just terminate.  If it does, Replace the
            ** equal sign with a null to seperate the name and value strings,
            ** so they can be copied directly
            */

            if(!vPtr)
                break;
            else
                *vPtr++ = 0;

            /*
            ** Insert into the local structure
            */

            CFG_set_by_name(buff,vPtr);
        }
    }
}


/******************************************************************************/
/*!
**  \brief Translate variable name
**
** This function translates the variable name provided by inserting the
** appropriate index values.  The # marker indicates the index value, and
** the @ marker indicates the radio ID value.  Output buffer is provided
** by the caller
**
**  \param  varName         Pointer to variable name string with embedded markers
**  \param  transVarName    Pointer to buffer to put the fully translated name into
**  \return pointer to buffer provided for variable name
*/

char *translateName(char *varName, char *transVarName)
{
    char    *p = transVarName;

    while(*varName)
    {
        if(*varName == '#')
        {
            /*
            ** If the parameter index is greater than 1, we will put in
            ** an actual value.  If it is 1, then we ignore the # charecter
            ** and not insert anything
            */

            if(parameterIndex > 1)
                p += sprintf(p,"_%d",parameterIndex);
        }
        else if(*varName == '@')
        {
            /*
            ** Here, we'll always insert the radio index value, with
            ** no preceeding underscore.  Usually the value is 0 or
            ** 1
            */

            p += sprintf(p,"%d",radioIndex);
        }
        else
        {
            *p++ = *varName;
        }
        varName++;
    }
    /*
    ** Null terminate, jut to be thorough
    */

    *p = 0;

    return(transVarName);
}

/*****************************************************************************
**
** processSpecial
**
** This function expands special processing tags.  These tags must exist on a
** single line.  These are used as substution tags in template files that
** are replaced with values in the enviornment or in the saved parameter
** cache.
**
** Formats Supported
**
** ~`executable string`~    Indicates run the command and insert it's output
** ~cParam:Value~           For Checkbox/Radio Button support (if Param = Value)
** ~sParam:Value~           For Select support (if Param=Value)
** ~~Param:Default~     `   Direct Parameter Substution, use default if not defined
** ~?Param:Value`executable`~   Conditional execution, if Param=value
**
**  PARAMETERS:
**
**  RETURNS: 
**
*/

char *processSpecial(char *paramStr, char *outBuff)
{

    char            arg;
    char            *secArg = NULL;
    char            *exeArg = NULL;
    char            *indexPtr;
    char            *exePtr;
    char            paramVal[128];
    char            paramRawVal[70];
    char            paramIndexName[48];
    unsigned int    extraParamIndex;
    unsigned int    negate = 0;
    unsigned int    cmpVal;

    /*
    ** Get the pointers to the unique components of the parameter string
    */

    arg = *paramStr++;

    secArg = strchr(paramStr,':');

    /*
    ** If a parameter is indicated with a ! instead of a :, then any comparison
    ** is negated (used for comparison implementations)
    */

    if(!secArg)
    {
        secArg = strchr(paramStr,'!');

        if(secArg)
            negate = 1;
    }

    /*
    ** If the second argument is specified, break it out
    */

    if(secArg)
    {
        *secArg++ = 0;

        exeArg = strchr(secArg,'`');
        if(exeArg)
        {
            *exeArg++ = 0;
            exePtr = strchr(exeArg,'`');
            if(exePtr)
                *exePtr = 0;
        }
    }

    /*
    ** Get the parameter in question
    **
    ** There are two "index" markers, the index marker (which vap) and the radio marker
    ** (which radio).  We need to search for both the # and @ characters, and replace them
    ** with the proper value.  Will create a function that iterates through the parameter
    ** name string, and replaces the tokens with the appropriate values.
    */

    CFG_get_by_name(translateName(paramStr,paramIndexName),paramRawVal);


    if(ModeFlag)
    {
        /*
        ** Direct translation, don't HTMLify
        */

        strcpy(paramVal,paramRawVal);
    }
    else
    {
        htmlEncode(paramRawVal,paramVal);
    }

    /*
    ** Unique processing depends on the argument.  The supported formats will
    ** be processed individually
    */

    switch (arg)
    {
    case '~':
        /*
        ** Direct Insertion.  If no value, insert default
        */

        if( paramVal[0] == 0 && secArg != 0)
            outBuff += sprintf(outBuff,"%s",secArg);
        else
            outBuff += sprintf(outBuff,"%s",paramVal);
        break;

    case '!':
        /*
        ** Abort Line.  If the parameter has no specified OR default
        ** value, simply do not output the line.  Used for file substution
        ** for values that may or may not be there
        */

        if( paramVal[0] == 0 && secArg != 0)
            outBuff += sprintf(outBuff,"%s",secArg);
        else
            if( paramVal[0] == 0 )
                AbortFlag = 1;
            else
                outBuff += sprintf(outBuff,"%s",paramVal);
        break;

    case 'c':
        /*
        ** If the sec arg and the value are equal, then put "checked" in the output
        */

        if( secArg != NULL)
        {
            cmpVal = strcmp(paramVal,secArg);

            if( (negate && cmpVal) || (!negate && !cmpVal) )
                outBuff += sprintf(outBuff,"checked");
        }
        break;

    case 's':
        /*
        ** If the sec arg and the value are equal, then put "checked" in the output
        */

        if( secArg != NULL)
        {
            cmpVal = strcmp(paramVal,secArg);

            if( (negate && cmpVal) || (!negate && !cmpVal) )
                outBuff += sprintf(outBuff,"selected");
        }
        break;

    case '`':
        {

            /*
            ** Execute the command. Contained in paramStr for this case
            */

            exePtr = strchr(paramStr,'`');
            if( exePtr )
                *exePtr = 0;

            outBuff = expandOutput(Execute_cmd(paramStr,rspBuff),outBuff);
        }
        break;

    case '?':
        cmpVal = strcmp(paramVal,secArg);

        if( (negate && cmpVal) || (!negate && !cmpVal) )
        {
            outBuff = expandOutput(Execute_cmd(exeArg,rspBuff),outBuff);
        }

        break;

    case '$':
        /*
        ** Insert "extra" Parameter by index
        */

        extraParamIndex = atoi(paramStr) - 1;
        if(extraParamIndex < numAdditionalParams)
            outBuff += sprintf(outBuff,"%s",additionalParams[extraParamIndex]);

        break;

    case '#':
        /*
        ** Insert _Index to allow for indexed parameter names
        */

        if(parameterIndex > 1)
            outBuff += sprintf(outBuff,"_%d",parameterIndex);
        break;

    case '@':
        /*
        ** Insert the radio index number directly.  Default is 0
        */

        outBuff += sprintf(outBuff,"%d",radioIndex);
        break;

    case 'h':
        /*
        ** Enable the line if the value is a hex string.  This can be negated, like the 'e' tag.
		** If the parameter name has "PSK" inserted, it's the WPA PSK key, otherwise it's a WEP
		** key.
        */

    	if(!strncmp(paramIndexName,"PSK_KEY",7))
			cmpVal = isKeyHex(paramVal,(KEY_IS_WPA | SILENT_ERROR));
		else
			cmpVal = isKeyHex(paramVal,(KEY_IS_WEP | SILENT_ERROR));

        if( (!negate && !cmpVal) || (negate && cmpVal || cmpVal == -1) )
            AbortFlag = 1;

        break;

    case 'f':
        /*
        ** Enable the line if the the values are factory defaults.
        */

        if( (!negate && !FactoryDefault) || (negate && FactoryDefault) )
            AbortFlag = 1;

        break;

    case 'e':
        /*
        ** Enable the line.  This is used in cases where the parameter line in a file
        ** is dependant on another variable.  If it's enabled, then further processing
        ** can occur.  If not, then the line is abandoned.
        */

        cmpVal = strcmp(paramVal,secArg);
        if( (!negate && cmpVal) || (negate && !cmpVal) )
            AbortFlag = 1;

        break;
    }

    return outBuff;
}

/*****************************************************************************
**
** expandOutput
**
** This function checks the input buffer, and replaces all cr/lf or lf
** strings with <br> strings.  Used to "html-ify" output data from an
** embedded command.
**
**  PARAMETERS:
**
**  RETURNS:
**
*/

char *expandOutput(char *inBuff, char *outBuff)
{
    int wasSpace = 0;

    /*
    ** Go until the line has a NULL
    */

    while(*inBuff)
    {
        if ( *inBuff == 0x0a)
        {
            wasSpace = 0;
            inBuff++;
            strcpy(outBuff,"<br>");
            outBuff+=4;
        }
        else if ( *inBuff == 0x0d )
        {
            wasSpace = 0;
            inBuff++;
        }
        else if ( *inBuff == 0 )
        {
            if(wasSpace)
            {
                strcpy(outBuff,"&nbsp;");
                outBuff+=6;
            }
            else
                wasSpace = 1;

            inBuff++;
        }
        else if ( *inBuff == 0x08 )
        {
            wasSpace = 0;
            strcpy(outBuff,"&nbsp;&nbsp;&nbsp;&nbsp;");
            outBuff+=24;
            inBuff++;
        }
        else
        {
            wasSpace = 0;
            *outBuff++ = *inBuff++;
        }
    }

    return outBuff;
}


/*****************************************************************************
**
** expandLine
**
** This function checks the input provided, and expands any section that is
** market with the special ~ sequences.  These sequences indicate a specific
** action to be taken regarding the parameter that
** is marked with the ~param~ marker.  It returns the a line that has been
** updated with the proper strings.
**
**  PARAMETERS:
**
**  RETURNS:
**
*/

char *expandLine(char *line,char *outBuff)
{
    int     respLen;
    char    paramStr[64];
    char    *p;
    char    *tl;
    int     exeFlag = 0;

    /*
    ** Go until the line has a LF or NULL
    */

    while(*line)
    {
        if ( *line == 0x0a || *line == 0x0d)
        {
            *outBuff++ = *line++;
            break;
        }

        if ( *line == '~')
        {
            /*
            ** This is a special parameter.  The parameter string
            ** will be copied into a temporary string, and passed to
            ** the special parameter processing function.
            */

            p = paramStr;
            line++;
            *p++ =*line++;    /* this is the qualifier character, "always there" */

            if(paramStr[0] == '`')
                exeFlag = 1;    /* This is an executable string */

            while(*line != '~')
            {
                /*
                ** Check for the start of an executable string.  We don't want to process
                ** any of the executable string (gets recursively called)
                */

                if(*line == '`')
                    exeFlag = 1;

                if(exeFlag)
                {
                    /*
                    ** If this is an executable string, then we need to find the termination
                    ** of the executable string.  There are two cases
                    **
                    ** 1: the ~`...`~ case, where exeFlag is determined above, and
                    ** 2: the ~?`...`~ case, where the ~ won't be found until here.
                    */

                    while(*line != '`')
                    {
                        /*
                        ** Look for an early null character.  Abort if so
                        */
                        if(*line == 0)
                        {
                            AbortFlag = 1;
                            return NULL;
                        }
                        *p++ = *line++;
                    }

                    /*
                    ** Terminate exe string processing, but we do want
                    ** to include the last `
                    */

                    exeFlag = 0;
                }

                *p++ = *line++;
            }

            line++; /* Increment past the last ~ */
            *p = 0; /* Null Terimnate, and line now points at "after" the parameter string */

            /*
            ** At this point paramStr contains the full parameter string, ready
            ** for expansion
            */

            outBuff = processSpecial(paramStr,outBuff);

            /*
            ** If an abort flag is raised, return now
            */

            if( AbortFlag)
                return (NULL);
        }
        else
        {
            *outBuff++ = *line++;
        }
    }

    *outBuff = 0;
    return NULL;
}

/*****************************************************************************
**
** Execute_cmd
**
** This function executes the given command, and returns the results in the
** buffer provided.  Usually good for one line response commands
**
**  RETURNS:
**      Output Buffer
**
*/

char *Execute_cmd(char *cmd,char *buffer)
{
    FILE            *f;
    char            *retBuff = buffer;
    char            cmdLine[1024];

    /*
    ** Code Begins
    ** Expand the command in case it contains variables within the command line.
    ** NOTE: THIS IS A RECURSIVE CALL.  DO NOT USE GLOBALS HERE.
    */

    expandLine(cmd,cmdLine);

    /*
    ** Provide he command string to popen
    */

    f = popen(cmdLine, "r");

    if(f)
    {
        /*
        ** Read each line.
        */

        while(1)
        {
            *buffer = 0;
            fgets(buffer,120,f);
            if(strlen(buffer) == 0)
            {
                break;
            }

            strcat(buffer,"<br>");
            buffer += strlen(buffer);
        }

        pclose(f);
    }

    return(retBuff);
}

/*****************************************************************************
**
** setParamValue
**
** This function puts a parameter value into the indicated location, processing
** for %i or %s markers that require the device ID or serial number to be
** inserted.
**
**  PARAMETERS:
**
**  RETURNS:
**
*/

void setParamValue(char *targ,char *val,int check)
{
    int     index = 0;

    /*
    ** Code begins.
    ** Assume the value is null terminated
    */

    while(*val)
    {
		/*
		** If the check flag is set, we need to see if ANY character
		** is changed when putting into the variable.
		*/

		if(check && *val != targ[index])
			FactoryDefault = 0;

        if(*val == 0x0a || *val == 0x0d)
        {
            /*
            ** line feed or carrage return.  This should be truncated, end of string
            */

            break;
        }
        else
            targ[index++] = *val;

        val++;
    }

    targ[index] = 0;    // Insert null terminator
}

/******************************************************************************/
/*!
**  \brief Determine if WEP key is hex
**
**  This routine will process the ASCII representation of the WEP key, and
**  determine if it is correctly formatted.  If it is hex, will return a 1
**  If it is properly formatted and not hex, will return a 0.  If improperly
**  formatted, returns -1
**
**  \param key Null terminated character string containing the WEP key ASCII
**  \return 1 on key is formatted as HEX
**  \return 0 on key is formatted as ASCII
**  \return -1 on incorrectly formatted key
*/

int isKeyHex(char *key, int flags)
{
	int		len;
	int		i;
	int		wep = flags & 0x01;
	int		silent = flags & 0x02;

	/*
	** Get the key length (in characters)
	*/

	len = strlen(key);

	/*
	** Return 0 if the string is blank, this is OK if the fields are not
	** filled out -- will be caught later
	*/

	if(len == 0)
		return(0);

	/*
	** This works for both WEP and WPA keys.  If the WEP flag is set, only
	** check for WEP lengths.  If not, only check for WPA
	*/

	if(wep)
	{
    	/*
    	** First Pass, if the length is 0, 5, or 13 it's a string.  Can be anything
    	** Also, automatically the corresponding "IS HEX" value
    	*/

    	if(len == 5 || len == 13 || len == 16)
			return (0);
	}
	else
	{
		if(len > 7 && len < 64)
			return (0);
	}


    /*
    ** If it's not a string, then we need to determine if it's the proper length
    */

    if(wep)
    {
    	if((len != 10) && (len != 26) && (len != 32))
    	{
    	    if(!silent)
    	    	modePrintf("'%s' invalid WEP Key length (length %d != 5,13,16 for ASCII or 10,26,32 for HEX)",
    	    		   key,len);
    	    return (-1);
    	}
	}
	else
	{
   		if(len != 64)
		{
    	    if(!silent)
                modePrintf("'%s' invalid lendth (length %d != 8 to 63 chars)",key,len);
    	    return (-1);
		}
	}

	/*
	** Make sure all characters are valid hex characters
	*/

    for(i=0;i<len;i++)
    {
        if(!isxdigit(key[i]))
        {
    	    if(!silent)
                modePrintf("'%s' has non hex digit, digit='%c' pos=%d",key,key[i],i);
            return (-1);
        }
    }
	return (1); // is Hex
}

/*****************************************************************************
**
** CheckValue
**
** Performs input validation on fields that must conform to certain values.
**
**  PARAMETERS:
**
**  RETURNS: 
**
*/


int CheckValue(char *name,char *val)
{
    int     len;
    int     i;
    char    ParamName[32];

    /*
    ** Code Begins
    ** Check for the fields of interest
    **
    ** SSID.  Must be 2 to 32 characters long, certain characters
    ** now allowed.
    */

    if(!strncmp(name,"AP_SSID",7))
    {
        len = strlen(val);

        if(len > 32)
        {
            modePrintf("%s=%s invalid lendth (length %d > 32 chars)",name,val,len);
            return (-1);
        }
    }

    /*
    ** PSK Key
    ** If Defined, it must conform to the 8 to 64 character rule
    */

    if(!strncmp(name,"PSK_KEY",7) && (strlen(name) < 10))
    {
		/*
		** Get the status value
		*/

		i = isKeyHex(val,KEY_IS_WPA);

		if(i < 0)
			return (-1);	// Invalid key format
    }

    /*
    ** WEP_KEY
    ** Perform automatic determination of Hex or String.  String values are 5,13, or 16
    ** characters where hex values are 10, 26, or 32 characters.  No other lengths allowed.
    */

    if(!strncmp(name,"WEPKEY",6))
    {
		/*
		** Get the status value
		*/

		i = isKeyHex(val,KEY_IS_WEP);

		if(i < 0)
			return (-1);	// Invalid key format
    }

    if(!strncmp(name,"AP_VLAN",7))
    {
        if(val)
        {
            if (strlen(val) == 0 )
                return 0;/* Emptry string, no error */
            if ( isNumericOnly(val) != 0 ){
                modePrintf("invalid vlan tag value, 1 <= vlan <= 4094");
                return -1;
            }
            int tagval = atoi(val);
            /* tag should be between 1-4094. Linux uses 4095 for its
             * internal use 
             */
            if((tagval >= 1 && tagval <= 4094))
            {
                return 0;
            }
            else
            {
                modePrintf("invalid vlan tag value, 1 <= vlan <= 4094");
                return -1;
            }
        }
    }
    return (0); // Parameter is OK.
}


/*****************************************************************************
**
** /brief writes parameters to file
**
** This function will write the save parameter list to the file pointer
** provided.  It is assumed that the file pointer is open and positioned to
** the correct location.  File opening and closing is the responsibility of
** the caller.
*/

void writeParameters(char *name,char *mode,unsigned long offset)
{
    int         i;
    FILE        *f;
    u_int32_t   Sync = 0xfaf30020;
    u_int32_t   Zero = 0;
#ifdef ATH_SINGLE_CFG
    int         nvram_write=0;
#endif

    /*
    ** Code Begins
    ** The name of the file, and the offset into the file are passed as parameters.
    ** This will return an error (-1) if the file does not exist
    */

    if(!FactoryDefault)
        Sync |= NOT_FACTORY_DEFAULT;

    f = fopen(name,mode);

    if(f)
    {
#ifdef ATH_SINGLE_CFG
        if (!strcmp(name,NVRAM)) {
            nvram_write = 1;

            /*
             * For factory reset (before erase) don't try to
             * save the wps cfg available in the nvram
             */

            if (athcfg_prepare_nvram(f, name,
                                     (config.numParams == 0 ? 0 : 1)) != 0) {
                fclose(f);
                return;
            }
        }
#endif
        /*
        ** If an offset is provided, seek to the offset
        */

        if(offset != 0)
            fseek( f, offset, SEEK_SET);

        /*
        ** Start writing the file.  Write sync word, then parameters
        */

        fwrite(&Sync,4,1,f);

        for(i=0;i<config.numParams;i++)
        {
            /*
            ** We don't want to store the "update" or "commit" parameters, so
            ** remove them if we get here.  Also, if we have values that have
            ** no value, don't write them out.
            */

            if( !strcmp(config.Param[i].Name,"UPDATE") )
                continue;
            if( !strcmp(config.Param[i].Name,"COMMIT") )
                continue;
            if( !strcmp(config.Param[i].Name,"INDEX") )
                continue;
            if( !strcmp(config.Param[i].Name,"dmzApply") )
                continue;
            if( !strcmp(config.Param[i].Name,"PortForwardApply") )
                continue;
            if( !strcmp(config.Param[i].Name,"deleteSelPortForward") )
                continue;
            if( !strcmp(config.Param[i].Name,"NTPApply") )
                continue;
            if( !strcmp(config.Param[i].Name,"PortFilterSetApply") )
                continue;
            if( !strcmp(config.Param[i].Name,"deleteSelPortFilter") )
                continue;
            if( !strcmp(config.Param[i].Name,"PortFilterApply") )
                continue;
            if( !strcmp(config.Param[i].Name,"SystemLogClear") )
                continue;
            if( !strcmp(config.Param[i].Name,"dynamicRoutingApply") )
                continue;
            if( !strcmp(config.Param[i].Name,"AddroutingConfigApply") )
                continue;
            if( !strcmp(config.Param[i].Name,"deleteRoutingConfig") )
                continue;                                
            if( config.Param[i].Val[0] == 0)
                continue;

            fprintf(f,"%s=%s\n",config.Param[i].Name,config.Param[i].Val);
        }

        fwrite(&Zero,4,1,f);

#ifdef ATH_SINGLE_CFG

        /*
         * For factory reset, there is no need to write back the
         * wps cfg to nvram
         */

        if (config.numParams != 0 && nvram_write == 1) {
            athcfg_save_wps_cfg_to_nvram(NULL, f, 1);
        }
#endif
        /* force the buffers to be flushed to the storage device */
        fsync(fileno(f));

        fclose(f);
    }
}

/*****************************************************************************
**
** CFG_set_by_name
**
** This will set the parameter specified by name to the indicated value.  Values
** are always strings.
**
**  PARAMETERS:
**          name        Name of the parameter to modify
**          val         New value for the parameter
**
**  RETURNS:
**          0 on success, -1 on failure
**
*/

int CFG_set_by_name(char *name,char *val)
{
    int     i;
	int		check = 1;

	/*
	** List of parameters NOT to check for changes, since changing them
	** will not affect the state of WPS configured/not configured
	*/

	char	*Dont_Check[] = {"WPS_ENABLE","AP_SSID",0};

	/*
	** Code Begins
	** Check the value against the parameter name for special processing
	*/

    if( CheckValue(name, val) )
        return (-1);

	/*
	** Determine if this parameter should be checked to see if it is
	** changed from factory default
	*/

	i = 0;
	while(Dont_Check[i])
	{
		if(!strncmp(name, Dont_Check[i], strlen(Dont_Check[i++])) )
		{
			check = 0;
			break;
		}
	}

	/*
	** Now, search the list and get the proper slot for this
	** parameter
	*/

    for( i=0; i < config.numParams; i++ )
    {
        if( !strcmp(config.Param[i].Name,name) )
        {
            /*
            ** This is der one.
            */

            setParamValue(config.Param[i].Val, val, check);
            return (0);     // Done
        }
    }

    /*
    ** If we get here, we did not find the item.  Insert as a new one
    */

    if(config.numParams < MAX_WLAN_ELEMENT)
    {
        strcpy(config.Param[config.numParams].Name,name);
        setParamValue(config.Param[config.numParams++].Val,val,0);
    }

    return (0);
}

/*****************************************************************************
**
** CFG_get_by_name
**
** This function gets the parameters from the config structure
**
**  PARAMETERS:
**
**  RETURNS: 
**
*/

char *CFG_get_by_name(char *name,char *buff)
{
    int     i;

    *buff = '\0';  // null terminate

    for(i=0;i<config.numParams;i++)
    {
        if(!strcmp(config.Param[i].Name,name))
        {
            /*
            ** This is der one.  Find out if there is a %s or %i
            ** in the stream.  If so, insert the proper value
            */

            strcpy(buff,config.Param[i].Val);
            break;
        }
    }

    return buff;     // Done
}

/*****************************************************************************
**
** CFG_remove_by_name
**
** This function removes a parameter from the config structure
**
**  PARAMETERS:
**
**  RETURNS: 
**
*/

void CFG_remove_by_name(char *name)
{
    int     i;


    for(i=0;i<config.numParams;i++)
    {
        if(!strcmp(config.Param[i].Name,name))
        {
            /*
            ** This is the one.  Move the rest of the items on the list
            ** "up" by one, and decrement the total number of
            ** parameters
            */

            for(i++;i<config.numParams;i++)
            {
                strcpy(config.Param[i-1].Name,config.Param[i].Name);
                strcpy(config.Param[i-1].Val,config.Param[i].Val);
            }
            config.numParams--;
            return;
        }
    }
}

/*****************************************************************************
**
** extractVal
**
** This function returns both the value name and value string for the next item in
** the list.  It returns a pointer to the next value, or NULL if the string has
** "run out".
**
**  PARAMETERS:
**
**  RETURNS:
**
*/

char *extractParam(char *str,char *Name,char *Val)
{
    int     param=0;
    int     val = 0;

    /*
    ** Code Begins
    ** Search for the ? or & to start the string
    */

    while(*str)
    {
        /*
        ** Check for the beginning ? or &.  This signifies the start or
        ** end of the parameter. start is null at the start
        */

        if(*str == '?' || *str=='&' || *str == 0x0a || *str == 0x0d )
        {
            if(!param)
            {
                param = 1;
            }
            else
            {
                /*
                ** All Done.  Return this pointer
                */

                *Val = 0;
                if(*str == 0x0a || *str == 0x0d)
                    return(NULL);
                return (str);
            }
        }
        else if(*str == '=')
        {
            val = 1;
            *Name = 0;  // Null terminate
        }
        else if(!val)
        {
            param = 1;
            *Name++ = *str;
        }
        else
            *Val++ = *str;

        str++;
    }

    if(val)
        *Val = 0;

    /*
    ** If we get here, we have run out before getting a complete
    ** parameter. End of the line
    */

    return (NULL);
}

/******************************************************************************/
/*!
**  \brief converts strings to HTML format
**
**  This function will translate HTML special characters to the equivalent
**  HTML form for display.  This is only required for display purposes,
**  not used for command line optiosn.
**
**  \param src Pointer to source string
**  \param dest Pointer to destination string
**  \return N/A
*/

void htmlEncode(char *src, char *dest)
{
    /*
    ** Code Begins
    ** Search for special characters to do encoding as required
    */

    while(*src)
    {
        switch (*src)
        {
        case 0x22:  // Quote Character
            dest += sprintf(dest,"&quot;");
            break;

        case '&':
            dest += sprintf(dest,"&amp;");
            break;

        case '>':
            dest += sprintf(dest,"&gt;");
            break;

        case '<':
            dest += sprintf(dest,"&lt;");
            break;

        default:
            *dest++ = *src;
        }
        src++;
    }
    /*
    ** Put in the terminal null for the destination
    */

    *dest = 0;
}

/******************************************************************************/
/*!
**  \brief converts strings to "shell safe" format
**
**  This function will translate strings into "shell safe" forms that can be
**  exported.  It will detect special characters and "escape" them as required.
**  Note that some characters cannot be escaped, so this will not always
**  work.  The return value will indicate whether the string should be
**  enclosed in quotes or not.
**
**  \param src Pointer to source string
**  \param dest Pointer to destination string
**  \return 0 for no quotes required
**  \return 1 for quotes required.
*/

int shellEncode(char *src, char *dest)
{
    int needQuot = 0;

    /*
    ** Code Begins
    ** Search for special characters to do encoding as required
    */

    while(*src)
    {
        switch (*src)
        {
	case '$':  // used in md5-crypt strings
        case 0x22:  //Quote Character
            dest += sprintf(dest,"\\%c", *src);
            needQuot = 1;
            break;
        default:
            *dest++ = *src ;
        }

         if(((*src < '0') || (*src > '9')) &&
        ((*src < 'a') || (*src > 'z')) &&
            ((*src < 'A') || (*src > 'Z')))
          {
            needQuot = 1;
          }

         src++;
         }

      /*
      ** Put in the terminal null for the destination
      */

      *dest =0;
      return(needQuot);
}

/******************************************************************************/
/*!
**  \brief decodes a string from HTML format (%hex)
**
**  This function will translate HTML special characters from the HTML form
**  of %xx to the equivalent 8 bit character.  Used to process input from
**  GET/POST transactions.
**
**  \param src Pointer to source string
**  \param dest Pointer to destination string
**  \return N/A
*/

void unencode(char *src, char *dest)
{
    for(; *src != '\0'; src++)
    {
        if(*src == '+') 
        {
            *dest = ' ';
            dest++;
        }
        //decoding special symbols
        else if(*src == '%') 
        {
            int code;
            if(sscanf(src+1, "%2x", &code) != 1) code = '?';
            // ignoring all newline symbols that come from form - we put our own instead of "&varname1=" --> '\n'
            if(code != 10 && code != 12 && code != 13)
            {
                *dest = code;
                dest++;
            }
            src +=2;
        }
        else
        {
            *dest = *src;
            dest++;
        }
    } //outer for loop

    *dest = 0;
}

#ifndef ATH_SINGLE_CFG
/*****************************************************************************
**
** /brief translateFile
**
** This function will read a provided file name, and output the file with
** any substutions included.  This is used to translate template files into
** specific files that have required parameters included.
**
** An optional "index" variable will be used to look for parameters that
** have a specific index (such as AP_SSID_2, etc).  If no index is specified,
** then the parameter is assumed to be not there.  If the index is specified,
** then parameters with a tailing "_#" will have # replaced with the parameter
** ID.
**
**  \param fname File pointer to input file to translate.
**  \return 0 for success
**  \return -1 on error
*/

int translateFile(char *fname)
{
    char            Name[32];
    char            Value[64];
    char            line[1024];
    FILE            *f;

    /*
    ** Code Begins.
    ** Input the parameter cache for processing
    */

    f = fopen(fname,"r");

    if ( !f )
    {
        return (-1);
    }

    /*
    ** Read the file, one line at a time.  If the line is aborted, then
    ** dump the line and continue
    */

    while(!feof(f))
    {
        line[0] = 0;
        fgets(line,1024,f);
        expandLine(line,opBuff);

        if( !AbortFlag )
            printf("%s",opBuff);
        else
            AbortFlag = 0;

        opBuff[0] = 0;  // clear the buffer for the next cycle

    }

    fclose ( f );

    return (0);
}
#endif /* #ifndef ATH_SINGLE_CFG */

/*****************************************************************************
**
** /brief getRadioID
**
** This function determine if the radio ID is defined as specified by
** the index value.
**
**  \param index	index of the VAP to check.
**  \return radioID	On success
**  \return -1 on error
*/


int getRadioID(int index)
{
    char    varname[32];
    char    valBuff[32];
    int     len;

//    fprintf(errOut,"%s: Index set to %d\n",__func__,parameterIndex);
    if(index > 1)
        sprintf(varname,"AP_RADIO_ID_%d",parameterIndex);
    else
        strcpy(varname,"AP_RADIO_ID");

    valBuff[0] = 0;
//    fprintf(errOut,"%s: Getting %s\n",__func__,varname);

    CFG_get_by_name(varname,valBuff);

    /*
    ** Only process if a non-null string is returned.  This is to protect the
    ** single radio implementations that don't need the radio ID variable.
    */

    if(strlen(valBuff))
    {
        return(atoi(valBuff));
	}
	else
		return (-1);
}


/**************************************************************************/	
/*************************add for port forwarding**************************/
/**************************************************************************/
/*
 * description: return WAN interface name
 *              0 = bridge, 1 = gateway, 2 = wirelss isp
 */
char* getWanIfName(void)
{
	char valBuff[128] = {0};
	char buf[128] = {0};
	char *if_name     = "br0";

	CFG_get_by_name("NETWORK_MODE",valBuff);
	if (NULL == valBuff)
	{
		return if_name;
	}
	if (!strcmp(valBuff, "Bridge"))
	{
		if_name = "br0";
	}
	else 
	{
		memset(buf, 0, sizeof(buf));
	    CFG_get_by_name("WAN_PROTO",buf);	
		
		if( !strcmp(valBuff, "Router") && !strcmp(buf, "PPPOE") )
    	{
        	if_name = "ppp0";
		}
		else
		{		
			if_name = "eth1";
		}
	}

	return if_name;
}
/*
 * description: return LAN interface name
 */
char* getLanIfName(void)
{
	char * if_name = "br0";
	
	return if_name;
}
int isIpValid(char *str)
{
	struct in_addr addr;	// for examination

	if(!(inet_aton(str, &addr)))
	{
		printf("isIpValid(): %s is not a valid IP address.\n", str);
		return 0;
	}
	return 1;
}

/*
 * substitution of getNthValue which dosen't destroy the original value
 */
int getNthValueSafe(int index, char *value, char delimit, char *result, int len)
{
    int  i          = 0;
    int  result_len = 0;
    char *begin     = NULL;
    char *end       = NULL;

    if(!value || !result || !len)
    {
        return -1;
	}
	
    begin = value;
    end = strchr(begin, delimit);

    while(i < index && end)
    {
        begin = end + 1;
        end = strchr(begin, delimit);
        i++;
    }

    //no delimit
    if(!end)
    {
		if(i == index)
		{
			end = begin + strlen(begin);
			result_len = (len-1) < (end-begin) ? (len-1) : (end-begin);
		}
		else
		{
			return -1;
		}
	}
	else
	{
		result_len = (len-1) < (end-begin)? (len-1) : (end-begin);
	}
	
	memcpy(result, begin, result_len );
	*(result+ result_len ) = '\0';

	return 0;
}


void makePortForwardRule(char *wan_name, char *ip_address, char *proto, int prf_int, int prt_int)
{
	char buf[128] = {0};
//	char val[70]; 
//	char cmd[140] = {0};
	
	snprintf(buf, sizeof(buf), "iptables -t nat -A %s -i %s -p %s --dport %d:%d -j DNAT --to %s", 
	         PORT_FORWARD_CHAIN, wan_name, proto, prf_int, prt_int, ip_address);
	system(buf);
		
/*	snprintf(buf, sizeof(buf), "iptables -A FORWARD -d %s -p %s --dport %d:%d -j ACCEPT", ip_address, proto, prf_int, prt_int);
	system(buf);
				
	CFG_get_by_name("LAN_IPADDR", val);
	if(!val || !strlen(val))
	{
		snprintf(buf, sizeof(buf), "iptables -t nat -A POSTROUTING -d %s -p %s --dport %d:%d -j SNAT",	ip_address, proto, prf_int, prt_int);		
	}
	else
	{
		snprintf(buf, sizeof(buf), "iptables -t nat -A POSTROUTING -d %s -p %s --dport %d:%d -j SNAT --to %s",	ip_address, proto, prf_int, prt_int, val);		
	}
	
	system(buf);*/
	return;
}

void iptablesPortForwardRun()
{
	int  i              = 0;
	int  rule_count     = 0;
	int  prf_int        = 0;
	int  prt_int        = 0;
	int  proto          = 0;
	char cmd[128]       = {0};
	char wan_name[16]   = {0};
	char pfe[8]         = {0};
	char ip_address[32] = {0};
	char prf[8]         = {0};
	char prt[8]         = {0};
	char protocol[8]    = {0};
    char rule[70]       = {0};
    char value[MaxRulesCount][70] = {0};	
    
	CFG_get_by_name("PortForwardEnabled", pfe);
	if(!pfe || !atoi(pfe))
	{
        return;
	}

	for(i = 0, rule_count = 0; i < MaxRulesCount; i++)
	{
		snprintf(cmd, sizeof(cmd),"PortForwardRules%d", i);
		CFG_get_by_name(cmd,rule);
		if(!rule || !strlen(rule))
		{
			continue;
	    }
	    else
	    {
	    	strcpy(value[rule_count++], rule);
	    }
    }
    if(rule_count == 0)
    {
		// there is no PortForwardRules        
        return;
	}

	strncpy(wan_name, getWanIfName(), sizeof(wan_name)-1);
	for(i = 0; i < rule_count; i++)
	{
		// get ip address
		if( getNthValueSafe(0, value[i], ',', ip_address, sizeof(ip_address)) == -1 )
		{
			printf("ip_address = %s\n", ip_address);	
			continue;
		}
		if(!isIpValid(ip_address))
		{
			continue;
		}
		// get port range "from"
		if( getNthValueSafe(1, value[i], ',', prf, sizeof(prf)) == -1 )
		{
			printf("prf = %s\n", prf);	
			continue;
		}
		if( (prf_int = atoi(prf)) <= 0 || prf_int > 65535)
		{
			continue;
		}
		// get port range "to"
		if( getNthValueSafe(2, value[i], ',', prt, sizeof(prt)) == -1 )
		{
			printf("prt = %s\n", prt);	
			continue;
		}
		if( (prt_int = atoi(prt)) > 65535)
		{
			continue;
		}
		// get protocol
		if( getNthValueSafe(3, value[i], ',', protocol, sizeof(protocol)) == -1 )
		{
			continue;
		}
		proto = atoi(protocol);
		switch(proto)
		{
			case PROTO_TCP:
			{
				makePortForwardRule(wan_name, ip_address, "tcp", prf_int, prt_int);
				break;
			}
			case PROTO_UDP:
			{
				makePortForwardRule(wan_name, ip_address, "udp", prf_int, prt_int);
				break;
			}
			case PROTO_TCP_UDP:
			{
				makePortForwardRule(wan_name, ip_address, "tcp", prf_int, prt_int);
				makePortForwardRule(wan_name, ip_address, "udp", prf_int, prt_int);
				break;
			}
			default:
			{
				continue;
			}
		}
	}
}

void iptablesPortForwardFlush()
{
	char cmd[120] = {0};
    snprintf(cmd, 120, "iptables -t nat -F %s ",PORT_FORWARD_CHAIN);
    system(cmd);
}

/*
 * ASP function
 */
void showPortForwardRulesASP()
{
	int  i              = 0;
	int  rule_count     = 0;
	int  prf_int        = 0;
	int  prt_int        = 0;
	int  proto          = 0;
	char ip_address[32] = {0};
	char prf[8]         = {0};
	char prt[8]         = {0};
	char protocol[8]    = {0};
	char comment[33]    = {0};
	char name[32]       = {0};
	char rule[70]       = {0};
	char value[MaxRulesCount][70] = {0};	
	
	for(i = 0, rule_count = 0; i < MaxRulesCount; i++)
	{
		snprintf(name, sizeof(name),"PortForwardRules%d", i);
		CFG_get_by_name(name,rule);
		if(!rule || !strlen(rule))
		{
			continue;
	    }
	    else
	    {
	    	strcpy(value[rule_count++], rule);
	    }
    }

	/*
	 * format is : [ip],[port_from],[port_to],[protocol],[comment],;
	 */
	for(i = 0; i < rule_count; i++)
	{
		// get ip address
		if( getNthValueSafe(0, value[i], ',', ip_address, sizeof(ip_address)) == -1 )
		{
			continue;
		}
		if(!isIpValid(ip_address))
		{
			continue;
		}

		// get port range "from"
		if( getNthValueSafe(1, value[i], ',', prf, sizeof(prf)) == -1 )
		{
			continue;
		}
		if( (prf_int = atoi(prf)) <= 0 || prf_int > 65535)
		{
			continue;
		}

		// get port range "to"
		if( getNthValueSafe(2, value[i], ',', prt, sizeof(prt)) == -1 )
		{
			continue;
		}
		if((prt_int = atoi(prt)) <= 0 || prt_int > 65535)
		{
			continue;
		}

		// get protocol
		if( getNthValueSafe(3, value[i], ',', protocol, sizeof(protocol)) == -1 )
		{
			continue;
		}
		proto = atoi(protocol);
		switch(proto)
		{
			case PROTO_TCP:
			case PROTO_UDP:
			case PROTO_TCP_UDP:
			{
				break;
			}
			default:
			{	
				continue;
			}
		}

		if( getNthValueSafe(4, value[i], ',', comment, sizeof(comment)) == -1 )
		{
			continue;
		}

		printf("<tr>\n");
		// output No.
		printf("<td class=headind> %d&nbsp; <input type=\"checkbox\" name=\"delRule%d\"> </td>\n", i+1, i);

		// output IP address
		printf("<td align=center class=headind> %s </td>\n", ip_address);

		// output Port Range
		if(prt_int)
		{
			printf("<td align=center class=headind> %d - %d </td>\n", prf_int, prt_int);
		}
		else
		{
			printf("<td align=center class=headind> %d </td>\n", prf_int);
		}
		// output Protocol
        switch(proto)
        {
            case PROTO_TCP:
			{	
				printf("<td align=center class=headind> TCP </td>\n");
				break;
			}
            case PROTO_UDP:
			{
				printf("<td align=center class=headind> UDP </td>\n");
				break;
			}
            case PROTO_TCP_UDP:
			{	
				printf("<td align=center class=headind> TCP&UDP </td>\n");
				break;
			}
		}

		// output Comment
		if(strlen(comment))
		{
			printf("<td align=center class=headind> %s</td>\n", comment);
		}
		else
		{
			printf("<td align=center> &nbsp; </td>\n");
		}
		printf("</tr>\n");
	}
	printf("</table>\n");
	printf("</form>\n");
	printf("</body></html>\n");

	return;	
}

void portForward()
{
	int  i               = 0;
	int  prf_int         = 0;
	int  prt_int         = 0;
	int  proto           = 0;
	char pfe[8]          = {0};
	char ip_address[32]  = {0};
	char prf[8]          = {0};
	char prt[8]          = {0};
	char protocol[8]     = {0};
	char comment[36]     = {0};
	int  old_prf_int     = 0;
	int  old_prt_int     = 0;
	int  old_proto       = 0;
	char old_ip[32]      = {0};
	char old_prf[8]      = {0};
	char old_prt[8]      = {0};
	char old_protocol[8] = {0};
	char cmd[128]        = {0};          
	char rule[70]        = {0};
	char valBuff[70]     = {0};

	CFG_get_by_name("PortForwardEnabled", pfe);	
	CFG_get_by_name("PortForward_ip_address", ip_address);
	CFG_get_by_name("PortForward_fromPort", prf);
	CFG_get_by_name("PortForward_toPort", prt);
	CFG_get_by_name("PortForward_protocol", protocol);
	CFG_get_by_name("PortForward_comment", comment);

	if(!pfe || !strlen(pfe))
	{
		return;
	}
	if(!atoi(pfe))
	{
		iptablesPortForwardFlush();		//disable
		return;
	}

	if(!strlen(ip_address) && !strlen(prf) && !strlen(prt) && !strlen(comment))
	{	
		// user choose nothing but press "apply" only
		iptablesPortForwardFlush();
		iptablesPortForwardRun();
		return;
	}

	if(!ip_address || !strlen(ip_address))
	{
		return;
	}
	if(!isIpValid(ip_address))
	{
		return;
	}
	// we dont trust user input.....
	if(!prf || !strlen(prf))
	{
		return;
	}
	if(!(prf_int = atoi(prf)) || (prf_int > 65535))
	{
		return;
	}

	if(!prt || !strlen(prt))
	{
		return;
	}
	if(!(prt_int = atoi(prt)) || (prt_int > 65535))
	{
		return;
	}
	if(prt_int < prf_int)
	{
		return;
	}

	if(!strcmp(protocol, "TCP"))
	{
		proto = PROTO_TCP;
	}
	else if(!strcmp(protocol, "UDP"))
	{
		proto = PROTO_UDP;
	}
	else if(!strcmp(protocol, "TCPUDP"))
	{
		proto = PROTO_TCP_UDP;
	}
	else
	{
		return;
	}
	
	if(strlen(comment) > 32)
	{
		return;
	}
	/* i know you will try to break our box... ;) */
	if(strchr(comment, ','))
	{
		return;
	}
	
	for (i = 0; i < MaxRulesCount; i++)
	{
		snprintf(cmd, sizeof(cmd), "PortForwardRules%d", i);
		CFG_get_by_name(cmd, valBuff);
		if( valBuff && strlen(valBuff) )
		{
			// get ip address
			if( getNthValueSafe(0, valBuff, ',', old_ip, sizeof(old_ip)) == -1 )
			{
				continue;
			}
			if(!isIpValid(old_ip))
			{
				continue;
			}

			// get port range "from"
			if( getNthValueSafe(1, valBuff, ',', old_prf, sizeof(old_prf)) == -1 )
			{
				continue;
			}
			if((old_prf_int = atoi(old_prf)) <= 0 || old_prf_int > 65535)
			{
				continue;
			}

			// get port range "to"
			if( getNthValueSafe(2, valBuff, ',', old_prt, sizeof(old_prt)) == -1 )
			{
				continue;
			}
			if((old_prt_int = atoi(prt)) <= 0 || old_prt_int > 65535)
			{
				continue;
			}

			// get protocol
			if( getNthValueSafe(3, valBuff, ',', old_protocol, sizeof(old_protocol)) == -1 )
			{
				continue;
			}
			old_proto = atoi(old_protocol);
			
			if(!strcmp(ip_address, old_ip) && (old_prf_int == prf_int) && (old_prt_int == prt_int) && (old_proto == proto))
			{
				printf("Already have this configuration!\n");
				break;
			}
		}
		else
		{
			sprintf(rule, "%s,%d,%d,%d,%s", ip_address, prf_int, prt_int, proto, comment);
			CFG_set_by_name(cmd, rule);
			writeParameters("/tmp/.apcfg","w+",0);
			writeParameters(NVRAM,"w+", NVRAM_OFFSET);
			break;
		}
	}

	iptablesPortForwardFlush();
	iptablesPortForwardRun();

	return;				
}

void portForwardDelete()
{
	int  i            = 0;
	int  j            = 0;
	int  flag         = 0;   //mark if some rules to delete
	int  delete_count = 0;
	int  rule_count   = 0;   //count the number of portForwardRules
	char name_buf[16] = {0};
	char val[32]      = {0};
	char cmd[128]     = {0};
	char rule[70]     = {0};
	char delRule[MaxRulesCount][32] = {0};
	int  delRuleNum = 0;
	
		
	for(i = 0, rule_count = 0; i < MaxRulesCount; i++)
	{
		memset(cmd, 0, sizeof(cmd));
		memset(rule, 0, sizeof(rule));			

		sprintf(cmd, "PortForwardRules%d", i);
		CFG_get_by_name(cmd,rule);
		if(!rule || !strlen(rule))
		{	
			continue;
	    }
	    else
	    {
			rule_count++;
	    }
    }
    if(rule_count == 0)
    {
		// there is no PortForwardRules        
        return ;
	}				

	for(i = 0; i < rule_count; i++)
	{
		snprintf(name_buf, 16, "delRule%d", i);
		CFG_get_by_name(name_buf,val);
		if(!val || !strlen(val))
		{
			continue;
		}
		else //delete this record
		{
			if (0 == flag)
			{
				flag = 1;
			}
			
			for(j = 0, delete_count = 0; j < MaxRulesCount; j++)
			{
				memset(cmd, 0, sizeof(cmd));
				memset(rule, 0, sizeof(rule));
				
				snprintf(cmd,sizeof(cmd),"PortForwardRules%d",j);
				CFG_get_by_name(cmd, rule);
				if( !rule || !strlen(rule))
				{
					continue;
				}
				else
				{
					delete_count++;									
					if( i == (delete_count-1) )
					{	
						strcpy(delRule[delRuleNum++], cmd);
						CFG_remove_by_name(name_buf);
						break;
					}
				}
			}
			
		}
	}

    if(0 == flag)
    {
        printf("You didn't select any rules to delete.<br>\n");
        return;
    }
	
	for (j = 0; j < delRuleNum; j++)
	{
		CFG_remove_by_name(delRule[j]);
	}
	
	writeParameters("/tmp/.apcfg","w+",0);
	writeParameters(NVRAM,"w+", NVRAM_OFFSET);
	// restart iptables if it is running
	CFG_get_by_name("PortForwardEnabled",val);
	if(val && atoi(val))
	{
		iptablesPortForwardFlush();
		iptablesPortForwardRun();
	}

	return;
}

/**************************************************************************/	
/*************************end for port forwarding**************************/
/**************************************************************************/


/*************************start for port filtering**************************/
/*** mac地址形式如下：00:e0:4c:00:4d:c0 ***/
int isMacValid(char *str)
{
	int i;
	int len = strlen(str);
	
	if(len != 17)
	{
		return 0;
	}
	
	for(i=0; i<5; i++)
	{
		if((!isxdigit( str[i*3])) || (!isxdigit( str[i*3+1])) || (str[i*3+2] != ':'))
		{
			return 0;
		}
	}
	
	return (isxdigit(str[15]) && isxdigit(str[16])) ? 1: 0;
}

void PortFilterFlush()
{
	char cmd[120] = {0};
	snprintf(cmd, sizeof(cmd), "iptables -t filter -F %s ",PORT_FILTER_CHAIN);
	system(cmd);
	return;
}

void makePortFilterRule(char *mac, char *sip, int sprf_int, int sprt_int, char *dip, int dprf_int, int dprt_int, int proto, int action)
{
	int  rc = 0;
	char buf[200] = {0};
	char *pos = buf;
	int  len = sizeof(buf);

	rc = snprintf(pos, len-rc, "iptables -t filter -A %s ", PORT_FILTER_CHAIN);
	pos = pos + rc;

	// write mac address
	if(mac && strlen(mac))
	{
		rc = snprintf(pos, len-rc, "-m mac --mac-source %s ", mac);
		pos = pos+rc;
	}

	// write source ip
	if(sip && strlen(sip))
	{
		rc = snprintf(pos, len-rc, "-s %s ", sip);
		pos = pos+rc;
	}
	
	// write dest ip
	if(dip && strlen(dip))
	{
		rc = snprintf(pos, len-rc, "-d %s ", dip);
		pos = pos+rc;
	}
	
	// write protocol type
	if(proto == PROTO_NONE)
	{
		rc = snprintf(pos, len-rc, " ");
		pos = pos + rc;
	}
	else if(proto == PROTO_ICMP)
	{
		rc = snprintf(pos, len-rc, "-p icmp ");
		pos = pos + rc;
	}
	else
	{
		if(proto == PROTO_TCP)
		{
			rc = snprintf(pos, len-rc, "-p tcp ");
		}
		else if (proto == PROTO_UDP)
		{
			rc = snprintf(pos, len-rc, "-p udp ");
		}
		pos = pos + rc;

		// write source port
		if(sprf_int)
		{
			if(sprt_int)
			{
				rc = snprintf(pos, len-rc, "--sport %d:%d ", sprf_int, sprt_int);
			}
			else
			{
				rc = snprintf(pos, len-rc, "--sport %d ", sprf_int);
			}
			pos = pos+rc;
		}

		// write dest port
		if(dprf_int)
		{
			if(dprt_int)
			{
				rc = snprintf(pos, len-rc, "--dport %d:%d ", dprf_int, dprt_int);
			}
			else
			{
				rc = snprintf(pos, len-rc, "--dport %d ", dprf_int);
			}
			pos = pos+rc;
		}
	}

	switch(action)
	{
		case ACTION_DROP:	// DROP mode
		{
			rc = snprintf(pos, len-rc, "-j DROP");
			break;
		}
		case ACTION_ACCEPT:	// ACCEPT mode
		{
			rc = snprintf(pos, len-rc, "-j ACCEPT");
			break;
		}
	}

	system(buf);
	return;
}

void PortFilterRun()
{
	int  i = 0;
	char pfe[10] = {0};
	int  rule_count = 0;
	char cmd[1024] = {0}, rule[100] = {0}, value[MaxRulesCount][80] = {0};
	char depolicy[8] = {0};
	char action[10] = {0};
	char mac[20] = {0};
	char dip[20] = {0}, sip[20] = {0};
	char dprf[10] = {0}, dprt[10] = {0}, sprf[10] = {0}, sprt[10] = {0};
	char proto[10] = {0};
	int  proto_int = 0;
	int  dprf_int = 0, sprf_int = 0, dprt_int = 0, sprt_int = 0;
	int  action_int = 0;
	
	CFG_get_by_name("PortFilterEnabled", pfe);
	if(!pfe || !atoi(pfe))
	{
		return;
	}

	for(i = 0, rule_count = 0; i < MaxRulesCount; i++)
	{
		snprintf(cmd, sizeof(cmd),"PortFilterRules%d", i);
		CFG_get_by_name(cmd,rule);
		if(!rule || !strlen(rule))
		{
			continue;
		}
		else
		{
		  	strcpy(value[rule_count++], rule);
		}
	}
    if(rule_count == 0)
    {
		// there is no PortFilterRules        
        return;
	}

	for(i=0; i<rule_count; i++)
	{
		// get mac address
		if( getNthValueSafe(0, value[i], ',', mac, sizeof(mac)) == -1 )
		{
			continue;
		}

		// get dest ip
		if( getNthValueSafe(1, value[i], ',', dip, sizeof(dip)) == -1 )
		{
			continue;
		}

		// get source ip
		if( getNthValueSafe(2, value[i], ',', sip, sizeof(sip)) == -1 )
		{
			continue;
		}

		// get protocol
		if( getNthValueSafe(3, value[i], ',', proto, sizeof(proto)) == -1 )
		{
			continue;
		}
		proto_int = atoi(proto);

		// get dest port from
		if( getNthValueSafe(4, value[i], ',', dprf, sizeof(dprf)) == -1 )
		{
			continue;
		}
		if((dprf_int = atoi(dprf)) > 65535)
		{
			continue;
		}
		// get dest port to
		if( getNthValueSafe(5, value[i], ',', dprt, sizeof(dprt)) == -1 )
		{
			continue;
		}
		if((dprt_int = atoi(dprt)) > 65535)
		{
			continue;
		}
	
		// get source port from
		if( getNthValueSafe(6, value[i], ',', sprf, sizeof(sprf)) == -1 )
		{
			continue;
		}
		if((sprf_int = atoi(sprf)) > 65535)
		{
			continue;
		}		
		// get source port to
		if( getNthValueSafe(7, value[i], ',', sprt, sizeof(sprt)) == -1 )
		{
			continue;
		}
		if((sprt_int = atoi(sprt)) > 65535)
		{
			continue;
		}

		// get action
		if( getNthValueSafe(8, value[i], ',', action, sizeof(action)) == -1 )
		{
			continue;
		}
		action_int = atoi(action);	

		makePortFilterRule(mac, sip, sprf_int, sprt_int, dip, dprf_int, dprt_int, proto_int, action_int);
	}

	CFG_get_by_name("defaultFirewallPolicy",depolicy);
	switch(atoi(depolicy))
	{
		case 0:
		{
			snprintf(cmd, sizeof(cmd), "iptables -t filter -A %s -j ACCEPT", PORT_FILTER_CHAIN);
			system(cmd);
			break;
		}
		case 1:
		{
			snprintf(cmd, sizeof(cmd), "iptables -t filter -A %s -j DROP", PORT_FILTER_CHAIN);
			system(cmd);
			break;
		}
	}

	return;
}
/*
 * ASP function
 */
void showPortFilterRulesASP()
{
	int  i = 0;
	int  rule_count = 0;
	char cmd[100] = {0}, rule[100] = {0}, value[MaxRulesCount][80] = {0};
	char depolicy[8] = {0};
	char action[10] = {0};
	char mac[20] = {0};
	char dip[20] = {0}, sip[20] = {0};
	char dprf[10] = {0}, dprt[10] = {0}, sprf[10] = {0}, sprt[10] = {0};
	char proto[10] = {0};
	int  dprf_int = 0, sprf_int = 0, dprt_int = 0, sprt_int = 0;

	for(i = 0, rule_count = 0; i < MaxRulesCount; i++)
	{
		snprintf(cmd, sizeof(cmd),"PortFilterRules%d", i);
		CFG_get_by_name(cmd,rule);
		if(!rule || !strlen(rule))
		{
			continue;
	    }
	    else
	    {
	    	strcpy(value[rule_count++], rule);
	    }
    }

	/*
	 * format is : [ip],[port_from],[port_to],[protocol],[comment],;
	 */
	for(i = 0; i < rule_count; i++)
	{
		// get mac address
		if( getNthValueSafe(0, value[i], ',', mac, sizeof(mac)) == -1 )
		{
			continue;
		}

		// get dest ip
		if( getNthValueSafe(1, value[i], ',', dip, sizeof(dip)) == -1 )
		{
			continue;
		}
		
		// get source ip
		if( getNthValueSafe(2, value[i], ',', sip, sizeof(sip)) == -1 )
		{
			continue;
		}

		// get protocol
		if( getNthValueSafe(3, value[i], ',', proto, sizeof(proto)) == -1 )
		{
			continue;
		}
		
		// get dest port from
		if( getNthValueSafe(4, value[i], ',', dprf, sizeof(dprf)) == -1 )
		{
			continue;
		}
		if((dprf_int = atoi(dprf)) > 65535)
		{
			continue;
		}
		// get dest port to
		if( getNthValueSafe(5, value[i], ',', dprt, sizeof(dprt)) == -1 )
		{
			continue;
		}
		if((dprt_int = atoi(dprt)) > 65535)
		{
			continue;
		}
		
		// get source port from
		if( getNthValueSafe(6, value[i], ',', sprf, sizeof(sprf)) == -1 )
		{
			continue;
		}
		if((sprf_int = atoi(sprf)) > 65535)
		{
			continue;
		}		
		// get source port to
		if( getNthValueSafe(7, value[i], ',', sprt, sizeof(sprt)) == -1 )
		{
			continue;
		}
		if((sprt_int = atoi(sprt)) > 65535)
		{
			continue;
		}
		
		// get action
		if( getNthValueSafe(8, value[i], ',', action, sizeof(action)) == -1 )
		{
			continue;
		}

		printf("<tr>\n");
		// output No.
		printf("<td align=center class=headind> %d&nbsp; <input type=\"checkbox\" name=\"delPortFilterRule%d\"></td>\n", i+1, i);
		// output Mac address
		if(strlen(mac) != 0)
		{
			printf("<td align=center class=headind> %s </td>\n", mac);
		}
		else
		{			
			printf("<td align=center> - </td>\n");
		}
		
		// output dest ip address
		if(strlen(dip) != 0)
		{
			printf("<td align=center class=headind> %s </td>\n", dip);
		}
		else
		{
			printf("<td align=center> - </td>\n");
		}
		
		//output source ip address
		if(strlen(sip) != 0)
		{
			printf("<td align=center class=headind> %s </td>\n", sip);
		}
		else
		{
			printf("<td align=center> - </td>\n");
		}
		
		//output proto
		switch(atoi(proto))
		{
			case PROTO_NONE:
			{
				printf("<td align=center class=headind>None</td>\n");
				break;
			}
			case PROTO_TCP:
			{
				printf("<td align=center class=headind>TCP</td>\n");
				break;	
			}
			case PROTO_UDP:
			{
				printf("<td align=center class=headind>UDP</td>\n");
				break;
			}
			case PROTO_ICMP:
			{
				printf("<td align=center class=headind>ICMP</td>\n");
				break;
			}
		}
		
		// output dest Port Range
		if((atoi(dprf) == 0) && (atoi(dprt) == 0))
		{
			printf("<td align=center> - </td>\n");
		}
		else
		{
			if(atoi(dprt) == 0)
			{
				printf("<td align=center class=headind>%s</td>\n",dprf);
			}
			else
			{
				printf("<td align=center class=headind> %s - %s </td>\n", dprf, dprt);
			}
		}
		
		// output source Port Range
		if((atoi(sprf) == 0) && (atoi(sprt) ==0))
		{
			printf("<td align=center> - </td>\n");
		}
		else
		{
			if(atoi(sprt) == 0)
			{
				printf("<td align=center class=headind>%s</td>\n",sprf);
			}
			else
			{
				printf("<td align=center class=headind> %s - %s </td>\n", sprf, sprt);
			}
		}

		// output action
		switch(atoi(action))
		{
			case ACTION_DROP:
			{
				printf("<td align=center class=headind>Drop</td>\n");
				break;
			}
			case ACTION_ACCEPT:
			{
				printf("<td align=center class=headind>Accept</td>\n");
				break;
			}
		}

		printf("</tr>\n");
	}
	printf("</table>\n");
	printf("</table>\n");
	printf("</form>\n");
	printf("</body></html>\n");
    
	return;	
}

void portFilterSet()
{
	char pfe[10]   = {0};
	char ip[32]    = {0};
	char cmd[100] = {0};
    
	PortFilterFlush();
    
	CFG_get_by_name("PortFilterEnabled", pfe);
	if(!pfe || !atoi(pfe))
	{
        return;
	}
	
	PortFilterRun();
	
	writeParameters("/tmp/.apcfg","w+",0);
	writeParameters(NVRAM,"w+", NVRAM_OFFSET);
	return;
}

void portFilter()
{
	char action[10] = {0};
	char mac[20] = {0};
	char dip[20] = {0}, sip[20] = {0};
	char dprf[10] = {0}, dprt[10] = {0}, sprf[10] = {0}, sprt[10] = {0};
	char proto[10] = {0};
	int  proto_int = 0;
	int  dprf_int = 0, sprf_int = 0, dprt_int = 0, sprt_int = 0;
	int  action_int = 0;
	char old_action[10] = {0};
	char old_mac[20] = {0};
	char old_dip[20] = {0}, old_sip[20] = {0};
	char old_dprf[10] = {0}, old_dprt[10] = {0}, old_sprf[10] = {0}, old_sprt[10] = {0};
	char old_proto[10] = {0};
	int  i = 0;
	char cmd[128] = {0};          
	char rule[80] = {0};
	char valBuff[80] = {0};
	
	CFG_get_by_name("PortFilterAction", action);	
	CFG_get_by_name("PortFilterMac", mac);
	CFG_get_by_name("PortFilterDestIPAddr", dip);
	CFG_get_by_name("PortFilterSourIPAddr", sip);
	CFG_get_by_name("PortFilterProtocol", proto);
	CFG_get_by_name("PortFilterDestPortFr", dprf);
	CFG_get_by_name("PortFilterDestPortTo", dprt);
	CFG_get_by_name("PortFilterSourPortFr", sprf);
	CFG_get_by_name("PortFilterSourPortTo", sprt);
	
	if(!mac || !sip || !dip || !sprf || !dprf)
	{
		return;
	}
	
	if(!strlen(mac) && !strlen(sip) && !strlen(dip) && !strlen(sprf) && !strlen(dprf))
	{
		PortFilterFlush();
		PortFilterRun();	
		return;
	}

	if(strlen(mac) && !isMacValid(mac))
	{
		return;
	}
	
	if(strlen(sip) && !isIpValid(sip))
	{
		return;
	}
	
	if(strlen(dip) && !isIpValid(dip))
	{
		return;
	}

	if( !strcmp(proto, "TCP"))
	{
		proto_int = PROTO_TCP;
	}
	else if( !strcmp(proto, "UDP"))
	{
		proto_int = PROTO_UDP;
	}
	else if( !strcmp(proto, "None"))
	{
		proto_int = PROTO_NONE;
	}
	else if( !strcmp(proto, "ICMP"))
	{
		proto_int = PROTO_ICMP;
	}
	else
	{
		return;
	}
	
	if(!strlen(sprf) || proto_int == PROTO_NONE || proto_int == PROTO_ICMP)
	{
		sprf_int = 0;
	}
	else
	{
		sprf_int = atoi(sprf);
		if(sprf_int < 0 || sprf_int > 65535)
		{
			return;
		}
	}

	if(!strlen(sprt) || proto_int == PROTO_NONE || proto_int == PROTO_ICMP)
	{
		sprt_int = 0;
	}
	else
	{
		sprt_int = atoi(sprt);
		if(sprt_int < 0 || sprt_int > 65535)
		{
			return;
		}
	}

	if(!strlen(dprf) || proto_int == PROTO_NONE || proto_int == PROTO_ICMP)
	{
		dprf_int = 0;
	}
	else
	{
		dprf_int = atoi(dprf);
		if(dprf_int < 0 || dprf_int > 65535)
		{
			return;
		}
	}

	if(!strlen(dprt) || proto_int == PROTO_NONE || proto_int == PROTO_ICMP)
	{
		dprt_int = 0;
	}
	else
	{
		dprt_int = atoi(dprt);
		if(dprt_int < 0 || dprt_int > 65535)
		{
			return;
		}
	}

	if(strlen(sprf) && strlen(sprt))
	{
		if(sprf_int > sprt_int)
		{
			return;
		}
	}
	if(strlen(dprf) && strlen(dprt))
	{
		if(dprf_int > dprt_int)
		{
			return;
		}
	}
	
	action_int = atoi(action);

	for (i = 0; i < MaxRulesCount; i++)
	{
		snprintf(cmd, sizeof(cmd), "PortFilterRules%d", i);
		CFG_get_by_name(cmd, valBuff);
		if( valBuff && strlen(valBuff) )
		{
			// get mac address
			if( getNthValueSafe(0, valBuff, ',', old_mac, sizeof(old_mac)) == -1 )
			{
				continue;
			}

			// get dest ip
			if( getNthValueSafe(1, valBuff, ',', old_dip, sizeof(old_dip)) == -1 )
			{
				continue;
			}

			// get source ip
			if( getNthValueSafe(2, valBuff, ',', old_sip, sizeof(old_sip)) == -1 )
			{
				continue;
			}

			// get protocol
			if( getNthValueSafe(3, valBuff, ',', old_proto, sizeof(old_proto)) == -1 )
			{
				continue;
			}
			
			// get dest port from
			if( getNthValueSafe(4, valBuff, ',', old_dprf, sizeof(old_dprf)) == -1 )
			{
				continue;
			}
			// get dest port to
			if( getNthValueSafe(5, valBuff, ',', old_dprt, sizeof(old_dprt)) == -1 )
			{
				continue;
			}
			// get source port from
			if( getNthValueSafe(6, valBuff, ',', old_sprf, sizeof(old_sprf)) == -1 )
			{
				continue;
			}
			// get source port to
			if( getNthValueSafe(7, valBuff, ',', old_sprt, sizeof(old_sprt)) == -1 )
			{
				continue;
			}
			// get action
			if( getNthValueSafe(8, valBuff, ',', old_action, sizeof(old_action)) == -1 )
			{
				continue;
			}						

			
			if(!strcmp(mac,old_mac) && !strcmp(dip,old_dip) && !strcmp(sip,old_sip) && !strcmp(proto,old_proto) &&
			   !strcmp(dprf,old_dprf) && !strcmp(dprt,old_dprt) && !strcmp(sprf,old_sprf) && !strcmp(sprt,old_sprt) &&
			   !strcmp(action,old_action))
			{
				printf("Already have this configuration!\n");
				break;
			}
		}
		else
		{
			sprintf(rule, "%s,%s,%s,%d,%d,%d,%d,%d,%d", mac, dip, sip, proto_int, dprf_int, dprt_int, sprf_int, sprt_int, action_int);
			CFG_set_by_name(cmd, rule);
			writeParameters("/tmp/.apcfg","w+",0);
			writeParameters(NVRAM,"w+", NVRAM_OFFSET);
			break;
		}
	}

	PortFilterFlush();
	PortFilterRun();
     
    return;
}

void portFilterDelete()
{
	int  i, rule_count;
	int  j, delete_count;
	char cmd[200], rule[200], buf[100], val[200];
	int  flag = 0;   //mark if some rules to delete
	char delRule[MaxRulesCount][32] = {0};
	int  delRuleNum = 0;
		
	for(i = 0, rule_count = 0; i < MaxRulesCount; i++)
	{
		memset(cmd, 0, sizeof(cmd));
		memset(rule, 0, sizeof(rule));			

		sprintf(cmd, "PortFilterRules%d", i);
		CFG_get_by_name(cmd,rule);
		if(!rule || !strlen(rule))
		{	
			continue;
	    }
	    else
	    {
			rule_count++;
	    }
    }
    if(rule_count == 0)
    {
		// there is no PortFilterRules        
        return ;
	}				

	for(i = 0; i < rule_count; i++)
	{
		snprintf(buf, sizeof(buf), "delPortFilterRule%d", i);
		CFG_get_by_name(buf,val);

		if(!val || !strlen(val))
		{
			continue;
		}
		else //delete this record
		{
			if (0 == flag)
			{
				flag = 1;
			}
			
			for(j = 0, delete_count = 0; j < MaxRulesCount; j++)
			{
				memset(cmd, 0, sizeof(cmd));
				memset(rule, 0, sizeof(rule));
				
				snprintf(cmd,sizeof(cmd),"PortFilterRules%d",j);
				CFG_get_by_name(cmd, rule);
				if( !rule || !strlen(rule))
				{
					continue;
				}
				else
				{
					delete_count++;									
					if( i == (delete_count-1) )
					{	
						strcpy(delRule[delRuleNum++], cmd);
						CFG_remove_by_name(buf);
						break;
					}
				}
			}			
		}
	}

    if(0 == flag)
    {
        printf("You didn't select any rules to delete.<br>\n");
        return;
    }
	
	for (j = 0; j < delRuleNum; j++)
	{
		CFG_remove_by_name(delRule[j]);
	}
	
	writeParameters("/tmp/.apcfg","w+",0);
	writeParameters(NVRAM,"w+", NVRAM_OFFSET);
	
	// restart iptables if it is running
	CFG_get_by_name("PortFilterEnabled",val);
	if(val && atoi(val))
	{
		PortFilterFlush();
		PortFilterRun();
	}

	return;
}


/*************************end for port filtering**************************/
/*
 * Add paramper 
 */
void AddRouting(void)
{
	
	char dest[20]={0}, old_dest[20] = {0};
	char hostnet[5]={0}, old_hostnet[5] = {0};
	char netmask[20]={0}, old_netmask[20] = {0};
	char gateway[20]={0}, old_gateway[20] = {0};
	char interface[10]={0}, old_interface[10] = {0};
	char true_interface[20]={0}, old_trueintf[20] = {0};
	char custom_interface[20]={0};
	char tmp[100]={0};
	int i = 0;
	char cmd[256] = {0};
	char valBuff[1024] = {0};
	char result[256] = {0};
	FILE *fp = NULL;
	
	CFG_get_by_name("StaticRouteDest", dest);
	CFG_get_by_name("StaticRouteHostNet", hostnet);
	CFG_get_by_name("StaticRouteMask", netmask);
	CFG_get_by_name("StaticRouteGateway", gateway);
	CFG_get_by_name("StaticRouteInterface", interface);
	CFG_get_by_name("StaticRouteCusIntf", custom_interface);
	
	if( !dest || !strlen(dest))
	{
		return;
	}
		
	memset(cmd, 0, sizeof(cmd));
	strcat(cmd, "route add ");

	if(!strcmp(hostnet, "net"))
	{
		strcat(cmd, "-net ");
	}
	else
	{
		strcat(cmd, "-host ");
	}
	
	// destination
	strcat(cmd, dest);
	strcat(cmd, " ");

	// netmask
	if(strlen(netmask))
	{
		sprintf(cmd, "%s netmask %s", cmd, netmask);
	}
	else
	{
		strcpy(netmask,"255.255.255.255");
	}

	//gateway
	if(strlen(gateway))
	{
		sprintf(cmd, "%s gw %s", cmd, gateway);
	}
	else
	{
		strcpy(gateway,"0.0.0.0");
	}
	
	//interface
	if(strlen(interface))
	{
		if (!strcmp(interface, "WAN"))
		{
			strncpy(true_interface, getWanIfName(), sizeof(true_interface)-1);
		}
		else if (!strcmp(interface, "Custom"))
		{
			if(!strlen(custom_interface))
			{
				return;
			}
			strcpy(true_interface,custom_interface);	
		}
		else	// LAN & unknown
		{
			strncpy(true_interface,getLanIfName(),sizeof(true_interface)-1);
		}
	}
	else
	{
		strcpy(interface,"LAN");
		strncpy(true_interface,getLanIfName(),sizeof(true_interface)-1);
	}
	
	sprintf(cmd, "%s dev %s 2>&1", cmd, true_interface);

	fp = popen(cmd, "r");
	fgets(result, sizeof(result), fp);
	pclose(fp);	
	
	if(!strlen(result))
	{
		for(i=0; i<MaxRulesCount; i++)
		{
			snprintf(cmd, sizeof(cmd), "RoutingRules%d", i);
			CFG_get_by_name(cmd, valBuff);
			
			if(valBuff && strlen(valBuff))
			{
				// get dest
				if(getNthValueSafe(0, valBuff, ',', old_dest, sizeof(old_dest)) == -1 )
				{ 
					continue;
				}
				// get hostnet
				if(getNthValueSafe(1, valBuff, ',', old_hostnet, sizeof(old_hostnet)) == -1 )
				{ 
					continue;
				}
				// get netmask
				if( getNthValueSafe(2, valBuff, ',', old_netmask, sizeof(old_netmask)) == -1 )
				{ 
					continue;
				}

				// get gateway
				if( getNthValueSafe(3, valBuff, ',', old_gateway, sizeof(old_gateway)) == -1 )
				{ 
					continue;
				}

				// get interface
				if( getNthValueSafe(4, valBuff, ',', old_interface, sizeof(old_interface)) == -1 )
				{ 
					continue;
				}

				// get true_interface
				if( getNthValueSafe(5, valBuff, ',', old_trueintf, sizeof(old_trueintf)) == -1 )
				{ 
					continue;
				}

				if(!strcmp(dest,old_dest) && !strcmp(netmask,old_netmask) && !strcmp(hostnet,old_hostnet))
				{
					printf("Already have this configuration!\n");
					return;
				}
			}
			else
			{
				snprintf(cmd, sizeof(cmd), "RoutingRules%d", i);
				sprintf(tmp, "%s,%s,%s,%s,%s,%s", dest, hostnet, netmask, gateway, interface, true_interface);
				CFG_set_by_name(cmd, tmp);
				writeParameters("/tmp/.apcfg","w+",0);
				writeParameters(NVRAM,"w+", NVRAM_OFFSET);
				break;
			}	
		}
	}
	else
	{
		printf("Add routing failed: %s <br>", result);
		return;
	}
	
	return;
}

void removeRoutingRule(char *dest, char *netmask, char *hostnet)
{
	char cmd[1024];
	
	strcpy(cmd, "route del ");	
	// host or net?
	if(!strcmp(hostnet,"net"))
	{
		strcat(cmd, " -net ");
	}
	else
	{
		strcat(cmd, " -host ");
	}
	// destination
	strcat(cmd, dest);
	// netmask
	if(!strcmp(hostnet,"net"))
	{
		sprintf(cmd, "%s netmask %s", cmd, netmask);
	}

	system(cmd);
	return;
}

int DelRouting(void)
{
	int  i, rule_count;
	int  j, delete_count;
	char cmd[200], rule[200], buf[100], val[200];
	int  flag = 0;   //mark if some rules to delete
	char delRule[MaxRulesCount][32] = {0};
	int  delRuleNum = 0;
	char dest[20] = {0}, hostnet[20] = {0}, netmask[20] = {0};
		
	for(i = 0, rule_count = 0; i < MaxRulesCount; i++)
	{
		memset(cmd, 0, sizeof(cmd));
		memset(rule, 0, sizeof(rule));			

		sprintf(cmd, "RoutingRules%d", i);
		CFG_get_by_name(cmd,rule);
		if(!rule || !strlen(rule))
		{	
			continue;
	    }
	    else
	    {
			rule_count++;
	    }
    }
    if(rule_count == 0)
    {
		// there is no RoutingRules        
        return ;
	}				

	for(i = 0; i < rule_count; i++)
	{
		snprintf(buf, sizeof(buf), "delRouting%d", i);
		CFG_get_by_name(buf,val);

		if(!val || !strlen(val))
		{
			continue;
		}
		else //delete this record
		{
			if (0 == flag)
			{
				flag = 1;
			}
			
			for(j = 0, delete_count = 0; j < MaxRulesCount; j++)
			{
				memset(cmd, 0, sizeof(cmd));
				memset(rule, 0, sizeof(rule));
				
				snprintf(cmd,sizeof(cmd),"RoutingRules%d",j);
				CFG_get_by_name(cmd, rule);
				if( !rule || !strlen(rule))
				{
					continue;
				}
				else
				{
					delete_count++;									
					if( i == (delete_count-1) )
					{	
						strcpy(delRule[delRuleNum++], cmd);
						CFG_remove_by_name(buf);
						
						if( getNthValueSafe(0, rule, ',', dest, sizeof(dest)) == -1 )
						{ 
							continue;
						}
						//hostnet
						if( getNthValueSafe(1, rule, ',', hostnet, sizeof(hostnet)) == -1 )
						{ 
							continue;
						}
						
						// get netmask
						if( getNthValueSafe(2, rule, ',', netmask, sizeof(netmask)) == -1 )
						{ 
							continue;
						}

						removeRoutingRule(dest, netmask, hostnet);
						break;
					}
				}				
			}
		}		
	}

    if(0 == flag)
    {
        printf("You didn't select any rules to delete.<br>\n");
        return;
    }
	
	for (j = 0; j < delRuleNum; j++)
	{
		CFG_remove_by_name(delRule[j]);
	}
	
	writeParameters("/tmp/.apcfg","w+",0);
	writeParameters(NVRAM,"w+", NVRAM_OFFSET);

	return;
}

void ShowRouting(void)
{
	int  i   = 0;
	int  index = 0;
	char dest[20]={0};
	char hostnet[5]={0};
	char netmask[20]={0};
	char gateway[20]={0};
	char interface[10]={0};
	char true_interface[20]={0};
	char custom_interface[20]={0};
	char comment[20]={0};
	char name[30]={0};
	char valBuff[70] = {0};
	for(i = 0;  i < MaxRulesCount; i++)
	{
		snprintf(name, sizeof(name),"RoutingRules%d", i);
		CFG_get_by_name(name,valBuff);
		if(!valBuff || !strlen(valBuff))
		{
			continue;
	    }
		else
		{
			//打印
			// get dest
			if( getNthValueSafe(0, valBuff, ',', dest, sizeof(dest)) == -1 )
			{ 
				continue;
			}
			
			// get hostnet
			if( getNthValueSafe(1, valBuff, ',', hostnet, sizeof(hostnet)) == -1 )
			{ 
				continue;
			}
			
			// get netmask
			if( getNthValueSafe(2, valBuff, ',', netmask, sizeof(netmask)) == -1 )
			{ 
				continue;
			}
			
			// get gateway
			if( getNthValueSafe(3, valBuff, ',', gateway, sizeof(gateway)) == -1 )
			{ 
				continue;
			}
			
			// get interface
			if( getNthValueSafe(4, valBuff, ',', interface, sizeof(interface)) == -1 )
			{ 
				continue;
			}
			
			// get true_interface
			if( getNthValueSafe(5, valBuff, ',', true_interface, sizeof(true_interface)) == -1 )
			{ 
				continue;
			}

			printf("<tr>");
			printf("<td class=headind> %d&nbsp; <input type=\"checkbox\" name=\"delRouting%d\"> </td>", index++,i);
			printf("<td class=headind align=\"center\">%s</td>", dest);
			printf("<td class=headind align=\"center\">%s</td>", hostnet);			
			printf("<td class=headind align=\"center\">%s</td>", netmask);
			printf("<td class=headind align=\"center\">%s</td>", gateway);
			printf("<td class=headind align=\"center\">%s(%s)</td>",interface,true_interface);
			printf("</tr>");
		 }
	}
	printf("</table>");
	printf("</form>");
	printf("</body></html>");
	return;	
}
/**************************************************************************/	
/***********************end for Static Routing Settings********************/
/************************add pppoe client *********************************/
int getLineNum(FILE *fp)
{
	int  line = 0;
	char c, cb ='\0';
	
	while((c = getc(fp)) != EOF)
	{
		if(c == '\n')
		{
			line++;
		}
		cb = c;
	}
	if(cb != '\n')
	{
		line++;
	}
	
	return line;
}

int AddPPPoEClient()
{
	char user[70] = {0};
	char auth[70] = {0};
	char cmd[200] = {0};
	char num[70]  = {0};
	char psw[70]  = {0};
	FILE *fp      = NULL;
	char *p       = NULL;
	int  n        = 0;  //record number of users(+2)
	int  len      = 0;
	
	CFG_get_by_name("ClientMaxNum", num);
	CFG_get_by_name("ClientUserName", user);
	if(!user || !strlen(user))
	{
		printf("get pppoe client ClientUserName failed!\n");
		return 1;		
	}
	
	len = strlen(user);
	CFG_get_by_name("PswAuthen", auth);
	if (!auth || !strlen(auth))
	{
		printf("get PswAuthen failed!\n");
		return 1;
	}
	else if(auth == "PAP")
	{
		//remove extra blank lines	
		system("sed -i \'/^s*$/d\' /etc/ppp/pap-secrets");	
		fp = fopen("/etc/ppp/pap-secrets", "r");
		if(fp == NULL)
		{
			printf("Open pap-secrets failed!\n");
			return 1;
		}
		
		//get file line numbers
		n = getLineNum(fp); 
		if(n >= atoi(num)+2)
		{
			printf("Exceeds the max allowed users!\n");
			return 1;
		}
		
		//relocate the file pointer
		fseek(fp, 0L, SEEK_SET);   
		while(!feof(fp))
		{
			if(fgets(cmd,sizeof(cmd),fp))
			{
				p = strstr(cmd, user);
				if((NULL != p) && (*(p-1) == ' ') && (*(p+len) == ' '))
				{
					fclose(fp);
					printf("Already has this user!\n");
					return 1;
				}
			}
		}
		
        // if no this user, add user
		CFG_get_by_name("ClientPsw",psw);
		sprintf(cmd, "echo \"  %s   *   %s   *\" >> /etc/ppp/pap-secrets", user, psw);
		system(cmd);
	}
	else
	{
		//remove extra blank lines
		system("sed -i \'/^s*$/d\' /etc/ppp/chap-secrets");
		
		fp = fopen("/etc/ppp/chap-secrets", "r");
		if(fp == NULL)
		{
			printf("Open chap-secrets failed!\n");
			return 1;		
		}
		
		n = getLineNum(fp);
		if(n >= atoi(num) + 2)
		{
			printf("Exceeds the max allowed users!\n");
			return 1;
		}
		
		//relocate the file pointer
		fseek(fp, 0L, SEEK_SET);
		while(!feof(fp))
		{
			if( NULL != fgets(cmd,sizeof(cmd),fp))
			{
				p = strstr(cmd, user);
				if((NULL != p) && (*(p-1) == ' ') && (*(p+len) == ' '))
				{
					fclose(fp);
					printf("Already has this user!\n");
					return 1;
				}
			}
		}
		
		// if no this user, add user
		CFG_get_by_name("ClientPsw",psw);
		sprintf(cmd,"echo \"  %s   *   %s   *\" >> /etc/ppp/chap-secrets", user, psw);
		system(cmd);
	}

	fclose(fp);
	return 0;
}

int ModifyPPPoEClient()
{
	char user[70] = {0};
	char auth[70] = {0};
	char cmd[200] = {0};
	char psw[70]  = {0};
	char *p       = NULL;
	FILE *fp      = NULL;
	int  len      = 0;
		
	CFG_get_by_name("ClientUserName", user);
	if(!user || !strlen(user))
	{
		printf("get pppoe client ClientUserName failed!\n");
		return 1;		
	}
	
	len = strlen(user);
	CFG_get_by_name("PswAuthen", auth);
	if (!auth || !strlen(auth))
	{
		printf("get PswAuthen failed!\n");
		return 1;
	}
	else if(auth == "PAP")
	{
		fp = fopen("/etc/ppp/pap-secrets", "r");
		if(fp == NULL)
		{
			printf("Open pap-secrets failed!\n");
			return 1;
		}
		while(!feof(fp))
		{
			if(fgets(cmd,sizeof(cmd),fp))
			{
				p = strstr(cmd, user);
				if((NULL != p) && (*(p-1) == ' ') && (*(p+len) == ' '))
				{
					CFG_get_by_name("ClientPsw",psw);
					sprintf(cmd, "sed -i \'s/%s   .*$/%s   *   %s   */\' /etc/ppp/pap-secrets", user, user, psw);
					system(cmd);
					fclose(fp);
					return 0;
				}
			}
		}
		fclose(fp);
		printf("Don't has this user!\n");
		return 1;
	}
	else
	{
		fp = fopen("/etc/ppp/chap-secrets", "r");
		if(fp == NULL)
		{
			printf("Open chap-secrets failed!\n");
			return 1;		
		}
		while(!feof(fp))
		{
			if(fgets(cmd,sizeof(cmd),fp))
			{
				p = strstr(cmd, user);
				if((NULL != p) && (*(p-1) == ' ') && (*(p+len) == ' '))
				{
					CFG_get_by_name("ClientPsw",psw);
					sprintf(cmd, "sed -i \'s/%s   .*$/%s   *   %s   */\' /etc/ppp/chap-secrets", user, user, psw);
					system(cmd);
					fclose(fp);
					return 0;
				}
			}
		}
		
		fclose(fp);
		printf("Don't has this user!\n");
		return 1;		
	}
	
	return 0;
}
/**************************end for pppoe client *****************************/

/************************* Modify web style *****************************/

int ChangeStyle()
{
	char val[70]  = {0};
	
	CFG_get_by_name("StyleSelection", val);
	if(!val || !strlen(val))
	{
		printf("get web style failed!\n");
		return 1;		
	}
	if(strcmp(val,"default") == 0 || strcmp(val,"默认风格") == 0)
	{
		system("cp /usr/www/style/default/styleSheet.css  /usr/www/");
	}
	else if (strcmp(val,"style2") == 0 || strcmp(val,"风格2") == 0)
	{
		system("cp /usr/www/style/style2/styleSheet.css  /usr/www/");
	}
    else if (strcmp(val,"style3") == 0 || strcmp(val,"风格3") == 0)
	{
		system("cp /usr/www/style/style3/styleSheet.css  /usr/www/");
	}
	else
	{
		system("cp /usr/www/style/default/styleSheet.css  /usr/www/");
		printf("Get web style error!");
		return 1;
	}
	printf("<script>");
	printf("parent.selector.location.reload();");
	printf("parent.title.location.reload();");
	printf("</script>");
	return 0;
}

/******************************* Add DMZ ***********************************/
void iptablesDMZFlush()
{
	char cmd[120] = {0};
    snprintf(cmd, 120, "iptables -t nat -F %s ",DMZ_CHAIN);
    system(cmd);
}

void makeDMZRule(char *buf, int len, char *wan_name, char *ip_address)
{
	/* iptables -t nat -A PREROUTING -i br0 -j DNAT --to 5.6.7.8 */
	snprintf(buf, len, "iptables -t nat -A %s -j DNAT -i %s -p udp --dport ! %d --to %s", DMZ_CHAIN, wan_name, DMZ_PORT, ip_address);
	system(buf);
	snprintf(buf, len, "iptables -t nat -A %s -j DNAT -i %s -p tcp --dport ! %d --to %s", DMZ_CHAIN, wan_name, DMZ_PORT, ip_address);
	system(buf);
}

void iptablesDMZRun()
{
    char pfe[70]   = {0};
    char ip[32]    = {0};
    char cmd[1024] = {0};
    
	CFG_get_by_name("DMZEnabled", pfe);
	if(!pfe || !atoi(pfe))
	{
        return;
	}    
    CFG_get_by_name("DMZIPAddress", ip);
	if(!ip || !atoi(ip))
	{
        return;
	}    
    
    makeDMZRule(cmd, sizeof(cmd), getWanIfName(), ip);
	return;
}

void iptablesDMZ()
{
	iptablesDMZFlush();
	iptablesDMZRun();
    writeParameters(NVRAM,"w+", NVRAM_OFFSET);
    writeParameters("/tmp/.apcfg","w+",0);
}

/******************************* End DMZ ***********************************/

/************************show dhcp client info******************************/
void showDhcpCliinfo()
{
	char time[64]  = {0};
	char mac[64]   = {0};
	char ip[64]    = {0};
	char host[64]  = {0};
	char buf[1024] = {0};
	char val[254]  = {0};
	FILE *fp       = NULL;
	char *p        = NULL;
	
	CFG_get_by_name("UDHCPD", val);

	if(strcmp(val,"server") == 0)
	{
		fp = fopen("/tmp/dnsmasq.leases","r");
		if(fp == NULL)
		{
			printf("Can't open dnsmasq.leases!");
		}
		fread(buf,sizeof(buf),1,fp);
	
		if(strlen(buf) != 0 )
		{
			p = buf;
			sscanf(p,"%s",time);

			p += strlen(time) + 1;
			sscanf(p,"%s",mac);

			p += strlen(mac) + 1;
			sscanf(p,"%s",ip);

			p += strlen(ip) + 1;
			sscanf(p,"%s",host);

			printf("<tr>\n");
			printf("<td> %s </td>\n", mac);	
			printf("<td> %s </td>\n",ip);
			printf("<td> %s </host>\n",host);			
			printf("</tr>\n");
		}
	}
	printf("</table>\n");
	printf("</form>\n");
	printf("</table>\n");	
	printf("</body></html>\n");
	
	return;
}

void systemlogclear(void)
{
	system("killall -q klogd");
	system("killall -q syslogd");
	system("syslogd -C8 1>/dev/null 2>&1");
	system("klogd 1>/dev/null 2>&1");
	
	return;
}

int getIfNetmask(char *ifname, char *if_net)
{
	struct ifreq ifr;
	int skfd = 0;

	if((skfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
	{
		printf("getIfNetmask: open socket error");
		return -1;
	}

	strncpy(ifr.ifr_name, ifname, IF_NAMESIZE);
	if(ioctl(skfd, SIOCGIFNETMASK, &ifr) < 0) 
	{
		return -1;
	}
	strcpy(if_net, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
	close(skfd);
	return 0;
}

int getIfIp(char *ifname, char *if_addr)
{
	struct ifreq ifr;
	int skfd = 0;

	if((skfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
	{
		printf("getIfIp: open socket error");
		return -1;
	}

	strncpy(ifr.ifr_name, ifname, IF_NAMESIZE);
	if (ioctl(skfd, SIOCGIFADDR, &ifr) < 0) 
	{
		return -1;
	}
	strcpy(if_addr, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));

	close(skfd);
	return 0;
}

int netmask_aton(const char *ip)
{
	int i, a[4], result = 0;
	sscanf(ip, "%d.%d.%d.%d", &a[0], &a[1], &a[2], &a[3]);
	for(i=0; i<4; i++)
	{
		if(a[i] == 255)
		{
			result += 8;
			continue;
		}
		if(a[i] == 254)
		{
			result += 7;
		}
		if(a[i] == 252)
		{
			result += 6;
		}
		if(a[i] == 248)
		{
			result += 5;
		}
		if(a[i] == 240)
		{
			result += 4;
		}
		if(a[i] == 224)
		{
			result += 3;
		}
		if(a[i] == 192)
		{
			result += 2;
		}
		if(a[i] == 128)
		{
			result += 1;
		}
		break;
	}
	
	return result;
}

void ripdRestart(void)
{
	char wan_ip[16] = {0}, wan_name[16] = {0}, wan_mask[16] = {0};
	char lan_ip[16] = {0}, lan_name[16] = {0}, lan_mask[16] = {0};	
	char opmode[20] = {0};
	char rip[16] = {0};
	char buf[100];
	
	system("killall -q ripd");

	CFG_get_by_name("NETWORK_MODE", opmode);
	if(!opmode || !strcmp(opmode, "Bridge"))
	{
		return;
	}

	CFG_get_by_name("RIPSelect", rip);
	if(!rip || !strlen(rip))
	{
		return;
	}

	strcpy(wan_name, getWanIfName());
	if(getIfIp(wan_name, wan_ip) != -1)
	{
		if(getIfNetmask(wan_name, wan_mask) != -1)
		{
			//delete line include network 
			system("sed -i \'/network.*$/d\' /etc/ripd.conf");
			//delete blank line
			system("sed -i \'/^\\s*$/d\' /etc/ripd.conf");
			
			memset(buf, 0, sizeof(buf));
			sprintf(buf,"echo \"network %s/%d\" >> /etc/ripd.conf", wan_ip, netmask_aton(wan_mask));
			system(buf);

			memset(buf, 0, sizeof(buf));
			sprintf(buf, "echo \"network %s\" >> /etc/ripd.conf", wan_name);
			system(buf);
		}
		else
		{
			printf("ripdRestart(): The WAN IP is still undeterminated...\n");
		}
	}

	strcpy(lan_name, getLanIfName());
	if(getIfIp(lan_name, lan_ip) != -1)
	{
		if(getIfNetmask(lan_name, lan_mask) != -1)
		{
			memset(buf, 0, sizeof(buf));
			sprintf(buf,"echo \"network %s/%d\" >> /etc/ripd.conf", lan_ip, netmask_aton(lan_mask));
			system(buf);

			memset(buf, 0, sizeof(buf));
			sprintf(buf, "echo \"network %s\" >> /etc/ripd.conf", lan_name);
			system(buf);
		}
	}
	
	system("ripd -f /etc/ripd.conf -d");	
	return;
}

void zebraRestart(void)
{
	char opmode[20] = {0};
	char rip[16] = {0};
	
	system("killall -q zebra");
	
	CFG_get_by_name("NETWORK_MODE", opmode);
	if(!opmode || !strcmp(opmode, "Bridge"))
	{
		return;
	}

	CFG_get_by_name("RIPSelect", rip);
	if(!rip || !strlen(rip))
	{
		return;
	}
	
	system("zebra -d -f /etc/zebra.conf");
	return;
}

void dynamicRouting(void)
{
	char opmode[20] = {0};
	char rip[16] = {0};
	
	CFG_get_by_name("NETWORK_MODE", opmode);
	if(!opmode || !strcmp(opmode, "Bridge"))
	{
		return;
	}

	CFG_get_by_name("RIPSelect", rip);
	if(!rip || !strlen(rip))
	{
		return;
	}

	if(!strcmp(rip, "0"))
	{
		system("killall -q ripd");
		system("killall -q zebra");
	}
	else if(!strcmp(rip, "1"))
	{
		zebraRestart();
		ripdRestart();
	}
	
	writeParameters(NVRAM,"w+", NVRAM_OFFSET);
	writeParameters("/tmp/.apcfg","w+",0);
	return;
}

/*****************************************************************************
**
** /brief Main
**
** This program will read parameterized HTML files and insert the proper
** strings from the parameter store -- either in flash or temp storage.
**
** The specific page to process will be determined by the actual command
** name returned in argv[0].  Each page name will be linked to the executable
** in much the same manner as busybox does it's commands.  This will require
** that each page link be located in the cgi-bin directory since they are ALL
** being processed as cgi scripts by the httpd daemon.  Actual pages will be
** located in the /usr/www directory.
**
** Other functions are provided to support the command line processing.
**
** Options: -a  Add a parameter/value pair to the cache
**              # cgiMain -a SSID=MySSID
**
**          -r  Remove a parameter from the parameter cache
**              # cgiMain -r AP_THIS_PARAM
**
**          -c  Commit parameter cache to flash
**              # cgiMain -c
**
**          -e  Print the export list for use in scripts
**              `cgiMain -e`
**
**          -i  Invalidate the parameter cache by re-reading the flash
**              values and overriding the parameter cache.  NOTE: this
**              will loose any changes made to the parameter cache.
**              # cgiMain -i
**
**          -s  Print the contents of the database without translation
**
**          -t  Translate File.  This will take the indicated file and
**              insert parameter data as marked by the ~ markers in the
**              file.  Uses the same rules as the HTML files.  Normal
**              output is to stdout, can be redirected to another file.
**              if the third parameter is specified, it is assumed to be
**              the interface name.
**
**              # cgiMain -t wpapsk.conf ath0 > /tmp/secvap.conf
**              # cgiMain -t2 wpapsk.conf > /tmp/secvap2.conf
*/

int main(int argc,char **argv)
{
    char            Page[64];
    char            Name[32];
    char            Value[70];
    char            valBuff[128];
    char            *outPtr;
    int             i;
    int             j;
    int             ret=0;
    int             holdOutput;
    unsigned long   syncWord;

    char            *nextField;
    char            *update;
    FILE            *f;
	char            cmd[70];

    /*
    ** Code Begins.
    ** Zero out the config structure, and read the parameter cache
    ** (or flash, depending on command)
    */

    errOut = fopen("/dev/ttyS0","w");

    memset(&config,0,sizeof(config));

    f = fopen("/tmp/.apcfg","r");

    if ( !f )
    {
        /*
        ** We need to read the parameter data starting at 32K into the calibration
        ** sector (or) the NVRAM sector.  This is mapped to /dev/nvram or /dev/caldata, so we simply open 
        ** that device and read until we hit the EOL
        */

        f = fopen( NVRAM, "r" );

        if (!f)
        {
            printf("ERROR:  %s not defined on this device\n", NVRAM);
            printf("ERROR:  Cannot store data in flash!!!!\n");
            exit(-1);
        }

        fseek(f, NVRAM_OFFSET, SEEK_SET);
    }

    /*
    ** At this point the file is either open or not.  If it is, read the 
    ** parameters as require
    */

    if ( f )
    {
        fillParamStruct(f);
        fclose(f);
    }

    /*
    ** Now we look for options.
    ** -t means translate the indicated file
    */

    if(argc > 1)
    {
#ifndef ATH_SINGLE_CFG
        if(!strncmp(argv[1],"-t",2))
        {
            /*
            ** Translate file
            ** read the file, then output the translated version
            */
            parameterIndex = 0;

            if(argv[1][2] != 0)
                parameterIndex = argv[1][2] - 0x30;

            if(parameterIndex > 16)
                parameterIndex = 0;

			/*
			** Get the radio ID, if it exists
			*/

			radioIndex = getRadioID(parameterIndex);
			if(radioIndex == -1)
				radioIndex = 0;	/* set back to default */

            /*
            ** Input the "extra" parameters that may be included
            */

            for(i=3,numAdditionalParams=0;i<argc;i++)
            {
                strcpy(additionalParams[numAdditionalParams++],argv[i]);
            }

            /*
            ** Now, perform the translation
            */

            ModeFlag = 1;
            translateFile(argv[2]);
            exit(0);
        }
        else if(!strncmp(argv[1],"-a",2))
#else
        if (!strncmp(argv[1],"-a",2))
#endif
        {
            char    *vname;
            char    *vval;

            /*
            ** Add a parameter.  Argv[2] should contain the parameter=value string.
            ** Do NOT use extractParam in this case, we need to directly enter
            ** the value.
            */

            if(argc < 3)
            {
                modePrintf("Invalid argument");
                exit(-1);
            }

            ModeFlag = 1;
            vname = argv[2];

            if(vval=strchr(argv[2],'='))
                *vval++ = 0;
            else
            {
                modePrintf("Mal formed string %s",argv[2]);
                exit(-1);
            }

            /*
            ** If setting fails, return a -1 (for scripts)
            */

            if( CFG_set_by_name(vname,vval) )
                exit(-1);

            writeParameters("/tmp/.apcfg","w+",0);
            exit(0);
        }
        else if(!strncmp(argv[1],"-c",2))
        {
            /*
            ** Write the parameter structure to the flash.  This is
            ** the "commit" function
            */
#ifdef ATH_SINGLE_CFG
            athcfg_set_default_config_values();
#endif
            writeParameters(NVRAM,"w+", NVRAM_OFFSET);
            writeParameters("/tmp/.apcfg","w+",0);
            exit(0);
        }
        else if(!strncmp(argv[1],"-r",2))
        {
            /*
            ** Remove the parameter from the cache.  This will write the
            ** cache, but not write to the flash.  Explicit commit required
            ** to write to flash.
            */

            CFG_remove_by_name(argv[2]);
			FactoryDefault = 0;
            writeParameters("/tmp/.apcfg","w+",0);
            exit(0);
        }
        else if(!strncmp(argv[1],"-h",2))
        {
            /*
            ** We want to check argv[2] to determine if it's a properly formattedRemove the parameter from the cache.  This will write the
            ** cache, but not write to the flash.  Explicit commit required
            ** to write to flash.
            */

            exit(isKeyHex(argv[2],atoi(argv[3])));
        }
        else if(!strncmp(argv[1],"-e",2))
        {
            /*
            ** Export the variables
            ** This is used as part of a shell script to "export" the variables
            ** to the environment
            */
#ifdef ATH_SINGLE_CFG
            athcfg_set_default_config_values();
#endif /* ATH_SINGLE_CFG */

            for(i=0;i<config.numParams;i++)
            {
                /*
                ** Check for certain variables -- mostly key values -- that
                ** we don't want to export due to the "funnies" in their string
                ** values.  They will still be included in the database, but the
                ** will not be exported in the script files.
                **
                ** Unfortunately, SSID is a parameter that cannot be ignored, but
                ** can have various issues with shell special characters.  This will
                ** be a limitation on the SSID string that cannot be avoided
                */

                if(!strncmp(config.Param[i].Name,"PSK_KEY",7))
                    continue;

                /*
                ** We can export this variable.
                */

                if(shellEncode(config.Param[i].Val,valBuff))
                    printf("export %s=\"%s\"\n",config.Param[i].Name,valBuff);
                else
                    printf("export %s=%s\n",config.Param[i].Name,valBuff);

            }
            exit(0);
        }
        else if(!strncmp(argv[1],"-s",2))
        {
            /*
            ** Show the variables
            ** This is used as a debug method to dump the variables to the output.
            ** This dumps ALL variables
            */
            for(i=0;i<config.numParams;i++)
            {
                printf("%s:=%s\n",config.Param[i].Name,config.Param[i].Val);
            }
            exit(0);
        }
        else if(!strncmp(argv[1],"-x",2))
        {
            /*
            ** Erase all parameters in flash and cache.
            ** This is the equivalent of a reset command.
            */

            memset(&config,0,sizeof(config));
            FactoryDefault = 1;
            writeParameters(NVRAM,"w+", NVRAM_OFFSET);
            writeParameters("/tmp/.apcfg","w+",0);
#ifndef ATH_SINGLE_CFG
            /*
            ** Now, remove WPS files and execute the apcfg script to set the
            ** values.  This is required for determination of "factory default" state
            */

            Execute_cmd("rm -rf /tmp/*.conf*;/etc/ath/apcfg", rspBuff);
#else
            system("rm -rf /tmp/WSC*.conf");
#endif /* #ifndef ATH_SINGLE_CFG */
            exit(0);
        }
        else if(!strncmp(argv[1],"-v",2))
        {
            /*
            ** Get the VALUE of the parameter without any processing or other
            ** stuff.  The form of this command is
            ** cfg -v NAME index1 index2
            **
            ** where NAME is the UNINDEXED name (such as AP_SSID), and index1 (index2)
            ** are indexes added to the name to get the fully qualified value.  It is
            ** intended that this can be used in the form
            &&
            ** `cfg -v AP_SSID $INDEX`
            **
            ** to allow for looping through parameters.  This is similiar to the -t
            ** function, but is for individual parameters.
            */

            if(argv[1][2] == '0')
                ModeFlag = 0;
            else
                ModeFlag = 1;

            if(argc > 4)
                sprintf(Name,"%s_%s_%s",argv[2],argv[3],argv[4]);
            else if(argc > 3)
                sprintf(Name,"%s_%s",argv[2],argv[3]);
            else
                sprintf(Name,"%s",argv[2]);

            CFG_get_by_name(Name,Value);
            printf("%s",Value);
            exit(0);
        }
#ifdef CONFIG_ATHR_ETH_EXTENSION
        else if (!strcmp(argv[1],"lanctl"))
        {
            ethlanctl_process_opt(argc,argv);
            exit(0);
        }
#endif /* #ifdef CONFIG_ATHR_ETH_EXTENSION */
#ifdef ATH_SINGLE_CFG
        athcfg_process_commandline(argc, argv);
#endif /* #ifdef ATH_SINGLE_CFG */
    }
#ifdef ATH_SINGLE_CFG
    usage(argv[0]);
#else
    /*
    ** Otherwise, this is processing for an HTML page
    ** Zero out the config structure, and get the page name
    */

    strcpy(Page,"../");
    strcat(Page,argv[0]);
    strcat(Page,".html");

    /*
    ** Now to get the environment data.
    ** We parse the input until all parameters are inserted.  If we see a reset, commit,
    ** or accept/submit label, we do the appropriate action
    */

    /* if every thing fine, make sure we add 200 OK with proper content type.
     * At this point we do not know Content-Length, but http 1.0 restricts to
     * add content length so just tell the browser read until connection close
     */
    printf("HTTP/1.0 200 OK\r\n");
    printf("Content-type: text/html\r\n");
    printf("Connection: close\r\n");
    printf("\r\n");
    printf("\r\n");

    /*
    ** This method allows processing of either "get" or "post" methods.  Post
    ** overrides Get.
    */

    nextField = getenv("CONTENT_LENGTH");

    if (nextField == NULL)
    {
        sprintf(valBuff,"?%s",getenv("QUERY_STRING"));
        nextField = valBuff;
    }

    if(nextField != NULL)
    {
        if(*nextField != '?')
        {
            j = atoi(nextField);

	    if((j+3) > 1024)
		j = 1024-3;
            memset(opBuff,0,1024);
            fgets(opBuff,j+3,stdin);
	    opBuff[j] = '\n';
	    opBuff[j+1] = '\0';
            nextField = opBuff;
        }

        /*
        ** Check for the reboot button
        ** If hit, we die gloriously
        */

        update = strstr(nextField,"RebootButton");
        if(update)
        {
            reboot(RB_AUTOBOOT);
        }

        /*
        ** We want to read all parameters regardless, and update
        ** what was read from tmp/flash.  If the commit parameter
        ** is set, we will write to flash
        */

        while(nextField)
        {
            nextField = extractParam(nextField,Name,valBuff);
            unencode(valBuff,Value);

            if(!strcmp("INDEX",Name))
            {
               parameterIndex = atoi(Value);
               sprintf(Value,"%d",parameterIndex);
            }
            CFG_set_by_name(Name,Value);
        }
    if(parameterIndex)
		radioIndex = getRadioID(parameterIndex);

    }

    /*
    ** use the "translate file" function to translate the file, inserting the
    ** special strings as required.
    */

    if(translateFile(Page) < 0)
    {
        printf("Content-Type:text/html\n\n");
        printf("<HTML><HEAD>\r\n");
        printf("<LINK REL=\"stylesheet\" href=\"../styleSheet.css\" type=\"text/css\">");
        printf("</head><body>");
        printf("Page %s Not Found",Page);
        printf("</body></html>");
        exit(1);
    }

	sync();

	/*Add for NTP processing by wcy: 2011-03-08*/
    if(strcmp(CFG_get_by_name("NTPSetTime",valBuff),"Sync with host") == 0 || strcmp(CFG_get_by_name("NTPSetTime",valBuff),"主机同步") == 0)
    {
   		CFG_get_by_name("NTPTime",valBuff);
   		if(valBuff != "")
   		{
   			snprintf(cmd, sizeof(cmd), "date -s %s", valBuff);
   			Execute_cmd(cmd, rspBuff);
   		}
   		else
   		{
   			printf("Error: set current time failed!\n");
   		}
    }
    
    if(strcmp(CFG_get_by_name("NTPApply",valBuff),"Apply") == 0 || strcmp(CFG_get_by_name("NTPApply",valBuff),"确定") == 0)
    {
        writeParameters("/tmp/.apcfg","w+",0);
        system("/etc/rc.d/ntp.sh"); /*Run ntp.sh script --- ntpclient*/
    }
	/*end for NTP processing*/

    /*
    ** Now, look for the update and/or commit strings to send to either
    ** the temp file or the flash file
    */

    if(strcmp(CFG_get_by_name("COMMIT",valBuff),"Commit") == 0 || strcmp(CFG_get_by_name("COMMIT",valBuff),"提交") == 0)
    {
        writeParameters(NVRAM,"w+", NVRAM_OFFSET);
        writeParameters("/tmp/.apcfg","w+",0);
    }

    if(strcmp(CFG_get_by_name("UPDATE",valBuff),"Update") == 0 || strcmp(CFG_get_by_name("UPDATE",valBuff),"更新") == 0 )
    {
        writeParameters("/tmp/.apcfg","w+",0);
    }

    if(strcmp(CFG_get_by_name("StopButton",valBuff),"Stop") == 0 || strcmp(CFG_get_by_name("StopButton",valBuff),"停止") == 0 )
    {
        Execute_cmd("apdown > /dev/null 2>&1", rspBuff);
    }

    if(strcmp(CFG_get_by_name("StartButton",valBuff),"Start") == 0 || strcmp(CFG_get_by_name("StartButton",valBuff),"启动") == 0 )
    {
        Execute_cmd("apup > /dev/null 2>&1", rspBuff);
    }

    if(strcmp(CFG_get_by_name("FactoryResetButton",valBuff),"FactoryReset") == 0 || strcmp(CFG_get_by_name("FactoryResetButton",valBuff),"恢复出厂设置") == 0 )
    {
        Execute_cmd("cfg -x", rspBuff);
        sleep(1);
        reboot(RB_AUTOBOOT);
    }

    if(strcmp(CFG_get_by_name("StartPINMethod",valBuff),"StartPINMethod") == 0 )
    {
        /* extract the enrollee key and pass it over to the command 
         */
        char cmd[256]={0};

        sprintf(cmd, "wpatalk -v ath%c 'configthem pin=%s'", '0'+(parameterIndex-1), CFG_get_by_name("AP_ENROLLEE", valBuff));
        Execute_cmd(cmd, rspBuff);
    }
    if(strcmp(CFG_get_by_name("StartPBC",valBuff),"StartPBC") == 0 )
    {
        /* extract the enrollee key and pass it over to the command 
         */
        char cmd[256]={0};
        sprintf(cmd, "wpatalk -v ath0 configthem");
        Execute_cmd(cmd, rspBuff);
    }
    if(strcmp(CFG_get_by_name("StartGPIO",valBuff),"Start") == 0 || strcmp(CFG_get_by_name("StartGPIO",valBuff),"启动") == 0)
    {
		Execute_cmd("gpio > /dev/null 2>&1", rspBuff);
    }
    if(strcmp(CFG_get_by_name("StartFan",valBuff),"StartFan") == 0 || strcmp(CFG_get_by_name("StartFan",valBuff),"启动") == 0)
    {
		Execute_cmd("fanup > /dev/null 2>&1", rspBuff);
    }
#endif /* #ifndef ATH_SINGLE_CFG */


/*************************add for port forwarding**************************/

	//setup iptables chain PORT_FORWARD_CHAIN
		
	if(strcmp(CFG_get_by_name("PortForwardApply",valBuff),"Apply") == 0 || strcmp(CFG_get_by_name("PortForwardApply",valBuff),"确定") == 0 )
	{
		portForward();
	}
	
	if(strcmp(CFG_get_by_name("deleteSelPortForward",valBuff),"Delete Selected") == 0 
	|| strcmp(CFG_get_by_name("deleteSelPortForward",valBuff),"选择删除") == 0)
	{
		portForwardDelete();
	}

	if(strcmp(argv[0],"PortForward") == 0)
	{
		showPortForwardRulesASP();
	}

/*************************end for port forwarding**************************/

/*************************add for port forwarding**************************/

	//setup iptables chain PORT_FORWARD_CHAIN
	if(strcmp(CFG_get_by_name("PortFilterSetApply",valBuff),"Apply") == 0 || strcmp(CFG_get_by_name("PortFilterSetApply",valBuff),"确定") == 0 )
	{
		portFilterSet();
	}
		
	if(strcmp(CFG_get_by_name("PortFilterApply",valBuff),"Apply") == 0 || strcmp(CFG_get_by_name("PortFilterApply",valBuff),"确定") == 0 )
	{
		portFilter();
	}
	
	if(strcmp(CFG_get_by_name("deleteSelPortFilter",valBuff),"Delete Selected") == 0 
	|| strcmp(CFG_get_by_name("deleteSelPortFilter",valBuff),"选择删除") == 0)
	{
		portFilterDelete();
	}

	if(strcmp(argv[0],"PortFilter") == 0)
	{
		showPortFilterRulesASP();
	}

/*************************end for port forwarding**************************/

	if((strcmp(CFG_get_by_name("PPPoECliAdd",valBuff),"Add") == 0) || 
	   (strcmp(CFG_get_by_name("PPPoECliAdd",valBuff),"增加") == 0 ))
	{
		AddPPPoEClient();	
	}
	
	if((strcmp(CFG_get_by_name("PPPoECliModify",valBuff),"Modify") == 0) || 
	   (strcmp(CFG_get_by_name("PPPoECliModify",valBuff),"修改") == 0 ))
	{
		ModifyPPPoEClient();
	}
	
/*********************** Modify the web style ******************************/	
	if((strcmp(CFG_get_by_name("styleApply",valBuff),"Apply") == 0) || 
	   (strcmp(CFG_get_by_name("styleApply",valBuff),"确定") == 0 ))
	{
		ChangeStyle();
	}

/***************************** add for DMZ *********************************/
	if(strcmp(CFG_get_by_name("dmzApply",valBuff),"Apply") == 0 || 
	   strcmp(CFG_get_by_name("dmzApply",valBuff),"确定") == 0 )
	{
		iptablesDMZ();
	}
/*************************add for Static Routing Settings**************************/
	CFG_get_by_name("AddroutingConfigApply", valBuff);
	if(strcmp(valBuff,"Apply") == 0 || strcmp(valBuff,"确定") == 0 )
	{
		AddRouting();
	}
	
	CFG_get_by_name("deleteRoutingConfig", valBuff);
	if(strcmp(valBuff,"Delete") == 0 || strcmp(valBuff,"删除") == 0 )
	{
		DelRouting();
	}
	
	if(strcmp(argv[0],"RoutingConfig") == 0)
	{
		ShowRouting();
	}
/*************************end for Static Routing Settings**************************/
	if(strcmp(argv[0],"DhcpCliInfo") == 0)
	{
		showDhcpCliinfo();
	}
/**************************** system log ******************************************/	
	if(strcmp(CFG_get_by_name("SystemLogClear",valBuff),"Clear") == 0 || 
	   strcmp(CFG_get_by_name("SystemLogClear",valBuff),"清除") == 0 )
	{
		systemlogclear();
	}
/******************************* rip process **************************************/
	CFG_get_by_name("dynamicRoutingApply",valBuff);
	if(strcmp(valBuff,"Apply") == 0 || strcmp(valBuff,"确定") == 0 )
	{
		dynamicRouting();
	}


    exit(0);
}

/********************************** End of Module *****************************/

