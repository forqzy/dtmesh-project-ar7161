#include <stdio.h>
#include <crypt.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

char * safe_strncpy(char *dst, const char *src, size_t size)
{
	dst[size-1] = '\0';
	return strncpy(dst, src, size-1);
}

char *pw_encrypt(const char *clear, const char *salt)
{
	static char cipher[128];
	char *cp;
	cp = (char *) crypt(clear, salt);
	safe_strncpy(cipher, cp, sizeof(cipher));
	return cipher;
}

int main(int argc,char **argv)
{
	if(argc!=2)
	{
		exit(0);
	}
	
	argv[2] = "$1$";
	printf("%s\n",pw_encrypt(argv[1],argv[2]));

	return 0;
}
