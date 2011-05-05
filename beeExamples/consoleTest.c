
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Welcome!
 *****
 ***** This file is part of the BeeSoft robot control software.
 ***** The version number is:
 *****
 *****                  v1.3.8 (released Aug 5, 1998)
 *****                  this is not an official BeeSoft version
 *****
 ***** Please refer to bee/src/COPYRIGHT for copyright and liability
 ***** information.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/time.h>

int portfd;

static void
outb (char val, short port)
{
  lseek(portfd, port, SEEK_SET);
  write(portfd, &val, 1);
}

static unsigned char
inb (short port)
{
  unsigned int ret;

  lseek(portfd, port, SEEK_SET);
  read(portfd, &ret, 1);
  return ret;
}


long int mtime(void) {
  struct timeval tv;
  struct timezone tz;

  tz.tz_minuteswest=0;
#ifdef DST_NONE
  tz.tz_dsttime=DST_NONE;  /* glibc-2.0.5c headers in RH-5.0 are broke */
#else
  tz.tz_dsttime=0;
#endif

  gettimeofday(&tv, &tz);

  return ((tv.tv_sec%100000)*1000+tv.tv_usec/1000);
}

int main (int argc, char *argv[]) {
  int i, c;
  unsigned char a, b, olda;

  portfd = open("/dev/port", O_RDWR);

  i=0; olda=0;
  b=0;

  while (1) {
    
    a = (inb(0x379) ^ 0x80) >> 4;
    a = a & 0xF;
    
    if (a!=olda) {
      olda=a;
      b=b^(a|olda);
      printf("%X\n", a);
    }
    
    a=a|(((~b)&3)<<6);

    c=(mtime()%900)/100;

    if (c<2) a|=0x20;
    else if (c<3) a|=0x30;
    else if (c<5) a|=0x10;

    outb(a, 0x378);

    a=(inb(0x37A)&0xF0)|((b&0x0F)^4);
    outb(a, 0x37A);

    usleep (100);
  }
  printf ("\n");
}
