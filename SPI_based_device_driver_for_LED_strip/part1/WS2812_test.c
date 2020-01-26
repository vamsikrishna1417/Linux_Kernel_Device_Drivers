#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include "WS2812_spi_device.h"


uint8_t msg_buf[] = {
  RED, GREEN, BLUE, RED, GREEN, BLUE, RED, GREEN, BLUE, RED, GREEN, BLUE, RED, GREEN, BLUE, GREEN,
};

int main(int argc, char *argv[])
{
  int fd;
  char dev_name[50] = "";
  int n;
  uint8_t *msg = (uint8_t*)malloc((BUF_LENGTH*sizeof(uint8_t)));
  int i = 0, j = 0, k = 0;
  int count = 0;
  int none_length = 0;
  int msg_length = 0;
  int max_length = 0;
  int full_length = BUF_LENGTH;

  sprintf (dev_name, "/dev/WS2812");
  /* open device */
  fd = open(dev_name, O_RDWR);
  if (fd < 0){
    printf ("Cannot open device file.! \n");
    return 0;
  }

  /* Resetting all the LEDs before sending the data */
  if (ioctl(fd, RESET) < 0)
  {
    printf ("ERROR IN IOCTL CONFIG PINS \n");
    return 0;
  }

  printf("Enter the number of LEDs to light\n");

  scanf("%d",&n);

  if( n < 1 || n > 16)
  {
    printf("NUMBER OF LEDS ENTERED IS NOT SUPPORTED \n");
    return 0;
  }

  for(k=0; k< 200; k++)
  {
    none_length = count*24;
    for(i=0; i<none_length; i++)
    {
      msg[i%full_length] = ZERO;
    }

    msg_length = 24*(n+count);
    for(i=none_length; i<msg_length; i++)
    {
      msg[i%full_length] = msg_buf[i%full_length];
    }

    max_length = (none_length > full_length) ? none_length : full_length;

    for(j=msg_length; j<max_length; j++)
    {
      msg[j%full_length] = ZERO;
    }

    count++;
    /* sendind the message using write to the spi device */
    if(write(fd, (uint8_t *)msg, BUF_LENGTH) < 0)
    {
      printf ("ERROR IN WRITE TESTAPP \n");
      return 0;
    }
    /* delay of 100ms */
    usleep(100000);
  }

  if (ioctl(fd, RESET) < 0)
  {
    printf ("ERROR IN IOCTL CONFIG PINS \n");
    return 0;
  }

  /* closing the device */
  close(fd);

  return 0;
}
