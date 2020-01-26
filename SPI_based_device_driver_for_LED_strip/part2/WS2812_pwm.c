#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void IOSetup(void)
{
  int FdExport, Fd16, Fd76, Fd64;
  int PWMExport, pwm1;

  FdExport = open("/sys/class/gpio/export", O_WRONLY);
  if (FdExport < 0)
  {
    printf("\n gpio export open failed \n");
  }

  if(0< write(FdExport,"16",2))
    printf("\n error FdExport 16 \n");
  if(0< write(FdExport,"76",2))
    printf("\n error FdExport 76 \n");
  if(0< write(FdExport,"64",2))
    printf("\n error FdExport 64 \n");

  close(FdExport);

  PWMExport = open("/sys/class/pwm/pwmchip0/export", O_WRONLY);
  if (PWMExport < 0)
  {
    printf("\n PWMExport open failed \n");
  }

  if(0< write(PWMExport,"1",1))
    printf("\n error PWMExport 64 \n");

  close(PWMExport);

  pwm1 = open("/sys/class/pwm/pwmchip0/pwm1/enable", O_WRONLY);
  if (pwm1 < 0)
  {
    printf("\n pwm1 open failed \n");
  }

  if(0< write(pwm1,"1",1))
    printf("\n error pwm1 enable \n");

  /* Initialize all GPIOs */
  Fd16 = open("/sys/class/gpio/gpio16/direction", O_WRONLY);
  if (Fd16 < 0)
  {
    printf("\n gpio16 direction open failed \n");
  }

  if(0< write(Fd16,"out",3))
    printf("\n error Fd16 \n");

  Fd76 = open("/sys/class/gpio/gpio76/value", O_WRONLY);
  if (Fd76 < 0)
  {
    printf("\n gpio76 value open failed \n");
  }
  Fd64 = open("/sys/class/gpio/gpio64/value", O_WRONLY);
  if (Fd64 < 0)
  {
    printf("\n gpio64 value open failed \n");
  }

  if(0< write(Fd76,"0",1))
    printf("\n error Fd76 value \n");

  if(0< write(Fd64,"1",1))
    printf("\n error Fd64 value \n");
}


int main()
{
  // create a GPIO object from MRAA using it
  int Led, period, pwm1;

  IOSetup();

  period = open("/sys/class/pwm/pwmchip0/pwm1/period", O_WRONLY);
  if(0< write(period,"13000000",8))
    printf("\n error period value \n");

  Led = open("/sys/class/pwm/pwmchip0/pwm1/duty_cycle", O_WRONLY);
  if(0< write(Led,"7000000",7))
    printf("\n error LED value \n");

  sleep(20);

  pwm1 = open("/sys/class/pwm/pwmchip0/pwm1/enable", O_WRONLY);
  if (pwm1 < 0)
  {
    printf("\n pwm1 open failed \n");
  }

  if(0< write(pwm1,"1",1))
    printf("\n error pwm1 enable \n");

  return 0;
}

