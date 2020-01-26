#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>

static int iNumberofLeds = 3;
module_param (iNumberofLeds, int, S_IRUGO);

MODULE_PARM_DESC (iNumberofLeds,
                  "Number of LEDs of LED strip to light");

#define ONE \
{ \
  gpio_set_value(12,1);\
  ndelay(700);\
  gpio_set_value(12,0);\
  ndelay(600); }

#define ZERO \
{ \
  gpio_set_value(12,1);\
  ndelay(350);\
  gpio_set_value(12,0);\
  ndelay(800); }

static int  set_IO1_configuration(void)
{
  int status = 0;
  status = gpio_request(12, "user_gpio");
  if (status != 0)
  {
    printk ("gpio_request FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_request(28, "user_gpio");
  if (status != 0)
  {
    printk ("gpio_request FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_request(45, "user_gpio");
  if (status != 0)
  {
    printk ("gpio_request FAILED with return %d \n", status);
    goto done;
  }

  status = gpio_direction_output (12,0);
  if (status != 0)
  {
    printk ("gpio_direction_output FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_direction_output (28,0);
  if (status != 0)
  {
    printk ("gpio_direction_output FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_direction_output (45,0);
  if (status != 0)
  {
    printk ("gpio_direction_output FAILED with return %d \n", status);
    goto done;
  }

  status = gpio_export(12, true);
  if (status != 0)
  {
    printk ("gpio_export FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_export(28, true);
  if (status != 0)
  {
    printk ("gpio_export FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_export(45, true);
  if (status != 0)
  {
    printk ("gpio_export FAILED with return %d \n", status);
    goto done;
  }

done:
  return status;
}

static void led_message(int iNumberofLeds)
{
  int i=0;
  for(i=0; i<(24*iNumberofLeds); i++)
  {
    ONE;
  }
}


static int __init WS2812_ndelay_init (void)
{
  int status = 0;

  printk("Installing WS2812 ndelay module\n");

  //configure IO1 pin to be spi MOSI
  status = set_IO1_configuration();
  if (status != 0)
  {
    printk ("set_IO1_configuration FAILED \n");
  }

  led_message(iNumberofLeds);

  return 0;
}

static void __exit WS2812_ndelay_exit (void)
{
  /* free GPIO pins */
  gpio_free(12);
  gpio_free(45);
  gpio_free(28);

  printk("Uninstalling WS2812 ndelay module\n");
}

module_init (WS2812_ndelay_init);
module_exit (WS2812_ndelay_exit);
MODULE_AUTHOR ("TEAM20");
MODULE_DESCRIPTION ("WS2812 NDELAY DRIVER MODULE");
MODULE_LICENSE ("GPL v2");
