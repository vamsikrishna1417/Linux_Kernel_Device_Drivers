#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>

static struct hrtimer hr_timer;
static int count = 0;

enum hrtimer_restart timer_callback (struct hrtimer *timer_for_restart)
{
  printk ("hrtimer_callback called (%ld).\n", jiffies);
  count++;
  /* Trying to light 2 leds */
  if (count == 48)
  {
    return HRTIMER_NORESTART;
  }
  else
  {
    if (count % 2 != 0)
    {
      gpio_set_value (12, 0);
      hrtimer_forward (timer_for_restart,
          hrtimer_cb_get_time (timer_for_restart), ktime_set (0,
            600));
    }
    else
    {
      gpio_set_value (12, 1);
      hrtimer_forward (timer_for_restart,
          hrtimer_cb_get_time (timer_for_restart), ktime_set (0,
            700));
    }
  }
  return HRTIMER_RESTART;
}

//configure IO1 pin to be spi MOSI
static int set_IO1_configuration (void)
{
  int status = 0;

  status = gpio_request (12, "user_gpio");
  if (status != 0)
  {
    printk ("gpio_request FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_request (28, "user_gpio");
  if (status != 0)
  {
    printk ("gpio_request FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_request (45, "user_gpio");
  if (status != 0)
  {
    printk ("gpio_request FAILED with return %d \n", status);
    goto done;
  }

  status = gpio_direction_output (12, 0);
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

  status = gpio_export (12, true);
  if (status != 0)
  {
    printk ("gpio_export FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_export (28, true);
  if (status != 0)
  {
    printk ("gpio_export FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_export (45, true);
  if (status != 0)
  {
    printk ("gpio_export FAILED with return %d \n", status);
    goto done;
  }

done:
  return status;
}

static int __init WS2812_hrtimer_init (void)
{
  int status = 0;
  ktime_t ktime;

  printk ("Installing HR Timer module\n");

  //configure IO1 pin to be spi MOSI
  status = set_IO1_configuration ();
  if (status != 0)
  {
    printk ("set_IO1_configuration FAILED \n");
  }

  ktime = ktime_set (0, 700);
  hrtimer_init (&hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  hr_timer.function = &timer_callback;
  printk ("Starting timer to fire in (%ld)\n", jiffies);
  gpio_set_value (12, 1);
  hrtimer_start (&hr_timer, ktime, HRTIMER_MODE_REL);

  return status;
}

static void __exit WS2812_hrtimer_exit (void)
{
  int ret = 0;

  /* free GPIO pins */
  gpio_free (12);
  gpio_free (28);
  gpio_free (45);
  ret = hrtimer_cancel (&hr_timer);
  if (ret)
  {
    printk ("The timer was still in use...\n");
  }
  printk ("Uninstalling HR Timer module\n");
}

module_init (WS2812_hrtimer_init);
module_exit (WS2812_hrtimer_exit);
MODULE_AUTHOR ("TEAM20");
MODULE_DESCRIPTION ("WS2812 HRTIMER DRIVER MODULE");
MODULE_LICENSE ("GPL v2");
