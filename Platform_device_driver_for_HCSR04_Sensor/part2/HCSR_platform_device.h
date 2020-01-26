#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>
#include <linux/kfifo.h>
#include <linux/time.h>
#include <linux/sched.h>

#ifndef __HCSR_PLATFORM_H__

#define __HCSR_PLATFORM_H__

#define NUMBER_OF_IO_PINS 20
#define DEVICE_NAME "HCSR_"
#define FIFO_SIZE 128                  /* Must be a power of 2 */
#define NaN 255
#define	LINUX_PIN 0
#define LEVEL_SHIFTER_PIN 1
#define	PULL_UP_PIN 2
#define	PIN_MUX_1 3
#define	PIN_MUX_1_VALUE 4
#define	PIN_MUX_2 5
#define	PIN_MUX_2_VALUE 6
#define	BOTH_INTERRUPT_SUPPORT 7
#define DIRECTION_IN  1
#define DIRECTION_OUT 0
#define HIGH  1
#define LOW 0
#define CONFIG_PINS _IOR(81, 1, pins)
#define SET_PARAMETERS _IOR(91, 2, parameters)

typedef struct HCSR_device_chip {
  char 		name[20];
  int			dev_no;
  struct platform_device 	plf_dev;
}HCSR_device_chip;

typedef struct pins
{
  unsigned int echo_pin;
  unsigned int trigger_pin;
}pins;

typedef struct parameters
{
  int m;
  int delta;
}parameters;

typedef struct read_data{
  unsigned int distance;
  unsigned long long timestamp;
}read_data;

typedef struct my_work_t
{
  struct work_struct my_work;
  int x;
  struct device_structure *work_devp;
} my_work_t;

typedef struct device_structure
{
  HCSR_device_chip p_plt_HCSR_device;
  struct miscdevice HCSR_dev_device;
  char name[20];
  struct kfifo in_fifo;
  int irq_no;
  int ongoing;
  unsigned int totaldistance;
  int irq_ret;
  int enable;
  int dev_minor_number;
  unsigned long long starttime;
  unsigned long long endtime;
  unsigned long long mindistance;
  unsigned long long maxdistance;
  pins gpio_pins;
  parameters hcsr_parameters;
  struct task_struct *sleeping_task;
  read_data sReadData;
  struct workqueue_struct *my_wq;  // work queue
} device_structure;

/* Took this function from linux source v3.19.8 /x86/util/tsc.c*/
static inline unsigned long long rdtsc(void)
{
  unsigned int low, High;
  asm volatile("rdtsc" : "=a" (low), "=d" (High));
  return (((unsigned long long)low) | (((unsigned long long)High) << 32));
}

#endif
