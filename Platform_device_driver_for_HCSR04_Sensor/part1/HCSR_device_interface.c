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
#include "HCSR_interface.h"

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
#define DELAY 70

typedef struct device_structure
{
  struct miscdevice HCSR_dev_device;
  char name[20];
  struct kfifo in_fifo;
  pins gpio_pins;
  parameters hcsr_parameters;
  int irq_no;
  int ongoing;
  unsigned long long starttime;
  unsigned long long endtime;
  unsigned int totaldistance;
  int irq_ret;
  unsigned long long mindistance;
  unsigned long long maxdistance;
  struct task_struct *sleeping_task;
  read_data sReadData;
  struct workqueue_struct *my_wq;  // work queue
} device_structure;

typedef struct my_work_t
{
  struct work_struct my_work;
  int x;
  struct device_structure *work_devp;
} my_work_t;

device_structure *p_device_structure;

static int iNumberOfInstances = 2;
module_param (iNumberOfInstances, int, S_IRUGO);

MODULE_PARM_DESC (iNumberOfInstances,
                  "Number of intances of HC-SR04 sensors to be created");

int set_pin_config_helper (unsigned int uiPinNum, int iDirection,
                           bool direction_may_change, int pin_to_set);
int set_pin_configuration (unsigned int uiPinNum, int iDirection,
                           bool direction_may_change, bool pull_up);
void gpio_pins_free (unsigned int uiPinNum);
static irqreturn_t interrupt_handler (int irq, void *dev_id,
                                      struct pt_regs *regs);
static void my_wq_function (struct work_struct *work);

/* pin mux table for configuring gpio on Galileo Gen 2 board */
static const unsigned int gen2_pin_mux_table[NUMBER_OF_IO_PINS][8] = {
  {11, 32, 33, NaN, NaN, NaN, NaN, 0}, //IO0
  {12, 28, 29, 45, 0, NaN, NaN, 0},    //IO1
  {13, 34, 35, 77, 0, NaN, NaN, 0},    //IO2
  {14, 16, 17, 76, 0, 64, 0, 0},       //IO3
  {6, 36, 37, NaN, NaN, NaN, NaN, 1},  //IO4
  {0, 18, 19, 66, 0, NaN, NaN, 1},     //IO5
  {1, 20, 21, 68, 0, NaN, NaN, 1},     //IO6
  {38, NaN, 39, NaN, NaN, NaN, NaN, 0}, //IO7
  {40, NaN, 41, NaN, NaN, NaN, NaN, 0}, //IO8
  {4, 22, 23, 70, 0, NaN, NaN, 1},     //IO9
  {10, 26, 27, 74, 0, NaN, NaN, 0},    //IO10
  {5, 24, 25, 44, 0, 72, 0, 1},        //IO11
  {15, 42, 43, NaN, NaN, NaN, NaN, 0}, //IO12
  {7, 30, 31, 46, 0, NaN, NaN, 1},     //IO13
  {48, NaN, 49, NaN, NaN, NaN, NaN, 1}, //IO14
  {50, NaN, 51, NaN, NaN, NaN, NaN, 1}, //IO15
  {52, NaN, 53, NaN, NaN, NaN, NaN, 1}, //IO16
  {54, NaN, 55, NaN, NaN, NaN, NaN, 1}, //IO17
  {56, NaN, 57, 60, 1, 78, 1, 1},      //IO18
  {58, NaN, 59, 60, 1, 79, 1, 1}       //IO19
};

/* Function to trigger the measurement */
void triggerPin (device_structure * p_device_structure, int delay)
{
  gpio_set_value_cansleep (gen2_pin_mux_table
      [p_device_structure->gpio_pins.
      trigger_pin][LINUX_PIN], LOW);
  udelay (2);
  gpio_set_value_cansleep (gen2_pin_mux_table
      [p_device_structure->gpio_pins.
      trigger_pin][LINUX_PIN], HIGH);
  udelay (12);
  gpio_set_value_cansleep (gen2_pin_mux_table
      [p_device_structure->gpio_pins.
      trigger_pin][LINUX_PIN], LOW);
  msleep (delay);
}

/*
* Open HCSR_dev_open
*/
static int HCSR_dev_open (struct inode *inode, struct file *file)
{
  /* By default, miscdevice framework sets file->private_data to point to the
     device-structure on miscdevice open() syscall to the device */
  device_structure *p_device_structure = file->private_data;

  p_device_structure->ongoing = 0;
  p_device_structure->totaldistance=0;
  /*Max Value of distance<<6==(400cm*400MHz*588*(2^6))*/
  p_device_structure->mindistance=593920000;
  p_device_structure->maxdistance=0;
  p_device_structure->irq_ret = true;
  p_device_structure->sleeping_task = NULL;
  p_device_structure->my_wq = create_workqueue ("my_queue");

  printk ("\n%s is openning \n", p_device_structure->name);
  return 0;
}

/*
 * Release HCSR_dev_close
 */
static int HCSR_dev_close (struct inode *inodep, struct file *file)
{
  device_structure *p_device_structure = file->private_data;

  /* free the allocated gpio pins */
  gpio_pins_free (p_device_structure->gpio_pins.echo_pin);
  gpio_pins_free (p_device_structure->gpio_pins.trigger_pin);
  /* free the interrupt only if it is called */
  if (p_device_structure->irq_ret == false)
  {
    free_irq (p_device_structure->irq_no, (void *) p_device_structure);
  }
  /* flushing the perdevice worker queue */
  if (p_device_structure->my_wq)
  {
    flush_workqueue (p_device_structure->my_wq);
    destroy_workqueue (p_device_structure->my_wq);
  }
  printk ("\nClosing\n");
  return 0;
}

/* Worker function */
static void my_wq_function (struct work_struct *work)
{
  my_work_t *my_work_fun = container_of (work, my_work_t, my_work);
  int *trigger = &((my_work_fun->work_devp)->ongoing);
  unsigned long long *endtime = &((my_work_fun->work_devp)->endtime);
  unsigned long long *starttime = &((my_work_fun->work_devp)->starttime);
  unsigned int *totaldistance =
    &((my_work_fun->work_devp)->totaldistance);
  unsigned long long *maxdistance=&((my_work_fun->work_devp)->maxdistance);
  unsigned long long *mindistance=&((my_work_fun->work_devp)->mindistance);
  read_data *sReadData=&((my_work_fun->work_devp)->sReadData);


  if (*trigger == 0)
  {
    *trigger = 1;
    *starttime = rdtsc ();
  }
  else
  {
    *endtime = rdtsc ();
    /* Here we are storing the distance in fixed point Q6 format */
    /* 400(galileo MHZ)*58 = 23200 as time in sec /58 will give distance */
    sReadData->distance = (int) ((*endtime - *starttime) << 6) / 23200;
    sReadData->timestamp = (*endtime);
    *totaldistance=*totaldistance+sReadData->distance;
    if(sReadData->distance<*mindistance)
      *mindistance=sReadData->distance;
    if(sReadData->distance>*maxdistance)
      *maxdistance=sReadData->distance;
    *trigger = 0;
  }
  return;
}

/* Interrupt service routine */
static irqreturn_t interrupt_handler (int irq, void *dev_id,
                                      struct pt_regs *regs)
{
  my_work_t *work;
  device_structure *p_device_structure = (device_structure *) dev_id;
  int ret = 0;

  /* Queue some work (item 1) */
  if (p_device_structure->my_wq)
  {
    work = (my_work_t *) kmalloc (sizeof (my_work_t), GFP_KERNEL);
    work->work_devp = p_device_structure;
    if (work)
    {
      INIT_WORK (&work->my_work, my_wq_function);

      /* If work is there then push into  the work queue */
      ret = queue_work (p_device_structure->my_wq, &work->my_work);
    }
  }
  return IRQ_HANDLED;
}

/*
 * Write to HCSR driver
 */
static ssize_t HCSR_dev_write (struct file *file, const char __user * buf,
                               size_t len, loff_t * ppos)
{
  device_structure *p_device_structure = file->private_data;
  int int_args = 0;

  /* Getting the input argument from the user */
  if (copy_from_user (&int_args, (int *) buf, sizeof (int)))
  {
    printk ("Unable to copy_from_user \n");
    return -EACCES;
  }

  /* If a measurement is ongoing then returning einval */
  if(p_device_structure->ongoing == 1)
  {
    printk("Ongoing measurement.Try after some time\n");
    return -EINVAL;
  }
  else if (int_args != 0)
  {
    /* clearing the FIFO buffer if input argument is not zero */
    kfifo_reset (&p_device_structure->in_fifo);
    msleep(1);
    triggerPin(p_device_structure, DELAY);
    kfifo_in (&p_device_structure->in_fifo, &p_device_structure->sReadData, sizeof (read_data));
  }
  else
  {
    /* triggering a new measurement */
    triggerPin(p_device_structure, DELAY);
    kfifo_in (&p_device_structure->in_fifo, &p_device_structure->sReadData, sizeof (read_data));
    if(p_device_structure->sleeping_task != NULL)
    {
      wake_up_process(p_device_structure->sleeping_task);
    }
  }
  return 0;
}

/*
 * read from HCSR sensor
 */
ssize_t HCSR_dev_read (struct file * file, char *buf,
                       size_t count, loff_t * ppos)
{
  int ret = 0;
  device_structure *p_device_structure = file->private_data;
  unsigned int uiNumBytesCopied = 0;

  /* Triggering a new measurement if buffer is empty and there is no ongoing measurement */
  if (kfifo_is_empty (&p_device_structure->in_fifo) && p_device_structure->ongoing == 0 )
  {
    printk(" TRIGGERING NEW MEASUREMENT AS BUFFER IS EMPTY AND NO ONGOING PROCESS\n");
    triggerPin(p_device_structure, DELAY);
    kfifo_in (&p_device_structure->in_fifo, &p_device_structure->sReadData, sizeof (read_data));
  }
  else
  {
    if (p_device_structure->ongoing == 1)
    {
      p_device_structure->sleeping_task = current;
      set_current_state(TASK_INTERRUPTIBLE);
      /* If a measurement is ongoing, this routine will wait till write or ioctl wakesup this routine */
      schedule();
      ret =
        kfifo_to_user (&p_device_structure->in_fifo, buf, sizeof (read_data),
            &uiNumBytesCopied);
      if (ret < 0)
      {
        return -EINVAL;
      }
    }
    else
    {
      /* If buffer is not empty and no ongoing measurement returning the buffer values to the user */
      ret =
        kfifo_to_user (&p_device_structure->in_fifo, buf, sizeof (read_data),
            &uiNumBytesCopied);
      if (ret < 0)
      {
        return -EINVAL;
      }
    }
  }
  return uiNumBytesCopied;
}

/* setting pin configurations */
int set_pin_config_helper (unsigned int uiPinNum, int iDirection,
                           bool direction_may_change, int pin_to_set)
{
  int ret = 0;

  ret = gpio_request (gen2_pin_mux_table[uiPinNum][pin_to_set], "user_gpio");
  if (ret != 0)
  {
    printk ("gpio_request FAILED with return %d \n", ret);
    goto done;
  }
  if (pin_to_set == PIN_MUX_1)
  {
    ret = gpio_cansleep (gen2_pin_mux_table[uiPinNum][pin_to_set]);
    if (ret != 0)
    {
      gpio_set_value_cansleep (gen2_pin_mux_table[uiPinNum][pin_to_set],
          gen2_pin_mux_table[uiPinNum][PIN_MUX_1_VALUE]);
    }
    else
    {
      gpio_set_value (gen2_pin_mux_table[uiPinNum][pin_to_set],
          gen2_pin_mux_table[uiPinNum][PIN_MUX_1_VALUE]);
    }
  }
  else if (pin_to_set == PIN_MUX_2)
  {
    ret = gpio_cansleep (gen2_pin_mux_table[uiPinNum][pin_to_set]);
    if (ret != 0)
    {
      gpio_set_value_cansleep (gen2_pin_mux_table[uiPinNum][pin_to_set],
          gen2_pin_mux_table[uiPinNum][PIN_MUX_2_VALUE]);
    }
    else
    {
      gpio_set_value (gen2_pin_mux_table[uiPinNum][pin_to_set],
          gen2_pin_mux_table[uiPinNum][PIN_MUX_2_VALUE]);
    }
  }
  else
  {
    if (iDirection == DIRECTION_OUT)
    {
      ret =
        gpio_direction_output (gen2_pin_mux_table[uiPinNum][pin_to_set],
            iDirection);
      if (ret != 0)
      {
        printk ("gpio_direction_output FAILED with return %d \n", ret);
        goto done;
      }
    }
    else
    {
      ret = gpio_direction_input (gen2_pin_mux_table[uiPinNum][pin_to_set]);
      if (ret != 0)
      {
        printk ("gpio_direction_input FAILED with return %d \n", ret);
        goto done;
      }
    }
  }
  ret =
    gpio_export (gen2_pin_mux_table[uiPinNum][pin_to_set],
        direction_may_change);
  if (ret != 0)
  {
    printk ("gpio_export FAILED with return %d \n", ret);
    goto done;
  }
done:
  return ret;
}

int set_pin_configuration (unsigned int uiPinNum, int iDirection,
                           bool direction_may_change, bool pull_up)
{
  int ret = 0;

  ret =
    set_pin_config_helper (uiPinNum, iDirection, direction_may_change,
        LINUX_PIN);
  if (ret != 0)
  {
    printk ("set_pin_config_helper for LINUX_PIN FAILED with return %d \n",
        ret);
    goto done;
  }
  if (gen2_pin_mux_table[uiPinNum][LEVEL_SHIFTER_PIN] != NaN)
  {
    ret =
      set_pin_config_helper (uiPinNum, iDirection, direction_may_change,
          LEVEL_SHIFTER_PIN);
    if (ret != 0)
    {
      printk
        ("set_pin_config_helper for LEVEL_SHIFTER_PIN FAILED with return %d \n",
         ret);
      goto done;
    }
  }
  if (pull_up)
  {
    ret =
      set_pin_config_helper (uiPinNum, iDirection, direction_may_change,
          PULL_UP_PIN);
    if (ret != 0)
    {
      printk ("set_pin_config_helper for PULL_UP_PIN FAILED with return %d \n",
          ret);
      goto done;
    }
  }
  if (gen2_pin_mux_table[uiPinNum][PIN_MUX_1] != NaN)
  {
    ret =
      set_pin_config_helper (uiPinNum, iDirection, direction_may_change,
          PIN_MUX_1);
    if (ret != 0)
    {
      printk ("set_pin_config_helper for PIN_MUX_1 FAILED with return %d \n",
          ret);
      goto done;
    }
  }
  if (gen2_pin_mux_table[uiPinNum][PIN_MUX_2] != NaN)
  {
    ret =
      set_pin_config_helper (uiPinNum, iDirection, direction_may_change,
          PIN_MUX_2);
    if (ret != 0)
    {
      printk ("set_pin_config_helper for PIN_MUX_2 FAILED with return %d \n",
          ret);
      goto done;
    }
  }
done:
  return ret;
}

/* To free gpio pins */
void gpio_pins_free (unsigned int uiPinNum)
{
  gpio_free (gen2_pin_mux_table[uiPinNum][LINUX_PIN]);
  if (gen2_pin_mux_table[uiPinNum][LEVEL_SHIFTER_PIN] != NaN)
  {
    gpio_free (gen2_pin_mux_table[uiPinNum][LEVEL_SHIFTER_PIN]);
  }
  gpio_free (gen2_pin_mux_table[uiPinNum][PULL_UP_PIN]);
  if (gen2_pin_mux_table[uiPinNum][PIN_MUX_1] != NaN)
  {
    gpio_free (gen2_pin_mux_table[uiPinNum][PIN_MUX_1]);
  }
  if (gen2_pin_mux_table[uiPinNum][PIN_MUX_2] != NaN)
  {
    gpio_free (gen2_pin_mux_table[uiPinNum][PIN_MUX_2]);
  }
}

/* HCSR ioctl */
static long HCSR_dev_ioctl (struct file *file, unsigned int cmd,
                            unsigned long arg)
{
  device_structure *p_device_structure = file->private_data;
  int ret = 0;
  int i = 0;

  switch (cmd)
  {
    case CONFIG_PINS:
      {
        if (copy_from_user
            (&p_device_structure->gpio_pins, (pins *) arg, sizeof (pins)))
        {
          return -EACCES;
        }
        printk ("ENTERED GPIO PINS ARE ECHO %d TRIGGER %d \n",
            p_device_structure->gpio_pins.echo_pin,
            p_device_structure->gpio_pins.trigger_pin);
        /* value of trigger pin should be between 0-19 */
        if ((p_device_structure->gpio_pins.trigger_pin >
              (NUMBER_OF_IO_PINS -
               1)) | (p_device_structure->gpio_pins.trigger_pin < 0))
        {
          printk ("Invalid trigger_pin config\n");
          return -EINVAL;
        }
        /* value of echo pin should be between 0-19 */
        if ((p_device_structure->gpio_pins.echo_pin >
              (NUMBER_OF_IO_PINS - 1)) | (p_device_structure->gpio_pins.echo_pin <
                0))
        {
          printk ("Invalid echo_pin config\n");
          return -EINVAL;
        }
        /* Echo pin should support both rising and falling interrupt */
        if (gen2_pin_mux_table[p_device_structure->gpio_pins.echo_pin]
            [BOTH_INTERRUPT_SUPPORT] != 1)
        {
          printk
            ("Echo pin should support both rising and falling edge interrupt modes. Please choose another pin for echo \n");
          return -EINVAL;
        }
        ret =
          set_pin_configuration (p_device_structure->gpio_pins.echo_pin,
              DIRECTION_IN, true, true);
        if (ret != 0)
        {
          printk ("set_pin_configuration FAILED \n");
          return ret;
        }
        printk (" ECHO PIN CONFIGURATION IS SET \n");
        ret =
          set_pin_configuration (p_device_structure->gpio_pins.trigger_pin,
              DIRECTION_OUT, true, false);
        if (ret != 0)
        {
          printk ("set_pin_configuration FAILED \n");
          return ret;
        }
        printk (" TRIGGER PIN CONFIGURATION IS SET \n");
        /* Getting the irq number from the gpio number */
        p_device_structure->irq_no =
          gpio_to_irq (gen2_pin_mux_table[p_device_structure->gpio_pins.echo_pin]
              [LINUX_PIN]);
        printk ("IRQ NUMBER : %d\n", p_device_structure->irq_no);
        /*registering the IRQ for echo pin */
        p_device_structure->irq_ret =
          request_irq (p_device_structure->irq_no,
              (irq_handler_t) interrupt_handler,
              (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
              "HCSR04_INT", (void *) p_device_structure);
        if (p_device_structure->irq_ret)
        {
          printk (KERN_INFO "short: can't get assigned irq %i\n",
              p_device_structure->irq_no);
        }
        break;
      }
    case SET_PARAMETERS:
      {
        if (copy_from_user
            (&p_device_structure->hcsr_parameters, (parameters *) arg,
             sizeof (parameters)))
        {
          return -EACCES;
        }
        if (!kfifo_is_empty (&p_device_structure->in_fifo))
        {
          kfifo_reset (&p_device_structure->in_fifo);
          msleep(1);
        }

        printk ("ENTERED OPERATION PARAMETES ARE %d %d \n",
            p_device_structure->hcsr_parameters.m,
            p_device_structure->hcsr_parameters.delta);

        p_device_structure->totaldistance=0;
        /*Max Value of distance<<6==(400cm*400MHz*588*(2^6))*/
        p_device_structure->mindistance=593920000;
        p_device_structure->maxdistance=0;

        /* Triggering the device M+2 times */
        for (i = 0; i < (p_device_structure->hcsr_parameters.m + 2); i++)
        {
          triggerPin(p_device_structure, p_device_structure->hcsr_parameters.delta);
        }

        /* Removing the minimum and maximum values in M+2 samples */
        p_device_structure->totaldistance=p_device_structure->totaldistance-(p_device_structure->maxdistance+p_device_structure->mindistance);

        /* Taking the average of M samples */
        p_device_structure->sReadData.distance=(p_device_structure->totaldistance)/(p_device_structure->hcsr_parameters.m);

        kfifo_in (&p_device_structure->in_fifo, &p_device_structure->sReadData, sizeof (read_data));
        if(p_device_structure->sleeping_task != NULL)
        {
          wake_up_process(p_device_structure->sleeping_task);
        }

        break;
      }
    default:
      return -EINVAL;
  }
  return 0;
}

/* File operations structure. Defined in linux/fs.h */
static const struct file_operations HCSR_dev_fops = {
  .owner = THIS_MODULE,                /* Owner */
  .open = HCSR_dev_open,               /* Open method */
  .release = HCSR_dev_close,           /* Release method */
  .write = HCSR_dev_write,             /* write method */
  .read = HCSR_dev_read,               /* Read method */
  .unlocked_ioctl = HCSR_dev_ioctl     /* IOCTL method */
};

static int __init HCSR_misc_init (void)
{
  int error;
  int i = 0;

  /* creating n instances of the device structure */
  p_device_structure =
    (device_structure *)
    kzalloc ((iNumberOfInstances * sizeof (device_structure)), GFP_KERNEL);
  if (!p_device_structure)
    return -ENOMEM;

  for (i = 0; i < iNumberOfInstances; i++)
  {
    sprintf (p_device_structure[i].name, "%s%d", DEVICE_NAME, i);
    /* allocating the kfifo buffer */
    if (kfifo_alloc (&p_device_structure[i].in_fifo, FIFO_SIZE, GFP_KERNEL))
    {
      printk ("error kfifo_alloc\n");
      return 1;
    }
    p_device_structure[i].HCSR_dev_device.minor = MISC_DYNAMIC_MINOR;
    p_device_structure[i].HCSR_dev_device.name = p_device_structure[i].name;
    p_device_structure[i].HCSR_dev_device.fops = &HCSR_dev_fops;

    /* registering the miscellaneous device */
    error = misc_register (&p_device_structure[i].HCSR_dev_device);
    if (error)
    {
      printk ("can't misc_register :(\n");
      return error;
    }
  }

  printk ("I'm in\n");
  return 0;
}

static void __exit HCSR_misc_exit (void)
{
  int i = 0;

  for (i = 0; i < iNumberOfInstances; i++)
  {
    /* Unregistering the miscellaneous device */
    misc_deregister (&p_device_structure[i].HCSR_dev_device);
    /* fee kfifo buffer */
    kfifo_free (&p_device_structure[i].in_fifo);
  }
  /* free the device structure */
  kfree (p_device_structure);
  printk ("I'm out\n");
}

module_init (HCSR_misc_init);
module_exit (HCSR_misc_exit);
MODULE_AUTHOR ("TEAM20");
MODULE_DESCRIPTION ("HCSR DRIVER MODULE");
MODULE_LICENSE ("GPL v2");
