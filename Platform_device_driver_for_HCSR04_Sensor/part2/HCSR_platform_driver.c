#include "HCSR_platform_device.h"

#define DRIVER_NAME		"HCSR_of_driver"
#define CLASS_NAME		"HCSR"
#define DELAY   70

static struct class *dev_class; /* Tie with the device model */
static int first_device = 1;
static int iNumDevMatched = 0;
static int minor_number = 0;
struct device *hcsr_dev;
static int i=0;
static int j=0;
static struct miscdevice* miscdevice_add[5]={};
static const struct platform_device_id P_id_table[] = {
  {"HCSR_0", 0},
  {"HCSR_1", 0},
  {},
};

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
      [p_device_structure->
      gpio_pins.trigger_pin][LINUX_PIN], LOW);
  udelay (2);
  gpio_set_value_cansleep (gen2_pin_mux_table
      [p_device_structure->
      gpio_pins.trigger_pin][LINUX_PIN], HIGH);
  udelay (12);
  gpio_set_value_cansleep (gen2_pin_mux_table
      [p_device_structure->
      gpio_pins.trigger_pin][LINUX_PIN], LOW);
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

  if (kfifo_alloc (&p_device_structure->in_fifo, FIFO_SIZE, GFP_KERNEL))
  {
    printk ("error kfifo_alloc \n");
  }
  p_device_structure->ongoing = 0;
  p_device_structure->totaldistance = 0;
  /*Max Value of distance<<6==(400cm*400MHz*588*(2^6)) */
  p_device_structure->mindistance = 593920000;
  p_device_structure->maxdistance = 0;
  p_device_structure->irq_ret = true;
  p_device_structure->sleeping_task = NULL;
  p_device_structure->my_wq = create_workqueue ("my_queue");

  printk ("\n%s is openning \n", p_device_structure->name);
  return 0;
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
    printk ("Ongoing measurement.Try after some time\n");
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
        if ((p_device_structure->gpio_pins.trigger_pin >
              (NUMBER_OF_IO_PINS -
               1)) | (p_device_structure->gpio_pins.trigger_pin < 0))
        {
          printk ("Invalid trigger_pin config\n");
          return -EINVAL;
        }
        if ((p_device_structure->gpio_pins.echo_pin >
              (NUMBER_OF_IO_PINS - 1)) | (p_device_structure->gpio_pins.echo_pin <
                0))
        {
          printk ("Invalid echo_pin config\n");
          return -EINVAL;
        }
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
          triggerPin (p_device_structure, p_device_structure->hcsr_parameters.delta);
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

static ssize_t trigger_store (struct device *dev,
                              struct device_attribute *attr, const char *buf,
                              size_t count)
{
  int ret = 0;
  device_structure *p_device_structure = dev_get_drvdata (dev);

  p_device_structure->ongoing = 0;
  p_device_structure->irq_ret = true;
  p_device_structure->enable = 0;
  p_device_structure->my_wq = create_workqueue ("my_queue");
  sscanf (buf, "%u", &p_device_structure->gpio_pins.trigger_pin);
  ret = set_pin_configuration (p_device_structure->gpio_pins.trigger_pin,
      DIRECTION_OUT, true, false);
  if (ret != 0)
  {
    printk ("set_pin_configuration FAILED \n");
    return ret;
  }
  printk ("Value of trigger pin=%u\n",
      p_device_structure->gpio_pins.trigger_pin);
  return count;
}

static ssize_t trigger_show (struct device *dev,
                             struct device_attribute *attr, char *buf)
{
  int len = 0;
  device_structure *p_device_structure = dev_get_drvdata (dev);

  len = sprintf (buf, "%u \n", (p_device_structure->gpio_pins.trigger_pin));
  return len;
}

static ssize_t echo_store (struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
  device_structure *p_device_structure = dev_get_drvdata (dev);
  int ret = 0;

  sscanf (buf, "%u", &p_device_structure->gpio_pins.echo_pin);
  ret =
    set_pin_configuration (p_device_structure->gpio_pins.echo_pin,
        DIRECTION_IN, true, true);
  if (ret != 0)
  {
    printk ("set_pin_configuration FAILED \n");
    return ret;
  }
  p_device_structure->irq_no =
    gpio_to_irq (gen2_pin_mux_table[p_device_structure->gpio_pins.echo_pin]
        [LINUX_PIN]);
  printk ("IRQ NUMBER : %d\n", p_device_structure->irq_no);
  /*request irq for echo pin */
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
  printk ("Value of echo pin = %u\n", p_device_structure->gpio_pins.echo_pin);
  return count;
}

static ssize_t echo_show (struct device *dev,
                          struct device_attribute *attr, char *buf)
{
  int len = 0;
  device_structure *p_device_structure = dev_get_drvdata (dev);

  len = sprintf (buf, "%u \n", (p_device_structure->gpio_pins.echo_pin));
  return len;
}

static ssize_t number_samples_store (struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
  device_structure *p_device_structure = dev_get_drvdata (dev);

  sscanf (buf, "%d", &p_device_structure->hcsr_parameters.m);
  printk ("Number Of Samples = %d \n", p_device_structure->hcsr_parameters.m);
  return count;
}

static ssize_t number_samples_show (struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
  int len = 0;
  device_structure *p_device_structure = dev_get_drvdata (dev);

  len = sprintf (buf, "%u \n", (p_device_structure->hcsr_parameters.m));
  return len;
}

static ssize_t sampling_period_store (struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t count)
{
  device_structure *p_device_structure = dev_get_drvdata (dev);

  sscanf (buf, "%u", &p_device_structure->hcsr_parameters.delta);
  printk ("Value of sampling_period = %d \n", p_device_structure->hcsr_parameters.delta);
  return count;
}

static ssize_t sampling_period_show (struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
  int len = 0;
  device_structure *p_device_structure = dev_get_drvdata (dev);

  len = sprintf (buf, "%d \n", (p_device_structure->hcsr_parameters.delta));
  return len;
}

static ssize_t enable_store (struct device *dev, struct device_attribute *attr,
                             const char *buf, size_t count)
{
  device_structure *p_device_structure = dev_get_drvdata (dev);
  int i = 0;
  sscanf (buf, "%u", &p_device_structure->enable);

  if(p_device_structure->enable == 1)
  {
    if (p_device_structure->ongoing == 1)
    {
      printk ("Ongoing measurement.Try after some time\n");
    }
    else
    {
      p_device_structure->totaldistance=0;
      /*Max Value of distance<<6==(400cm*400MHz*588*(2^6))*/
      p_device_structure->mindistance=593920000;
      p_device_structure->maxdistance=0;
      for (i = 0; i < (p_device_structure->hcsr_parameters.m + 2); i++)
      {
        triggerPin (p_device_structure, p_device_structure->hcsr_parameters.delta);
      }
      p_device_structure->totaldistance =
        p_device_structure->totaldistance - (p_device_structure->maxdistance +
            p_device_structure->mindistance);
      p_device_structure->sReadData.distance=(p_device_structure->totaldistance)/(p_device_structure->hcsr_parameters.m);
    }
  }
  return count;
}

static ssize_t enable_show (struct device *dev,
                            struct device_attribute *attr, char *buf)
{
  int len = 0;
  device_structure *p_device_structure = dev_get_drvdata (dev);

  len = sprintf (buf, "%u \n", (p_device_structure->enable));
  return len;
}

static ssize_t distance_show (struct device *dev,
                              struct device_attribute *attr, char *buf)
{
  int len = 0;
  device_structure *p_device_structure = dev_get_drvdata (dev);

  if(p_device_structure->enable == 1)
  {
    len = sprintf (buf, "%u \n", (p_device_structure->sReadData.distance >> 6));
    if (p_device_structure->my_wq)
    {
      flush_workqueue (p_device_structure->my_wq);
      destroy_workqueue (p_device_structure->my_wq);
    }
    if (p_device_structure->irq_ret == false)
    {
      free_irq (p_device_structure->irq_no, (void *) p_device_structure);
    }
  }
  else
  {
    len = sprintf (buf, "%u \n", 0);
  }
  gpio_pins_free (p_device_structure->gpio_pins.echo_pin);
  gpio_pins_free (p_device_structure->gpio_pins.trigger_pin);


  return len;
}

static DEVICE_ATTR (trigger, S_IRUSR | S_IWUSR, trigger_show, trigger_store);
static DEVICE_ATTR (echo, S_IRUSR | S_IWUSR, echo_show, echo_store);
static DEVICE_ATTR (number_samples, S_IRUSR | S_IWUSR, number_samples_show,
                    number_samples_store);
static DEVICE_ATTR (sampling_period, S_IRUSR | S_IWUSR, sampling_period_show,
                    sampling_period_store);
static DEVICE_ATTR (enable, S_IRUSR | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR (distance, S_IRUSR | S_IWUSR, distance_show,
                    NULL);

static struct attribute *gpio_attributes[] = {
  &dev_attr_trigger.attr,
  &dev_attr_echo.attr,
  &dev_attr_number_samples.attr,
  &dev_attr_sampling_period.attr,
  &dev_attr_enable.attr,
  &dev_attr_distance.attr,
  NULL,
};

static struct attribute_group gpio_group = {.attrs = gpio_attributes };

static const struct attribute_group *gpio_groups[] = {
  &gpio_group,
  NULL,
};

static int init_device (HCSR_device_chip * p_HCSR_device_chip,
                        device_structure * p_device_structure)
{
  int ret = 0;
  p_device_structure->p_plt_HCSR_device = *p_HCSR_device_chip;

  sprintf (p_device_structure->name, "%s%d", DEVICE_NAME,
      p_HCSR_device_chip->dev_no);
  p_device_structure->HCSR_dev_device.minor = MISC_DYNAMIC_MINOR;
  p_device_structure->HCSR_dev_device.name = p_device_structure->name;
  p_device_structure->HCSR_dev_device.fops = &HCSR_dev_fops;

  ret = misc_register (&p_device_structure->HCSR_dev_device);
  if (ret)
  {
    printk ("can't misc_register :( \n");
    return ret;
  }
  /* stroing all the misc devices in a global array to deregister at the end */
  miscdevice_add[i++]= &p_device_structure->HCSR_dev_device;
  return ret;
}

static int P_driver_probe (struct platform_device *dev_found)
{
  int ret = 0;
  device_structure *p_device_structure = NULL;
  HCSR_device_chip *p_HCSR_device_chip = NULL;

  p_HCSR_device_chip =
    container_of (dev_found, struct HCSR_device_chip, plf_dev);

  printk (KERN_ALERT "Found the device -- %s  %d \n", p_HCSR_device_chip->name,
      p_HCSR_device_chip->dev_no);

  iNumDevMatched += 1;

  /* creating the sysfs class only the fisrt time it is entering the probe */
  if (first_device == 1)
  {
    dev_class = class_create (THIS_MODULE, CLASS_NAME);
    if (IS_ERR (dev_class))
    {
      printk (KERN_ERR DEVICE_NAME " cant create class %s\n", CLASS_NAME);
      ret = PTR_ERR (dev_class);
      goto class_err;
    }
    first_device = 0;
  }

  p_device_structure =
    (device_structure *) kzalloc ((sizeof (device_structure)), GFP_KERNEL);
  if (!p_device_structure)
  {
    printk ("BAD ALLOC \n");
    return -ENOMEM;
  }

  memset(p_device_structure, 0 , sizeof(device_structure));

  /* Initializing the device structure */
  ret = init_device (p_HCSR_device_chip, p_device_structure);
  if (ret < 0)
  {
    printk ("INITIALIZATION OF THE DEVICE FAILED \n");
    return ret;
  }

  minor_number += 1;
  p_device_structure->dev_minor_number=minor_number;

  /* creating the sysfs devices using device_create_with_groups API */
  hcsr_dev =
    device_create_with_groups (dev_class,
        &(p_device_structure->p_plt_HCSR_device.
          plf_dev.dev), MKDEV (0, minor_number),
        p_device_structure, gpio_groups,
        p_device_structure->name);
  if (IS_ERR (hcsr_dev))
  {
    printk (KERN_ERR DEVICE_NAME " cant create class %s\n",
        p_device_structure->name);
    ret = PTR_ERR (hcsr_dev);
    goto device_err;
  }

  return 0;
device_err:
  device_destroy (dev_class, MKDEV (0, minor_number));
class_err:
  class_unregister (dev_class);
  class_destroy (dev_class);
  return ret;
};

static int P_driver_remove (struct platform_device *pdev)
{
  device_structure *p_temp = NULL;
  HCSR_device_chip *p_HCSR_device_chip = NULL;
  p_HCSR_device_chip = container_of (pdev, HCSR_device_chip, plf_dev);
  p_temp = container_of(p_HCSR_device_chip,device_structure, p_plt_HCSR_device);
  sysfs_remove_group(&pdev->dev.kobj, &gpio_group);
  misc_deregister (miscdevice_add[j++]);
  /* freeing all the devices */
  kfree (p_temp);
  iNumDevMatched--;

  if (iNumDevMatched == 0)
  {
    first_device = 1;
    /* class unregister */
    class_unregister (dev_class);
    class_destroy (dev_class);
    minor_number = 0;
  }
  printk (" REMOVED ALL OBJECTS \n");
  return 0;
};

static struct platform_driver P_driver = {
  .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
  },
  .probe = P_driver_probe,
  .remove = P_driver_remove,
  .id_table = P_id_table,
};

module_platform_driver (P_driver);
MODULE_AUTHOR ("TEAM20");
MODULE_DESCRIPTION ("HCSR PLATFORM DRIVER MODULE");
MODULE_LICENSE ("GPL v2");
