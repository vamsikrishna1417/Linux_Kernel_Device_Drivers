#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include "WS2812_spi_device.h"

#define DRIVER_NAME	"WS2812_spi_driver"
#define SPI_CLASS_NAME	"WS2812"
#define SPI_DEVICE_NAME "WS2812"
#define SPIDEV_MAJOR	201
#define SPIDEV_MINOR	0


typedef struct spidev_data {
  dev_t			devt;
  struct spi_device	*spi;
  unsigned int			speed_hz;
  struct list_head	device_entry;
  unsigned		users;
}spidev_data;

static struct class *spi_device_class;
static LIST_HEAD(device_list);

uint8_t reset_buf[] = {
  NONE, NONE, NONE, NONE,
  NONE, NONE, NONE, NONE,
  NONE, NONE, NONE, NONE,
  NONE, NONE, NONE, NONE};

static void spidev_complete(void *arg)
{
  /* Signals a single thread waiting on this completion */
	complete(arg);
}

/* Send the message buffer */
static int write_message(spidev_data *p_spidev_data, uint8_t *p_inbuf)
{
  /* declaring and initializing a completion structure */
  DECLARE_COMPLETION_ONSTACK(done);
  int status;
  struct spi_message message;
  struct spi_transfer t = {
    .tx_buf = p_inbuf,
    .len = BUF_LENGTH,
    .speed_hz = p_spidev_data->speed_hz,
  };

  /*  Initializing spi_message */
  spi_message_init(&message);
  /* appending transfers */
  spi_message_add_tail(&t, &message);

  message.complete = spidev_complete;
  message.context = &done;

  if(p_spidev_data->spi != NULL)
  {
    /* asynchronous SPI transfer */
    status = spi_async(p_spidev_data->spi, &message);
    if (status == 0)
    {
      /* waits for completion of the task */
      wait_for_completion(&done);
      status = message.status;
      if (status == 0)
      {
        status = message.actual_length;
      }
    }
  }
  else
  {
    return -EINVAL;
  }
  return status;
}

/*
* Open spi_device_open
*/
static int spi_device_open (struct inode *inode, struct file *file)
{
  spidev_data *p_spidev_data;
  int	status = -ENXIO;

  /* Getting the device data pointer from the global list */
  list_for_each_entry(p_spidev_data, &device_list, device_entry) {
    if (p_spidev_data->devt == inode->i_rdev) {
      status = 0;
      break;
    }
  }
  if (status) {
    printk("Requested device is not present for minor %d\n", iminor(inode));
    return status;
  }

  /* To keep track of number of users using the subsystem */
  p_spidev_data->users++;

  /* Easy access to cmos_devp from rest of the entry points */
  file->private_data = p_spidev_data;

  /*Not to have seekable file descriptors for the subsystem */
  nonseekable_open(inode, file);

  printk ("\n%s is Opening !!! \n", SPI_DEVICE_NAME);

  return status;
}

/*
 * Release spi_device_release
 */
static int spi_device_release (struct inode *inode, struct file *file)
{
  struct spidev_data	*p_spidev_data;
  int	status = 0;

  p_spidev_data = file->private_data;
  file->private_data = NULL;

  /* last close? */
  p_spidev_data->users--;
  if (!p_spidev_data->users) {
    int		dofree;

    p_spidev_data->speed_hz = p_spidev_data->spi->max_speed_hz;

    /* ... after we unbound from the underlying device? */
    dofree = (p_spidev_data->spi == NULL);
    if (dofree)
      kfree(p_spidev_data);
  }

  printk ("\n%s is Closing !!! \n", SPI_DEVICE_NAME);

  return status;
}

/*
 * Write to spi device
 */
static ssize_t spi_device_write (struct file *file, const char __user * buf,
                               size_t len, loff_t * ppos)
{
  struct spidev_data	*p_spidev_data;
  int	status = 0;
  uint8_t *msg = (uint8_t*)kmalloc((BUF_LENGTH*sizeof(uint8_t)),GFP_KERNEL);

  p_spidev_data = file->private_data;
  /*copy object from user */
  if(copy_from_user(msg, (uint8_t *)buf, BUF_LENGTH))
  {
    printk ("Unable to copy_from_user \n");
    return -EACCES;
  }


  status = write_message(p_spidev_data, msg);
  if(status == 0)
    goto done;

done:
  return status;
}

/* SPI ioctl */
static long spi_device_ioctl (struct file *file, unsigned int cmd,
                            unsigned long arg)
{
  struct spidev_data	*p_spidev_data;
  int	status = 0;

  p_spidev_data = file->private_data;

  switch(cmd)
  {
    case RESET:
      /* defining SPI mode to correct 1-wire data transmission */
      p_spidev_data->spi->mode = SPI_MODE_3;
      /* resetting IO pin multiplexing to enable the connection from SPI_MOSI to Data_in of the ring */
      status = gpio_direction_output (24,0);
      if (status != 0)
      {
        printk ("gpio_direction_output FAILED with return %d \n", status);
        return status;
      }
      status = gpio_direction_output (44,1);
      if (status != 0)
      {
        printk ("gpio_direction_output FAILED with return %d \n", status);
        return status;
      }
      status = gpio_cansleep (72);
      if (status != 0)
      {
        gpio_set_value_cansleep (72,0);
      }
      else
      {
        gpio_set_value (72,0);
      }
      status = write_message(p_spidev_data, reset_buf);
      break;
    default:
      printk("Ioctl command not found!\n");
      return -EINVAL;
  }
  return status;
}

/* file operations */
static const struct file_operations spi_device_fops = {
  .owner 	 =	THIS_MODULE,
  .write 	 =	spi_device_write,
  .unlocked_ioctl = spi_device_ioctl,
  .open 	 =	spi_device_open,
  .release =	spi_device_release,
};

static const struct of_device_id spi_device_id_table[] = {
  { .compatible = "rohm,dh2228fv" },
  { }
};

MODULE_DEVICE_TABLE(of, spi_device_id_table);

static int spi_driver_probe(struct spi_device *spi)
{
  spidev_data *p_spidev_data;
  int		status;
  struct device *dev;
  printk(" Found Device -- %s \n", SPI_DEVICE_NAME);

  /* Allocate driver data */
  p_spidev_data = kzalloc(sizeof(*p_spidev_data), GFP_KERNEL);
  if (!p_spidev_data)
    return -ENOMEM;

  /* Initialize the driver data */
  p_spidev_data->spi = spi;

  INIT_LIST_HEAD(&p_spidev_data->device_entry);

  /*
   * Reusing minors is fine so long as udev or mdev is working.
   */

  p_spidev_data->devt = MKDEV(SPIDEV_MAJOR, SPIDEV_MINOR);
  /* Send uevents to udev, so it'll create /dev nodes */
  dev = device_create(spi_device_class, &spi->dev, p_spidev_data->devt,
      p_spidev_data, SPI_DEVICE_NAME);
  status = PTR_ERR_OR_ZERO(dev);

  if (status == 0) {
    list_add(&p_spidev_data->device_entry, &device_list);
  }

  p_spidev_data->speed_hz = spi->max_speed_hz;

  if (status == 0)
  {
    spi_set_drvdata(spi, p_spidev_data);
  }
  else
  {
    printk("Error while creating the device!\n");
    kfree(p_spidev_data);
  }

  return status;
};

static int spi_driver_remove(struct spi_device *spi)
{
  spidev_data	*p_spidev_data = spi_get_drvdata(spi);

  /* make sure ops on existing fds can abort cleanly */
  p_spidev_data->spi = NULL;

  device_destroy(spi_device_class, p_spidev_data->devt);

  if (p_spidev_data->users == 0)
    kfree(p_spidev_data);

  printk("Driver Removed!\n");

  return 0;
};

static struct spi_driver WS2812_driver = {
  .driver		= {
    .name	= "WS2812_spi_driver",
    .owner = THIS_MODULE,
    .of_match_table = of_match_ptr(spi_device_id_table),
  },
  .probe		= spi_driver_probe,
  .remove		= spi_driver_remove,
};

/* configure IO11 pin to be spi MOSI */
static int  set_IO11_configuration(void)
{
  int status = 0;
  status = gpio_request(24, "user_gpio");
  if (status != 0)
  {
    printk ("gpio_request FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_request(44, "user_gpio");
  if (status != 0)
  {
    printk ("gpio_request FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_request(72, "user_gpio");
  if (status != 0)
  {
    printk ("gpio_request FAILED with return %d \n", status);
    goto done;
  }

  status = gpio_direction_output (24,0);
  if (status != 0)
  {
    printk ("gpio_direction_output FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_direction_output (44,1);
  if (status != 0)
  {
    printk ("gpio_direction_output FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_cansleep (72);
  if (status != 0)
  {
    gpio_set_value_cansleep (72,0);
  }
  else
  {
    gpio_set_value (72,0);
  }

  status = gpio_export(24, true);
  if (status != 0)
  {
    printk ("gpio_export FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_export(44, true);
  if (status != 0)
  {
    printk ("gpio_export FAILED with return %d \n", status);
    goto done;
  }
  status = gpio_export(72, true);
  if (status != 0)
  {
    printk ("gpio_export FAILED with return %d \n", status);
    goto done;
  }

done:
  return status;
}

static int __init spi_device_init(void)
{
  int status;

  /* Request dynamic allocation of a device major number */
  status = register_chrdev(SPIDEV_MAJOR, SPI_DEVICE_NAME, &spi_device_fops);
  if (status < 0)
    return status;

  /* Populate sysfs entries */
  spi_device_class = class_create(THIS_MODULE, SPI_CLASS_NAME);
  if (IS_ERR(spi_device_class)) {
    unregister_chrdev(SPIDEV_MAJOR, WS2812_driver.driver.name);
    return PTR_ERR(spi_device_class);
  }

  /* registering the spi driver */
  status = spi_register_driver(&WS2812_driver);
  if (status < 0) {
    class_destroy(spi_device_class);
    unregister_chrdev(SPIDEV_MAJOR, WS2812_driver.driver.name);
  }

  /* Setting the gpio pin configuration for SPI MOSI */
  status = set_IO11_configuration();
  if (status != 0) {
    printk ("set_IO11_configuration FAILED \n");
  }

  return status;
}

static void __exit spi_device_exit(void)
{
  /* free GPIO pins */
  gpio_free (24);
  gpio_free (44);
  gpio_free (72);
  /* Unregistering the spi driver */
  spi_unregister_driver(&WS2812_driver);
  /* Destroy driver_class */
  class_destroy(spi_device_class);
  /* Release the major number */
  unregister_chrdev(SPIDEV_MAJOR, WS2812_driver.driver.name);
}

module_init(spi_device_init);
module_exit(spi_device_exit);
MODULE_AUTHOR ("TEAM20");
MODULE_DESCRIPTION ("SPI PLATFORM DRIVER MODULE");
MODULE_LICENSE ("GPL v2");
