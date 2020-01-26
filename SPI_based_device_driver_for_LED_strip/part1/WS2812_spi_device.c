#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#define MY_BUS_NUM 1
static struct spi_device *p_spi_device;

static struct spi_board_info spi_device_info = {
  .modalias = "WS2812_spi_driver",
  .max_speed_hz = 5*1000000, //speed your device (slave) can handle 5MHZ
  .bus_num = MY_BUS_NUM,
  .chip_select = 1,
  .mode = SPI_MODE_3,
};

/**
 * register the device when module is initiated
 */

static int spi_device_init(void)
{
  int ret = 0;
  struct spi_master *master;

  /*To send data we have to know what spi port/pins should be used. This information
    can be found in the device-tree. */
  master = spi_busnum_to_master( spi_device_info.bus_num );
  if( !master ){
    printk("MASTER not found.\n");
    return -ENODEV;
  }

  /* create a new slave device, given the master and device info */
  p_spi_device = spi_new_device( master, &spi_device_info );
  if( !p_spi_device ) {
    printk("FAILED to create slave.\n");
    return -ENODEV;
  }

  p_spi_device->bits_per_word = 8;

  /* setup SPI mode and clock rate */
  ret = spi_setup( p_spi_device );
  if( ret ){
    printk("FAILED to setup slave.\n");
    spi_unregister_device( p_spi_device );
    return -ENODEV;
  }

  return ret;
}

static void spi_device_exit(void)
{
  if( p_spi_device ){
    /* Unregistering the spi device */
    spi_unregister_device( p_spi_device );
  }

  printk(KERN_ALERT "Unregistered the device, Goodbye..!!\n");
}

module_init (spi_device_init);
module_exit (spi_device_exit);
MODULE_AUTHOR ("TEAM20");
MODULE_DESCRIPTION ("SPI PLATFORM DEVICE MODULE");
MODULE_LICENSE ("GPL v2");
