#include "HCSR_platform_device.h"

static int iNumberOfInstances = 5;
module_param (iNumberOfInstances, int, S_IRUGO);

MODULE_PARM_DESC (iNumberOfInstances,
                  "Number of intances of HC-SR04 sensors to be created");
HCSR_device_chip *p_HCSR_device_chip;

/**
 * register the device when module is initiated
 */
static void device_release (struct device *dev)
{

}

static int p_device_init (void)
{
  int ret = 0;
  int i = 0;

  p_HCSR_device_chip = (HCSR_device_chip *)
    kzalloc ((iNumberOfInstances * sizeof (HCSR_device_chip)), GFP_KERNEL);
  if (!p_HCSR_device_chip)
    return -ENOMEM;

  for (i = 0; i < iNumberOfInstances; i++)
  {
    sprintf (p_HCSR_device_chip[i].name, "%s%d", DEVICE_NAME, i);

    p_HCSR_device_chip[i].dev_no = i;
    p_HCSR_device_chip[i].plf_dev.name = p_HCSR_device_chip[i].name;
    p_HCSR_device_chip[i].plf_dev.id = -1;
    p_HCSR_device_chip[i].plf_dev.dev.release = device_release;
    /* Register the device */
    platform_device_register (&p_HCSR_device_chip[i].plf_dev);

    printk (KERN_ALERT "Platform device %d is registered in init \n", i);
  }

  return ret;
}

static void p_device_exit (void)
{
  int i = 0;

  for (i = 0; i < iNumberOfInstances; i++)
  {
    platform_device_unregister (&p_HCSR_device_chip[i].plf_dev);
  }
  printk (KERN_ALERT "Goodbye, unregistered the devices \n");
}

module_init (p_device_init);
module_exit (p_device_exit);
MODULE_AUTHOR ("TEAM20");
MODULE_DESCRIPTION ("HCSR PLATFORM DEVICE MODULE");
MODULE_LICENSE ("GPL v2");
