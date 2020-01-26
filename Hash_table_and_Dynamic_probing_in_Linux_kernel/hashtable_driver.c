#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/hashtable.h>

#include<linux/init.h>
#include<linux/moduleparam.h>
#include"hashtable_ioctl.h"

#define DRIVER_NAME  "ht530_drv"       /* driver/module name to be created and registered */
#define DEVICE_NAME  "ht530-"          /* device name to be created and registered */
/* #define DEBUG_ENABLE */

DEFINE_HASHTABLE (ht530_tbl, 7);

typedef struct hash_node
{
  ht_object_t ht_objectp; /* Hashtable node structure */
  struct hlist_node list; /* Pointer to the next node in the hashtable */
} hash_nodep;

/* per device structure */
struct ht530_tbl_dev
{
  struct cdev cdev;             /* The cdev structure */
  char name[20];                /* Name of device */
  hash_nodep *hash_nodep;
};
struct ht530_tbl_dev ht530_tbl_devp[NUMBER_OF_DEVICES];

int iKey;                       /* Key read from User */
int iErrorNumber;
static dev_t ht530_tbl_dev_number; /* Allotted device number */
struct class *ht530_tbl_dev_class;      /* Tie with the device model */
static struct device *ht530_tbl_dev_device[NUMBER_OF_DEVICES];

/*
* Open ht530_tbl driver
*/
int ht530_tbl_driver_open (struct inode *inode, struct file *file)
{
  struct ht530_tbl_dev *ht530_tbl_devp;

  printk ("\nopening\n");

  /* Get the per-device structure that contains this cdev */
  ht530_tbl_devp = container_of (inode->i_cdev, struct ht530_tbl_dev, cdev);

  /* Easy access to cmos_devp from rest of the entry points */
  file->private_data = ht530_tbl_devp;

  printk ("\n%s is openning \n", ht530_tbl_devp->name);
  return 0;
}

/*
 * Release ht530_tbl driver
 */
int ht530_tbl_driver_release (struct inode *inode, struct file *file)
{
  struct ht530_tbl_dev *ht530_tbl_devp = file->private_data;

  printk ("\n%s is closing\n", ht530_tbl_devp->name);

  return 0;
}

  /*
   * Write to gmem driver
   */
ssize_t ht530_tbl_driver_write (struct file * file, const char *buf,
                                size_t count, loff_t * ppos)
{
  /*bucket number */
  int bkt;

  /* Check for hashtable */
  int iCheck1 = 0;
  int iCheck2 = 0;

  /*New node in hashtable from user input */
  hash_nodep *new_nodep = NULL;

  /*Node used for looping in a hash table */
  hash_nodep *loop_nodep = NULL;

#ifdef DEBUG_ENABLE
  printk ("ENTERING HASHWRITE\n");
#endif

  /*allocate memory for new object to be created and inserted */
  new_nodep = kmalloc (sizeof (struct hash_node), GFP_KERNEL);
  if (!new_nodep)
  {
    printk ("Bad Kmalloc\n");
    return -ENOMEM;
  }

  /*copy object from user */
  if(copy_from_user (&new_nodep->ht_objectp, buf, count))
  {
    printk ("Unable to copy_from_user \n");
    return -EACCES;
  }
#ifdef DEBUG_ENABLE
  printk ("key value %d", (new_nodep->ht_objectp.key));
  printk ("data value %d", (new_nodep->ht_objectp.data));
#endif

  if (hash_empty (ht530_tbl))
  {
    /* Add entry into the hastable if the table is empty */
    hash_add (ht530_tbl, &new_nodep->list, new_nodep->ht_objectp.key);
    iCheck1 = 1;
#ifdef DEBUG_ENABLE
    printk ("HashTable Empty !! Added object at key \n");
#endif
  }
  else
  {
    /* Looping over the complete hashtable */
    hash_for_each (ht530_tbl, bkt, loop_nodep, list)
    {
      if (loop_nodep->ht_objectp.key == new_nodep->ht_objectp.key)
      {
        if (new_nodep->ht_objectp.data != 0)    /*if object with the key exists and data not equal to 0 */
        {
#ifdef DEBUG_ENABLE
          printk
            ("Key present !! Data is not '0', hence replacing the object at key\n");
#endif
          hash_del (&loop_nodep->list);
          hash_add (ht530_tbl, &new_nodep->list, new_nodep->ht_objectp.key);
        }
        else                           /*if object with the key exists and data equal to 0 */
        {
#ifdef DEBUG_ENABLE
          printk
            ("Key present !! Data is '0', hence deleting the object at key\n");
#endif
          hash_del (&loop_nodep->list);
        }
        iCheck2 = 1;
      }
    }
  }
  /* object with the key does not exist */
  if ((iCheck1 == 0) && (iCheck2 == 0))
  {
#ifdef DEBUG_ENABLE
    printk ("New Key !! Added object at key \n");
#endif
    hash_add (ht530_tbl, &new_nodep->list, new_nodep->ht_objectp.key);
  }

#ifdef DEBUG_ENABLE
  hash_for_each (ht530_tbl, bkt, loop_nodep, list)
  {
    if (loop_nodep->ht_objectp.key == new_nodep->ht_objectp.key)
    {
      printk ("Object Exits %d\n", loop_nodep->ht_objectp.data);
    }
  }
#endif

  return 0;
}

/*
 * Read to ht530_tbl driver
 */
ssize_t ht530_tbl_driver_read (struct file * file, char *buf,
                               size_t count, loff_t * ppos)
{
  int bkt;
  int flag = 0;
  int bytes_read = 0;
  hash_nodep *loop_nodep = NULL;

#ifdef DEBUG_ENABLE
  printk ("ENTERING HASHREAD\n");
#endif

  /* searching the hashtable for specified key to read the value */
  hash_for_each (ht530_tbl, bkt, loop_nodep, list)
  {
    if (loop_nodep->ht_objectp.key == iKey)
    {
#ifdef DEBUG_ENABLE
      printk ("key exists%d", loop_nodep->ht_objectp.data);
#endif
      flag = 1;
      break;
    }
  }

  /* If key is not found return -1  and ser error numner EINVAL */
  if (flag == 0)
  {
#ifdef DEBUG_ENABLE
    printk ("Object doesnotExits\n");
#endif
    return -1;
    iErrorNumber = EINVAL;
  }

  /* If key is present copy the values of the object into the buffer */
  if (copy_to_user (buf, &(loop_nodep->ht_objectp.data), sizeof (int)))
  {
    printk ("Unable to copy_to_user \n");
    return -EACCES;
  }
  /*
   * Most read functions return the number of bytes put into the buffer
   */
  bytes_read = sizeof (int);

  return bytes_read;
}

static long ht530_tbl_driver_ioctl (struct file *f, unsigned int cmd,
                                    unsigned long arg)
{
  int NoofObjects = 0;
  hash_nodep *loop_nodep;

  dump_arg dump_arg_in;

  memset (&dump_arg_in, 0, sizeof (dump_arg));
#ifdef DEBUG_ENABLE
  printk ("ENTERING IOCTL \n");
#endif
  switch (cmd)
  {
    case READ_KEY:
      {
        /* Reading key from the user for the read function to print the data at specified key */
        if (copy_from_user (&iKey, (int *) arg, sizeof (int)))
        {
          printk ("Unable to copy_from_user \n");
          return -EACCES;
        }
#ifdef DEBUG_ENABLE
        printk ("KEY FOUND %d \n", iKey);
#endif
        break;
      }
    case DUMP_OBJECTS:
      {
        /* Getting bucket number from the user to dump the values */
        if (copy_from_user (&dump_arg_in, (dump_arg *) arg, sizeof (dump_arg)))
        {
          printk ("Unable to copy_from_user \n");
          return -EACCES;
        }
        /* Looping over the list at specified bucket number and copying the objects to a buffer */
        hlist_for_each_entry (loop_nodep, &ht530_tbl[dump_arg_in.n], list)
        {
          if (NoofObjects < 8)
          {
            dump_arg_in.object_array[NoofObjects] = loop_nodep->ht_objectp;
            NoofObjects++;
          }
          else
          {
            return -1;
#ifdef DEBUG_ENABLE
            printk ("n is outofbound %d \n");
#endif
            iErrorNumber = EINVAL;
          }
        }
        /* Number of elements in the bucket */
        dump_arg_in.n = NoofObjects;
        /* copy the values into the buffer */
        if (copy_to_user ((dump_arg *) arg, &dump_arg_in, sizeof (dump_arg)))
        {
          printk ("Unable to copy_to_user \n");
          return -EACCES;
        }
#ifdef DEBUG_ENABLE
        printk ("Dumped Objects %d \n", NoofObjects);
#endif
        break;
      }

    default:
      return -EINVAL;
  }
#ifdef DEBUG_ENABLE
  printk ("LEAVING IOCTL \n");
#endif
  return 0;
}

/* File operations structure. Defined in linux/fs.h */
static struct file_operations ht530_tbl_fops = {
  .owner = THIS_MODULE,                /* Owner */
  .open = ht530_tbl_driver_open,       /* Open method */
  .release = ht530_tbl_driver_release, /* Release method */
  .write = ht530_tbl_driver_write,     /* Write method */
  .read = ht530_tbl_driver_read,       /* Read method */
  .unlocked_ioctl = ht530_tbl_driver_ioctl      /* IOCTL method */
};

/*
 * Driver Initialization
 */
int __init ht530_tbl_init (void)
{
  int ret;
  int i = 0;

  /* Request dynamic allocation of a device major number */
  if (alloc_chrdev_region (&ht530_tbl_dev_number, 0, 2, DRIVER_NAME) < 0)
  {
    printk (KERN_DEBUG "Can't register device\n");
    return -1;
  }

  /* Populate sysfs entries */
  ht530_tbl_dev_class = class_create (THIS_MODULE, DRIVER_NAME);

  for (i = 0; i < NUMBER_OF_DEVICES; i++)
  {
    /* Request I/O region */
    sprintf (ht530_tbl_devp[i].name, "%s%d", DEVICE_NAME, i);

    /* Connect the file operations with the cdev */
    cdev_init (&ht530_tbl_devp[i].cdev, &ht530_tbl_fops);
    ht530_tbl_devp[i].cdev.owner = THIS_MODULE;

    /* Connect the major/minor number to the cdev */
    ret =
      cdev_add (&ht530_tbl_devp[i].cdev,
          MKDEV (MAJOR (ht530_tbl_dev_number), i), 1);
    if (ret)
    {
      printk ("Bad cdev\n");
      return ret;
    }

    /* Send uevents to udev, so it'll create /dev nodes */
    ht530_tbl_dev_device[i] =
      device_create (ht530_tbl_dev_class, NULL,
          MKDEV (MAJOR (ht530_tbl_dev_number), i), NULL, "%s%d",
          DEVICE_NAME, i);

  }

  printk ("ht530_tbl driver initialized.\n");   // '%s'\n",ht530_tbl_devp->in_string);
  return 0;
}

/* Driver Exit */
void __exit ht530_tbl_exit (void)
{
  int i = 0;

  /* Release the major number */
  unregister_chrdev_region ((ht530_tbl_dev_number), 2);

  /* Destroy device */
  for (i = 0; i < NUMBER_OF_DEVICES; i++)
  {
    device_destroy (ht530_tbl_dev_class,
        MKDEV (MAJOR (ht530_tbl_dev_number), i));
    cdev_del (&ht530_tbl_devp[i].cdev);
  }

  /* Destroy driver_class */
  class_destroy (ht530_tbl_dev_class);

  printk ("ht530_tbl driver removed.\n");
}

module_init (ht530_tbl_init);
module_exit (ht530_tbl_exit);
MODULE_AUTHOR("TEAM20");
MODULE_DESCRIPTION("HASHTABLE DRIVER MODULE");
MODULE_LICENSE ("GPL v2");
