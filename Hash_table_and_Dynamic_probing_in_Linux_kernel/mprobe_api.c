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
#include <linux/sched.h>
#include<linux/init.h>
#include<linux/moduleparam.h>
#include<linux/kprobes.h>
#include<linux/ptrace.h>
#include <linux/hashtable.h>
#include "mprobe_ioctl.h"
#include <asm/msr.h>
#include <asm/current.h>
#include <linux/version.h>
#include <linux/kallsyms.h>

#define DRIVER_NAME  "Mprobe"          // device name to be created and registered
/* #define DEBUG_ENABLE */
/* #define HOST_PC */

struct mprobe_dev
{
  struct cdev cdev;
} *mprobe_devp;

static struct kprobe kp;

int offset;  /* Offset read from User to keep the probe */
static dev_t mprobe_dev_number; /* Allotted device number */
struct class *mprobe_class;     /* Tie with the device model */
static struct device *mprobe_device;
Sring_buffer ring_buffer;

DEFINE_HASHTABLE (hashtable, 7); /* Temporary hashtable to calculate the bucket number from the key */

static int Pre_Handler (struct kprobe *p, struct pt_regs *regs)
{
  ring_buffer.address = p->addr;
  ring_buffer.pid = current->pid;
  ring_buffer.time_stamp = rdtsc();
  ring_buffer.bucket = hash_min ((*((int *)((regs->ax)+(regs->si)))), HASH_BITS (hashtable));
#ifdef DEBUG_ENABLE
  unsigned int i = 0;
  trace_printk ("pre_handler:p->addr=0x%p, stack values reg=%x \n", p->addr, *((int *)((regs->ax)+(regs->si))));
  for(i=0; i<400; i++)
    printk (" stack values at %d = %lx \n", i, regs_get_kernel_stack_nth(regs, i));
  dump_stack();
#endif
  return 0;
}

static void Post_Handler (struct kprobe *p, struct pt_regs *regs,
                          unsigned long flags)
{
#ifdef DEBUG_ENABLE
  printk(KERN_INFO "Post_Handler: p->addr = 0x%p, ip = %lx,"
      "flags = 0x%lx\n", p->addr, regs->ip, regs->flags);
  dump_stack ();
  printk ("Post_handler: pid = %ld timestamp = %llu, Address =0x%p \n",
      ring_buffer.pid, (ring_buffer.time_stamp), ring_buffer.address);
#endif
}

static int handler_fault (struct kprobe *p, struct pt_regs *regs, int trapnr)
{
  printk ("fault_handler:p->addr=0x%p, eflags=0x%lx\n", p->addr, regs->sp);
  return 0;
}

/*
* Open kprobe driver
*/
int mprobe_open (struct inode *inode, struct file *file)
{
  struct mprobe_dev *mprobe_devp;

  printk ("\nopening\n");

  /* Get the per-device structure that contains this cdev */
  mprobe_devp = container_of (inode->i_cdev, struct mprobe_dev, cdev);

  /* Easy access to cmos_devp from rest of the entry points */
  file->private_data = mprobe_devp;

  printk ("\n%s is openning \n", DRIVER_NAME);
  return 0;
}

/*
 * Release kprobe driver
 */
int mprobe_release (struct inode *inode, struct file *file)
{
  printk ("\n%s is closing\n", DRIVER_NAME);

  return 0;
}

  /*
   * Write to kprobe driver
   */
static ssize_t mprobe_write (struct file *file, const char *buf,
                             size_t count, loff_t * ppos)
{
  int ret = 0;

  /* copy offset from user */
  if(copy_from_user (&offset, buf, sizeof (int)))
  {
    printk ("Unable to copy_from_user \n");
    return -EACCES;
  }
#ifdef DEBUG_ENABLE
  printk ("offset = %d ", offset);
#endif

  kp.pre_handler = Pre_Handler;
  kp.post_handler = Post_Handler;
  kp.fault_handler = handler_fault;
  kp.addr =
    (kprobe_opcode_t *) kallsyms_lookup_name ("ht530_tbl_driver_write") +
    offset;
  if (kp.addr == NULL)
  {
    printk
      (" kallsyms_lookup_name could not find address for the specified symbol name \n");
  }

#ifdef DEBUG_ENABLE
  printk (" probing address = %lx \n", kp.addr);
#endif
  /* registering the probe */
  ret = register_kprobe (&kp);
  if (ret < 0)
  {
    printk (KERN_INFO "register_kprobe failed, returned %d\n", ret);
    return ret;
  }

  return 0;
}

/*
 * Read to kprobe driver
 */
static ssize_t mprobe_read (struct file *file, char *buf,
                            size_t count, loff_t * ppos)
{
  int bytes_read = 0;

  if (copy_to_user (buf, &ring_buffer, sizeof (Sring_buffer)))
  {
    printk ("Unable to copy_to_user \n");
    return -EACCES;
  }
  /*
   * Most read functions return the number of bytes put into the buffer
   */
  bytes_read = sizeof (Sring_buffer);

  return bytes_read;
}

/*Ioctl for mprobe*/
static long mprobe_ioctl (struct file *f, unsigned int cmd, unsigned long arg)
{
  switch (cmd)
  {
    case UNREGISTER:
      {
        /* unregistering the existing probe */
        unregister_kprobe (&kp);
        printk (KERN_INFO "unregistered the kprobe \n");
        break;
      }
    default:
      return -EINVAL;

  }
  return 0;
}

/* File operations st   ructure. Defined in linux/fs.h */
static struct file_operations mprobe_fops = {
  .owner = THIS_MODULE,                /* Owner */
  .open = mprobe_open,                 /* Open method */
  .release = mprobe_release,           /* Release method */
  .write = mprobe_write,               /* Write method */
  .read = mprobe_read,                 /* Read method */
  .unlocked_ioctl = mprobe_ioctl       /* IOCTL method */
};

/*
 * Driver Initialization
 */
int __init mprobe_init (void)
{
  int ret;

  /* Request dynamic allocation of a device major number */
  if (alloc_chrdev_region (&mprobe_dev_number, 0, 1, DRIVER_NAME) < 0)
  {
    printk (KERN_DEBUG "Can't register device\n");
    return -1;
  }

  /* Populate sysfs entries */
  mprobe_class = class_create (THIS_MODULE, DRIVER_NAME);

  /* Allocate memory for the per-device structure */
  mprobe_devp = kmalloc (sizeof (struct mprobe_dev), GFP_KERNEL);
  if (!mprobe_devp)
  {
    printk ("Bad Kmalloc\n");
    return -ENOMEM;
  }

  /* Connect the file operations with the cdev */
  cdev_init (&mprobe_devp->cdev, &mprobe_fops);
  mprobe_devp->cdev.owner = THIS_MODULE;

  /* Connect the major/minor number to the cdev */
  ret = cdev_add (&mprobe_devp->cdev, MKDEV (MAJOR (mprobe_dev_number), 0), 1);
  if (ret)
  {
    printk ("Bad cdev\n");
    return ret;
  }

  /* Send uevents to udev, so it'll create /dev nodes */
  mprobe_device =
    device_create (mprobe_class, NULL,
        MKDEV (MAJOR (mprobe_dev_number), 0), NULL, DRIVER_NAME);

  printk ("mprobe driver initialized.\n");      // '%s'\n",mprobe_devp->in_string);
  return 0;
}

/* Driver Exit */
void __exit mprobe_exit (void)
{
  /* Release the major number */
  unregister_chrdev_region ((mprobe_dev_number), 1);

  /* Destroy device */
  device_destroy (mprobe_class, MKDEV (MAJOR (mprobe_dev_number), 0));
  cdev_del (&mprobe_devp->cdev);
  kfree (mprobe_devp);

  /* Destroy driver_class */
  class_destroy (mprobe_class);

  printk ("mprobe driver removed.\n");
}

module_init (mprobe_init);
module_exit (mprobe_exit);
MODULE_AUTHOR("TEAM20");
MODULE_DESCRIPTION("KPROBE MODULE");
MODULE_LICENSE ("GPL v2");
