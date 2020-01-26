#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <linux/kernel.h>
#include <sys/syscall.h>
#include <unistd.h>

#define SYS_INSDUMP 359
#define SYS_RMDUMP 360

typedef struct ht_object
{
  int key;
  int data;
} ht_object_t;

void childprocess(int iFile)
{
  ht_object_t *inObj;
  long int res1 = 0, res2 = 0;
  long int res = 0;
  inObj = (ht_object_t *) calloc (1, sizeof (ht_object_t));
  inObj->key = rand () % 100;
  inObj->data = rand () % 40;

  int ret = 0;
  /*Sys call with valid symbolname with mode 0*/
  res1 = syscall(SYS_INSDUMP, "ht530_tbl_driver_write", 0);
  printf("System call sys_insdump returned %ld\n", res1);

  ret= write (iFile, inObj, sizeof (struct ht_object));
  if (ret == sizeof (struct ht_object))
  {
    printf ("Can not write to the device file.\n");
  }

  res2 = syscall(SYS_INSDUMP, "ht530_tbl_driver_write", 0);
  printf("System call sys_insdump returned %ld\n", res2);

  ret = write (iFile, inObj, sizeof (struct ht_object));
  if (ret == sizeof (struct ht_object))
  {
    printf ("Can not write to the device file.\n");
  }

  res = syscall(SYS_RMDUMP, res1);
  printf("System call sys_rmdump returned after removing first kprobe %ld\n", res);

  /*Calling write after removing the first kprobe.It should print dumpstack as second probe is not yet removed*/
  ret = write (iFile, inObj, sizeof (struct ht_object));
  if (ret == sizeof (struct ht_object))
  {
    printf ("Can not write to the device file.\n");
  }

  res = syscall(SYS_RMDUMP, res2);
  printf("System call sys_rmdump returned after removing second kprobe %ld\n", res);

  /*Calling write after removing the 2 kprobes.It should not print dumpstack as second probe is also removed*/
  ret = write (iFile, inObj, sizeof (struct ht_object));
  if (ret == sizeof (struct ht_object))
  {
    printf ("Can not write to the device file.\n");
  }
}

int main (int argc, char **argv)
{
  pid_t   pid;
  long int res;
  int fd0 = 0;
  int iFile=0;
  int ret = 0;
  ht_object_t *inObj;

  /* open devices */
  iFile = open ("/dev/ht530-0", O_RDWR);
  if (iFile < 0)
  {
    printf ("Can not open device file.\n");
    return 1;
  }

  inObj = (ht_object_t *) calloc (1, sizeof (ht_object_t));
  inObj->key = rand () % 100;
  inObj->data = rand () % 40;

  /*Sys call with invalid symbolname*/
  res = syscall(SYS_INSDUMP, "random", 1);
  printf("System call sys_insdump returned for invalid symbolname %ld\n", res);

  /*Sy call with valid symbolname but not in text section*/
  res = syscall(SYS_INSDUMP, "cpu_number", 1);
  printf("System call sys_insdump returned  for valid symbolname but not in text section %ld\n", res);

  /*Adding probe at printk, testing and removing Probe*/
  res = syscall(SYS_INSDUMP, "printk", 0);
  printf("System call sys_insdump returned  for printk %ld\n", res);

  res = syscall(SYS_RMDUMP, res);
  printf("System call sys_rmdump returned  for printk %ld\n", res);

  printf("TEST_PROGRAM: Parent is about to fork child process \n");
  if ((pid = fork()) < 0) {
    printf("Failed to fork process 1\n");
    exit(1);
  }
  else if (pid == 0)
  {
    childprocess(iFile);
  }

  /*Sys call with valid symbolname with mode 1*/
  res = syscall(SYS_INSDUMP, "ht530_tbl_driver_write", 1);
  printf("System call sys_insdump returned %ld\n", res);
  ret = write (iFile, inObj, sizeof (struct ht_object));
  if (ret == sizeof (struct ht_object))
  {
    printf ("Can not write to the device file.\n");
  }

  /*Removing invalid dump stack by the owner process*/
  res = syscall(SYS_RMDUMP, 36);
  printf("System call sys_rmdump returned after trying to remove invalid dumpstack %ld\n", res);


  /*Removing all the dumpstacks at the end of the process*/
  res = syscall(SYS_RMDUMP, 0);
  printf("System call sys_rmdump returned after trying to removing all dumpstacks %ld\n", res);

  /*Trying to call write after removing all probes.Dumpstack must not be printed*/
  ret = write (iFile, inObj, sizeof (struct ht_object));
  if (ret == sizeof (struct ht_object))
  {
    printf ("Can not write to the device file.\n");
  }

  /* closing all the devices */
  close (fd0);
  return 0;
}
