#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*#include <sys/time.h>
#include <linux/rtc.h>*/
#include <time.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include "hashtable_ioctl.h"
#include "mprobe_ioctl.h"
#include <math.h>

#define NUMBER_OF_THREADS 5

struct arg_struct
{
  int fd0;
  int fd1;
  int kpd;
} args;

pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

void *test_file_operations (void *args)
{
  int i = 0;
  int j = 0;
  int k = 0;
  int res = 0;
  unsigned long long MHz=400*(pow(10,6)); /* MHz of Galileo board */
  struct arg_struct arguments = *((struct arg_struct *) args);
  int iFile = 0;
  int kFile = arguments.kpd;

  ht_object_t *inObj;
  Sring_buffer ring_buffer;
  dump_arg dump_arg_obj;

  inObj = (ht_object_t *) calloc (1, sizeof (ht_object_t));
  for(k=0; k<NUMBER_OF_DEVICES; k++)
  {
    if(k==0)
    {
      iFile=arguments.fd0;
      printf("Testing Device ht530-0 \n");
    }
    else
    {
      iFile=arguments.fd1;
      printf("Testing Device ht530-1 \n");
    }

    for (i = 0; i < 40; i++)
    {
      inObj->key = rand () % 100;
      inObj->data = rand () % 40;
      pthread_mutex_lock (&lock);
      /* writing data into the hashtable */
      res = write (iFile, inObj, sizeof (struct ht_object));
      if (res == sizeof (struct ht_object))
      {
        printf ("Can not write to the device file.\n");
      }
      /* Reading values of probed location from hashtable write using Mprobe module */
      res = read (kFile, &ring_buffer, sizeof (ring_buffer));
      if (res < 0)
      {
        printf ("kprobe read failed.\n");
        return 0;
      }
      printf ("address=%p , pid=%ld , time_Stamp=%lf bucket_number=%d \n", ring_buffer.address,
          ring_buffer.pid, (((double)(ring_buffer.time_stamp))/((double)MHz)), ring_buffer.bucket);
      memset (&dump_arg_obj, 0, sizeof (dump_arg));
      dump_arg_obj.n = ring_buffer.bucket;
      if (ioctl (iFile, DUMP_OBJECTS, &dump_arg_obj) == -1)
      {
        printf ("ERROR \n");
      }
      printf ("No of objects in the bucket %d\n", dump_arg_obj.n);
      for (j = 0; j < dump_arg_obj.n; j++)
      {
        printf (" key %d data %d\n",
            dump_arg_obj.object_array[j].key,
            dump_arg_obj.object_array[j].data);
      }
      pthread_mutex_unlock (&lock);
      sleep(rand()%3);
      /* Setting key to read from hashtable using ioctl */
      if (ioctl (iFile, READ_KEY, &inObj->key) != 0)
      {
        printf ("ERROR \n");
      }
      /* Reading the data from hashtable */
      res = read (iFile, &(inObj->data), sizeof (int));
      if (res != -1)
      {
        printf ("read value from the device %d \n", inObj->data);
      }
      else
      {
        printf ("No value present at specified key\n");
      }
    }
  }
  pthread_exit (NULL);
}

int main (int argc, char **argv)
{
  int i = 0;
  int j = 0;
  int k = 0;
  int fd = 0;
  int count = 0;
  int ret = 0;
  dump_arg dump_arg_obj;
  pthread_t thrd[NUMBER_OF_THREADS];
  unsigned int offset = 0;
  pthread_attr_t tattr;
  struct sched_param param[NUMBER_OF_THREADS];
  int iRTPrioritiy = 20;

  /* open devices */
  args.fd0 = open ("/dev/ht530-0", O_RDWR);
  if (args.fd0 < 0)
  {
    printf ("Can not open device file.\n");
    return 1;
  }

  args.fd1= open ("/dev/ht530-1", O_RDWR);
  if (args.fd1 < 0)
  {
    printf ("Can not open device file.\n");
    return 1;
  }

  args.kpd = open ("/dev/Mprobe", O_RDWR);
  if (args.kpd < 0)
  {
    printf ("Can not open device file.\n");
    return 1;
  }

  offset=0x31; /* Offset should be changed based on the board this program is tested */

  ret = write (args.kpd, &offset, sizeof (int));
  if (ret < 0)
  {
    printf ("kprobe write failed.\n");
    return 0;
  }

  if (pthread_mutex_init (&lock, NULL) != 0)
  {
    printf ("Mutex init has failed\n");
    return 1;
  }

  /* initialized with default attributes */
  ret = pthread_attr_init (&tattr);
  if (ret != 0)
  {
    printf ("Thread Init Failed \n");
  }

  for (i = 0; i < NUMBER_OF_THREADS; i++)
  {
    /* safe to get existing scheduling param */
    pthread_attr_getschedparam (&tattr, &param[i]);
    /* set the priority; others are unchanged */
    param[i].sched_priority = iRTPrioritiy + i;
    /* setting the new scheduling param */
    pthread_attr_setschedparam (&tattr, &param[i]);

    ret =
      pthread_create (&thrd[i], &tattr, test_file_operations, (void *) &args);
    if (ret != 0)
    {
      printf ("Thread Cannot be created :[%s] \n", strerror (ret));
    }
  }
  for (i = 0; i < NUMBER_OF_THREADS; i++)
  {
    pthread_join (thrd[i], NULL);
  }

  for(k=0; k<NUMBER_OF_DEVICES; k++)
  {
    if(k==0)
    {
      fd = args.fd0;
    }
    else
    {
      fd = args.fd1;
    }
    printf("Object dump for device : ht530-%d \n",k);
    count=0;
    /* Dumping the tables */
    for (i = 0; i < 128; i++)
    {
      memset (&dump_arg_obj, 0, sizeof (dump_arg));
      dump_arg_obj.n = i;
      if (ioctl (fd, DUMP_OBJECTS, &dump_arg_obj) == -1)
      {
        printf ("ERROR \n");
      }

      count=0;
      printf (" Bucket Number %d : ", i);
      for (j = 0; j < dump_arg_obj.n; j++)
      {

        printf ("key %d data %d\n",
            dump_arg_obj.object_array[j].key,
            dump_arg_obj.object_array[j].data);
        count++;
      }

      if(count==0)
        printf("EMPTY\n");
    }
  }
  pthread_mutex_destroy (&lock);

  /*kprobe ioctl to unregister kprobe*/
  ret = ioctl (args.kpd, UNREGISTER);
  if (ret < 0)
  {
    printf ("ERROR \n");
  }

  /* closing all the devices */
  close (args.kpd);
  close (args.fd0);
  close (args.fd1);

  return 0;
}
