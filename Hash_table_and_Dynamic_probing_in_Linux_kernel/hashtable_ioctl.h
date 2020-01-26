#include <linux/ioctl.h>

typedef struct ht_object
{
  int key;
  int data;
} ht_object_t;

typedef struct dump_arg
{
  int n;
  ht_object_t object_array[8];
} dump_arg;

#define NUMBER_OF_DEVICES 2
#define READ_KEY _IOR(81, 1, int *)
#define DUMP_OBJECTS _IOWR(91,2,dump_arg)
