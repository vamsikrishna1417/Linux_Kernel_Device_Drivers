#include<linux/ioctl.h>

typedef struct Sring_buffer
{
  void *address;
  long int pid;
  unsigned long long time_stamp;
  int bucket;
} Sring_buffer;

/* Took this function from linux source v3.19.8 /x86/util/tsc.c*/
unsigned long long rdtsc(void)
{
unsigned int low, High;
asm volatile("rdtsc" : "=a" (low), "=d" (High));
return (low | (((unsigned long long)High) << 32));
}

#define UNREGISTER _IO(71,1)
