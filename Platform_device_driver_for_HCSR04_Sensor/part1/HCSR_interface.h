typedef struct pins
{
  unsigned int echo_pin;
  unsigned int trigger_pin;
}pins;

typedef struct parameters
{
  int m;
  int delta;
}parameters;

typedef struct read_data{
  unsigned int distance;
  unsigned long long timestamp;
}read_data;

#define CONFIG_PINS _IOR(81, 1, pins)
#define SET_PARAMETERS _IOR(91, 2, parameters)

#define INT_TO_FLOAT_CONVERSION(x,fractional_bits) \
  ((float)x / (float)((long)1 << (fractional_bits)))

/* Took this function from linux source v3.19.8 /x86/util/tsc.c*/
static inline unsigned long long rdtsc(void)
{
  unsigned int low, High;
  asm volatile("rdtsc" : "=a" (low), "=d" (High));
  return (((unsigned long long)low) | (((unsigned long long)High) << 32));
}
