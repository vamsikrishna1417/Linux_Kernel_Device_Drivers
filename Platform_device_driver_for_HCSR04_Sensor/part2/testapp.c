#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <math.h>

#define CONFIG_PINS _IOR(81, 1, pins)
#define SET_PARAMETERS _IOR(91, 2, parameters)
#define INT_TO_FLOAT_CONVERSION(x,fractional_bits) \
  ((float)x / (float)((long)1 << (fractional_bits)))

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


int main (int argc, char **argv)
{
  int *fd;
  int iNumberOfInstances = 0;
  char dev_name[50] = "";
  parameters operation_params[2];
  pins config_pins[2];
  int in_write_arg1 = 0;
   int in_write_arg2 = 0;

  float distance = 0.0;
  double timestamp = 0.0;
  read_data sReadData[2];

  FILE *fp =
    fopen ("/sys/module/HCSR_platform_device/parameters/iNumberOfInstances",
        "r");

  if (!fp)
  {
    printf ("Module is not installed \n");
    return 0;
  }
  if (fscanf (fp, "%d", &iNumberOfInstances) == 1)      /* read/validate value */
  {
    printf ("iNumberOfInstances: %d\n", iNumberOfInstances);
  }

  fclose (fp);

  /* creating file descriptors for all the devices */
  fd = (int *) calloc (iNumberOfInstances, sizeof (int));

  /* open device 1 */
  sprintf (dev_name, "/dev/HCSR_0");
  fd[0] = open (dev_name, O_RDWR);
  if (fd[0] < 0)
  {
    printf ("Can not open device file.\n");
    return 0;
  }

  /* open device 2 */
  sprintf (dev_name, "/dev/HCSR_1");
  fd[1] = open (dev_name, O_RDWR);
  if (fd[1] < 0)
  {
    printf ("Can not open device file.\n");
    return 0;
  }


  printf
    ("ENTER THE PIN NUMBERS TO CONFIGURE ECHO AND TRIGGER RESPECTIVELY FOR DEVICE 1 \n");
  scanf ("%u", &config_pins[0].echo_pin);
  scanf ("%u", &config_pins[0].trigger_pin);

  printf
    ("ENTER THE PIN NUMBERS TO CONFIGURE ECHO AND TRIGGER RESPECTIVELY FOR DEVICE 2 \n");
  scanf ("%u", &config_pins[1].echo_pin);
  scanf ("%u", &config_pins[1].trigger_pin);

  /* Setting pin configuration for all the device 1 */
  if (ioctl (fd[0], CONFIG_PINS, &config_pins[0]) < 0)
  {
    printf ("ERROR IN IOCTL CONFIG PINS \n");
    return 0;
  }

  /* Setting pin configuration for all the device 2 */
  if (ioctl (fd[1], CONFIG_PINS, &config_pins[1]) < 0)
  {
    printf ("ERROR IN IOCTL CONFIG PINS \n");
    return 0;
  }

  
 /*SCENARIO 1 Both the devices*/
	printf("Scenario1 device 1\n");
  if (read (fd[0], &sReadData[0], sizeof (read_data)) < 0)
  {
    printf ("ERROR IN READ TESTAPP \n");
    return 0;
  }

	printf("Scenario1 device 2\n");
  /* Calling read when the buffer is empty */
  if (read (fd[1], &sReadData[1], sizeof (read_data)) < 0)
  {
    printf ("ERROR IN READ TESTAPP \n");
    return 0;
  }

  /* read the measurement triggered from read function */
  if (read (fd[0], &sReadData[0], sizeof (read_data)) < 0)
  {
    printf ("ERROR IN READ TESTAPP \n");
    return 0;
  }
  else
  {
    /* using Qformat 6 to store the distance inside kernel */
    distance = INT_TO_FLOAT_CONVERSION (sReadData[0].distance, 6);
    timestamp = (double) sReadData[0].timestamp / (double) 400000000;
    printf ("read distance for device 1 after read triggered a measurement %f cm\n", distance);
    printf ("read timestamp for device 1 after read triggered a measurement %lf sec \n", timestamp);
  }

  /* read the measurement triggered from read function */
  if (read (fd[1], &sReadData[1], sizeof (read_data)) < 0)
  {
    printf ("ERROR IN READ TESTAPP \n");
    return 0;
  }
  else
  {
    /* using Qformat 6 to store the distance inside kernel */
    distance = INT_TO_FLOAT_CONVERSION (sReadData[1].distance, 6);
    timestamp = (double) sReadData[1].timestamp / (double) 400000000;
    printf ("read distance for device 2 after read triggered a measurement %f cm\n", distance);
    printf ("read timestamp for device 2 after read triggered a measurement %lf sec \n", timestamp);
  }

  
  /*Scenario2  Both the devices*/
  printf("Scenario2 device1\n");
  in_write_arg1 = 0;
  /* writing into the buffer without clearing */
  if (write (fd[0], &in_write_arg1, sizeof (int)) < 0)
  {
    printf ("ERROR IN WRITE TESTAPP \n");
    return 0;
  }

  printf("Scenario2 device2\n");
  in_write_arg2 = 0;
  /* writing into the buffer without clearing */
  if (write (fd[1], &in_write_arg2, sizeof (int)) < 0)
  {
    printf ("ERROR IN WRITE TESTAPP \n");
    return 0;
  }

  /* Reading the value from fifo buffer of device 1 */
  if (read (fd[0], &sReadData[0], sizeof (read_data)) < 0)
  {
    printf ("ERROR IN READ TESTAPP \n");
    return 0;
  }
  else
  {
    /* using Qformat 6 to store the distance inside kernel */
    distance = INT_TO_FLOAT_CONVERSION (sReadData[0].distance, 6);
    timestamp = (double) sReadData[0].timestamp / (double) 400000000;
    printf ("read distance for device 1 after write %f cm\n", distance);
    printf ("read timestamp for device 1 after write %lf sec \n", timestamp);
  }


  /* Reading the value from fifo buffer of device 2 */
  if (read (fd[1], &sReadData[1], sizeof (read_data)) < 0)
  {
    printf ("ERROR IN READ TESTAPP \n");
    return 0;
  }
  else
  {
    /* using Qformat 6 to store the distance inside kernel */
    distance = INT_TO_FLOAT_CONVERSION (sReadData[1].distance, 6);
    timestamp = (double) sReadData[1].timestamp / (double) 400000000;
    printf ("read distance for device 2 after write %f cm\n", distance);
    printf ("read timestamp for device 2 after write %lf sec \n", timestamp);
  }

  
  /*Scenario3 Both the devices*/
    printf("Scenario3 device1\n");
  in_write_arg1 = 1;
  /* writing into the buffer without clearing */
  if (write (fd[0], &in_write_arg1, sizeof (int)) < 0)
  {
    printf ("ERROR IN WRITE TESTAPP \n");
    return 0;
  }
  printf("Scenario3 device2\n");

  in_write_arg2 = 1;
  /* writing into the buffer without clearing */
  if (write (fd[1], &in_write_arg2, sizeof (int)) < 0)
  {
    printf ("ERROR IN WRITE TESTAPP \n");
    return 0;
  }

  /* Reading the value from fifo buffer of device 1 */
  if (read (fd[0], &sReadData[0], sizeof (read_data)) < 0)
  {
    printf ("ERROR IN READ TESTAPP \n");
    return 0;
  }
  else
  {
    /* using Qformat 6 to store the distance inside kernel */
    distance = INT_TO_FLOAT_CONVERSION (sReadData[0].distance, 6);
    timestamp = (double) sReadData[0].timestamp / (double) 400000000;
    printf ("read distance for device 1 after write %f cm\n", distance);
    printf ("read timestamp for device 1 after write %lf sec \n", timestamp);
  }


  /* Reading the value from fifo buffer of device 2 */
  if (read (fd[1], &sReadData[1], sizeof (read_data)) < 0)
  {
    printf ("ERROR IN READ TESTAPP \n");
    return 0;
  }
  else
  {
    /* using Qformat 6 to store the distance inside kernel */
    distance = INT_TO_FLOAT_CONVERSION (sReadData[1].distance, 6);
    timestamp = (double) sReadData[1].timestamp / (double) 400000000;
    printf ("read distance for device 2 after write %f cm\n", distance);
    printf ("read timestamp for device 2 after write %lf sec \n", timestamp);
  }
  
  
  /*Scenario4 Both the devices*/
  printf("Scenario4 for both the devices\n");
  /* setting parameters for the measurement and testing ioctl */
  printf
    ("ENTER THE OPERATION PARAMETERS FOR HC-SR04 M AND DELTA RESPECTIVELY FOR DEVICE 1 \n NOTE: DELTA SHOULD BE MORE THAN 60 \n");
  scanf ("%d", &operation_params[0].m);
  scanf ("%d", &operation_params[0].delta);

  printf
    ("ENTER THE OPERATION PARAMETERS FOR HC-SR04 M AND DELTA RESPECTIVELY FOR DEVICE 1 \n NOTE: DELTA SHOULD BE MORE THAN 60 \n");
  scanf ("%d", &operation_params[1].m);
  scanf ("%d", &operation_params[1].delta);

  if (ioctl (fd[0], SET_PARAMETERS, &operation_params[0]) < 0)
  {
    printf ("ERROR IN IOCTL SET PARAMS \n");
    return 0;
  }

  /* setting parameters for the measurement and testing ioctl */

  if (ioctl (fd[1], SET_PARAMETERS, &operation_params[1]) < 0)
  {
    printf ("ERROR IN IOCTL SET PARAMS \n");
    return 0;
  }

  /* Reading the value from fifo buffer of device 1 */
  if (read (fd[0], &sReadData[0], sizeof (read_data)) < 0)
  {
    printf ("ERROR IN READ TESTAPP \n");
    return 0;
  }
  else
  {
    /* using Qformat 6 to store the distance inside kernel */
    distance = INT_TO_FLOAT_CONVERSION (sReadData[0].distance, 6);
    timestamp = (double) sReadData[0].timestamp / (double) 400000000;
    printf ("read distance for device 1 after ioctl %f cm\n", distance);
    printf ("read timestamp for device 1 after ioctl %lf sec \n", timestamp);
  }


  /* Reading the value from fifo buffer of device 2 */
  if (read (fd[1], &sReadData[1], sizeof (read_data)) < 0)
  {
    printf ("ERROR IN READ TESTAPP \n");
    return 0;
  }
  else
  {
    /* using Qformat 6 to store the distance inside kernel */
    distance = INT_TO_FLOAT_CONVERSION (sReadData[1].distance, 6);
    timestamp = (double) sReadData[1].timestamp / (double) 400000000;
    printf ("read distance for device 2 after ioctl %f cm\n", distance);
    printf ("read timestamp for device 2 after ioctl %lf sec \n", timestamp);
  }
  /* closing all the devices */
  close (fd[0]);
  close (fd[1]);

  return 0;
}
