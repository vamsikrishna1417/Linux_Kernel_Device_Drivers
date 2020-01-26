RUN_ON_GALILEO_BOARD = 1
ifeq ($(RUN_ON_GALILEO_BOARD), 1)

KDIR:=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux/usr/src/kernel
PWD:= $(shell pwd)

CC = /opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux/i586-poky-linux-gcc
ARCH = x86
CROSS_COMPILE = /opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux/i586-poky-linux-
SROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux/

APP = syscall_tester

all:
	$(CC) -pthread -Wall -o $(APP) testapp.c --sysroot=$(SROOT)

clean:
	rm -f *.o
	rm -f $(APP)

  else

  APP = syscall_tester

all:
	gcc -o $(APP) -C test.c

clean:
	rm -f *.o
	rm -f $(APP)

 endif
