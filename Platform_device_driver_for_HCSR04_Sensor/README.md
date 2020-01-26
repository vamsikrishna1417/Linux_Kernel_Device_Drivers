# Device_driver_for_HCSR04_Sensor
Developed a platform driver/platform device infrastructure which initiates a Linux miscellaneous character device driver interface with multiple instances of Ultrasonic sensor(HC-SR04) and allows the sensors to be accessed as device files. Configured the trigger and echo pins of sensor to Linux GPIO pins using Linux GPIO pin multiplexing and measured the distance from multiple HC-SR04 devices using kernel FIFO buffer concurrently using kernel timer and GPIO interrupts which was tested using user-dev application program. Also enabled sysfs interface for these platform devices and tested using a bash-script.