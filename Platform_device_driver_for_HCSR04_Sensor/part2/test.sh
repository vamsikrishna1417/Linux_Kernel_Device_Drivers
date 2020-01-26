#!/bin/bash

echo "***TESTING WITH ENABLE AS 1***"
echo "ENTER THE TRIGGER PIN FOR DEVICE 1"
read trigger
echo "$trigger" > /sys/class/HCSR/HCSR_0/trigger

echo "ENTER THE ECHO PIN FOR DEVICE 1"
read echo_pin
echo "$echo_pin" > /sys/class/HCSR/HCSR_0/echo

echo "ENTER THE TRIGGER PIN FOR DEVICE 2"
read trigger
echo "$trigger" > /sys/class/HCSR/HCSR_1/trigger

echo "ENTER THE ECHO PIN FOR DEVICE 2"
read echo_pin
echo "$echo_pin" > /sys/class/HCSR/HCSR_1/echo

echo "ENTER THE NUMBER OF SAMPLES FOR DEVICE 1"
read number_of_samples
echo "$number_of_samples" > /sys/class/HCSR/HCSR_0/number_samples

echo "ENTER THE NUMBER OF SAMPLES FOR DEVICE 2"
read number_of_samples
echo "$number_of_samples" > /sys/class/HCSR/HCSR_1/number_samples

echo "ENTER THE SAMPLING PERIOD FOR DEVICE 1"
read sampling_period
echo "$sampling_period" > /sys/class/HCSR/HCSR_0/sampling_period

echo "ENTER THE SAMPLING PERIOD FOR DEVICE 2"
read sampling_period
echo "$sampling_period" > /sys/class/HCSR/HCSR_1/sampling_period

echo "1" > /sys/class/HCSR/HCSR_0/enable
echo "1" > /sys/class/HCSR/HCSR_1/enable

echo " DISTANCE CALCULATED FOR DEVICE 1 = $(cat /sys/class/HCSR/HCSR_0/distance)cm"
echo " DISTANCE CALCULATED FOR DEVICE 2 = $(cat /sys/class/HCSR/HCSR_1/distance)cm"

echo "***TESTING WITH ENABLE AS 0***"

echo "0" > /sys/class/HCSR/HCSR_0/enable
echo "0" > /sys/class/HCSR/HCSR_0/enable

echo " DISTANCE CALCULATED FOR DEVICE 1 = $(cat /sys/class/HCSR/HCSR_0/distance)cm"
echo " DISTANCE CALCULATED FOR DEVICE 2 = $(cat /sys/class/HCSR/HCSR_1/distance)cm"
