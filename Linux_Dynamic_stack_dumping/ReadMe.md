## Implemented two new syscalls in linux to insert and remove dump_stack function in the execution path of kernel programs.

-- Added dynamic dump_stack option “kernel hacking” menuconfig.
-- By default the CONFIG_DYNAMIC_DUMP_STACK option is set to 'y'
-- If needed change the option by following steps:
   1 run "make menuconfig" in the command line inside kernel directory.
   2 select kernel hacking in the options listed.
   3 select dynamic dump_stack option in the kernel hacking options.
   4 Then change the "compile the kernel with dynamic dump stack" option to Y/N.

-- Testing the patch file
The patch file is generated using the kernel source directory downloaded from dropbox and our modifiled kernel directory. To patch source with our patch file you need to change the path in the patch file or download the patch file to the path where kernel source directory is present and run the following command:
  * patch -p1 < LDSD_patchfile.patch

-- After applying the patch run the following commands in the kernel directory
  * ARCH=x86 LOCALVERSION= CROSS_COMPILE=/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux/i586-poky-linux- make -j4
  * ARCH=x86 LOCALVERSION= INSTALL_MOD_PATH=../galileo-install CROSS_COMPILE=/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux/i586-poky-linux- make modules_install
  * cp arch/x86/boot/bzImage ../galileo-install/
  * cp ../galileo-install/bzImage /path_to_sd_card/
  * reboot the board with the new boot-image

-- Run the makefile given in the assignment directory which creates the "syscall_tester" executable.
   make clean; make
   copy hashtable_driver.ko and syscall_tester to the board.
-- Steps to test the code:
   1 insmod hashtable_driver.ko
   2 ./syscall_tester

-- The Following functions are tested in the test program
 we are inserting kprobe at hashtable write function and kernel printk functions to test the below functionalities:
  * Invalid symbolname
  * Valid symbolname but not in text section
	*	Valid symbolname with dumpmode 0 - dumpstack is inserted only in the owner process
	*	Valid symbolname with dumpmode  1 - dumpstack in inserted for processes which have the same parent process
	*	Insert multiple probes at same location and call the function after removing 1 probe and 2 probes
	*	Removal of the dumpstack by owner process
	*	Removal of the dumpstack by non owner process
