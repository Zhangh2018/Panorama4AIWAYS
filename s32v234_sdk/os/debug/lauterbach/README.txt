PREREQUISITES FOR S32V234 DEBUG
===============================

1. Environment variable S32V234_SDK_ROOT pointing to the s32v234_sdk directory (should be set by the installer)
2. Lauterbach T32 V2.3.3 (tested on S.2015.03.000061311)

DEBUG OF S32V234 EVB (standalone)
=================================

1. Open the S32V234.ts2 configuration file
	a. (optional) Or open the T32Start, right click to the blank area, choose "File->Load from file and add" and select this file
	
2. On opened T32Start window, select S32V234 EVB setting and press "Start"
3. Two T32 instances will open, from now, only PowerView for CA53 will be used (all scripts are ran from here)
4. Reset the board (first press SW3 and then SW4 - each for 1 second).
5. Run the script in s32v234/demos/*/build-v234ce-gnu-sa-d/*.cmm to run the specific demo


DEBUG OF S32V234 EVB (Linux)
============================

- From BSP18 the "meltdown" patch is included, which makes 'stop-mode' debugger attachment more difficult.
  As a result, swapper_pg_dir address (virtual and physical) need to be specified accurately. The swapper_pg_dir
  location can differ with each configuration/build of the kernel. The following steps can be used to find it out:
    a) Booting the Linux.
    b) Enabling the Linux awareness with the old (nonworking) configuration.
    c) Break the CPU in idle state (inside kernel).
    d) Run "mmu.dump.pagetable swapper_pg_dir" LTB command to display the location of the swapper_pg_dir -> mentioned
       in the first row.
    e) Use the Virtual and physical addresses from the first row of the above mmu table window to update the
       "MMU.FORMAT LINUXSWAP3 swapper_pg_dir" line in S32V234-CA53_linux.cmm file.
    f) Rerun the Linux awareness and try debugging your application. The break point at main should hit now.
    g) The RCU stall detection makes the debugging difficult. It can be disabled by doing:
         sudo echo 1 > /sys/module/rcupdate/parameters/rcu_cpu_stall_suppress

DEBUGGING T32 ISSUES
====================

- before each application load, the board reset must be issued (both reset buttons, top and bottom pressed in correct order)
- if any warning comes during the script load ("Debugger acts like slave...."), please reset the board and restart the T32 (from the point 2.)
