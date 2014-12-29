## Introduction

This is an Eclipse project for the TM4C123XL LaunchPad from Texas Instruments (TI). It uses the ARM Cortex-M Software Interface Standard (CMSIS) project style and structure and supports Open On-Chip Debug (OpenOCD) for debugging. The C compiler, assembler, and loader are from the GNU Compiler Collection (gcc). All of the software used is open-source and free.

Note that this project definition is _not_ intended to be compatible with the
TivaWare package supplied by TI, and that the TM4C-specific files supplied for CMSIS
compatibility are not provided, maintained, or endorsed by Texas Instruments or ARM.

Files that support CMSIS projects:

  - Processor peripheral memory map definition header file (__<tt>TM4C123GH6PM.h</tt>__)
  - System clock configuration and initialization (__<tt>system\_TM4C.c</tt>__)
  - C run-time startup (__<tt>startup\_TM4C.s</tt>__)

Files that support debugging with OpenOCD:

  - Eclipse launcher for the OpenOCD gdb server
  - Eclipse launcher for gdb

## CMSIS

The part of the CMSIS that is most specific to the TM4C123 processor is the
"core peripheral access layer header file", which defines the memory map for
all of the on-chip peripheral devices as well as their interrupt vectors.
The header file supplied with this project is specifically written for the
TM4C123GH6PM processor and is therefore named __<tt>TM4C123GH6PM.h</tt>__
This header file was adapted from similar header files generated for similar
processors. To the best of my knowledge, this header file is consistent with
the peripheral register definitions provided in the TI data sheet for the
TM4C123GH6PM processor. However, I have not made an explicit attempt to verify
every definition.

The __<tt>system\_TM4C.c</tt>__ and __<tt>system\_TM4C.h</tt>__ files provides
one global variable and two functions that are required by the ARM CMSIS. The
variable, __<tt>SystemCoreClock</tt>__, is a global variable whose value is
equal to the processor core clock in hertz. This variable can be updated by
calling the __<tt>SystemCoreClockUpdate</tt>__ function. The most important
function, __<tt>SystemInit</tt>__, configures and initializes the processor.
This consists mainly in configuring the system clock source and the phase-locked
loop. There are many options for the clocks in a TM4C processor, and the user
should review the compile-time constants defined in __<tt>system\_TM4C.c</tt>__
before using this file with any system other than the TM4C123XL LaunchPad.

The __<tt>startup\_TM4C.s</tt>__ file defines all of the interrupt service
routines and provides the C runtime initialization. The steps in initializing
the processor are:

  - Enable the Usage, Bus, and Memory Management Fault exceptions to make
    debugging a little easier
  - Clear the <tt>bss</tt> segment. This is the part of RAM that holds static variables that lack an explicit, non-zero initial value.
  - Copy the <tt>data</tt> segment from flash to RAM. This is the part of RAM that holds static variables with explicit, non-zero initial values.
  - Call the CMSIS __<tt>SystemInit</tt>__ function to setup the clocks.
  - Call the user's __<tt>main</tt>__ function.

Note that the __<tt>main</tt>__ function should never end or execute a
<tt>return</tt> statement. If it does, the processor will sit in an infinite
loop that is the last instruction in the C runtime initialization. Here's an
example of a main program using the CMSIS style of peripheral access:

    #include "TM4C123GH6PM.h"

    #define BIT4 (1 << 4)
    #define PORTE (1 << 4)

    int main()
    {
      uint32_t Increment = 0;

      // Enable clock to GPIO E, wait for port to be ready
      SYSCTL->RCGCGPIO |= PORTE;
      while ((SYSCTL->PRGPIO & PORTE) == 0) {
      }

      // Configure PE4 as an output pin
      GPIOE->DIR |= BIT4;
      // Configure PE4 for 8mA output
      GPIOE->DR8R |= BIT4;
      // Enable digital function for PE4
      GPIOE->DEN |= BIT4;
      
      // Loop forever, blinking the LED
      for (;;) {
        Increment++;
        if (Increment >= (SystemCoreClock >> 4)) {
          Increment = 0;
          GPIOE->DATA ^= BIT4;
        }
      }
      return 0;
    }

## OpenOCD

There are two Eclipse launchers that are used to debug code running on a
LaunchPad. The first, __<tt>OOCDWinSrvrTM4C123GH6PM.launch</tt>__, simply starts
the OpenOCD executable as a gdb server. A command line switch is used to specify
that the target board is a TM4C123GXL LaunchPad. If the project is imported
correctly you should see an entry for this launcher under the "external tools"
menu item in the debug perspective.

The second launcher, __<tt>OOCDgdbWin.launch</tt>__, starts gdb itself. (If the
project is imported correctly you should see an entry for this launcher under the
"debug" menu item in the debug perspective.)
A
sequence of commands is then sent from gdb to the gdb server in order to wake
up the target TM4C processor, program its flash, and start execution of the
user code. This launcher expects the compiled target code to be in a file named
__<tt>main.elf</tt>__ within a project named __<tt>MyProject</tt>__. Once you
have imported the project you can use the Eclipse menus to rename it and this
change will automatically be made to the launcher as well. However, if your
executable code is in a file other than __<tt>main.elf</tt>__ then you will
need to manually edit the launcher.

## Dependencies

This project has been configured for use under Windows, where all of the
important software has been installed in a directory named __<tt>C:\\armtools</tt>__

However, the project may be used under Linux (and probably OSX) by changing the
search path definition in the project preferences and by changing the
hard-coded paths in the two debug launchers.

### GNU tools and utilities

The Eclipse project preferences add these two values to the search path for executable software:

  - __<tt>C:\\armtools\\gnutools\\bin</tt>__
  - __<tt>C:\\armtools\\yagarto-tools\\bin</tt>__

The launcher for gdb (__<tt>OOCDgdbWin.launch</tt>__) contains a hard-coded path for
the GNU debugger. It expects to find the __<tt>gdb</tt>__ executable file at

  - __<tt>C:\\armtools\\gnutools\\bin\\arm-none-eabi-gdb.exe</tt>__

#### gcc

The __<tt>gnutools</tt>__ directory must contain the executable files and
libraries needed by gcc and related tools. Pre-built versions of gcc for
ARM Cortex-M processors are maintained and provided by ARM employees
[here.](https://launchpad.net/gcc-arm-embedded "GNU Tools for ARM Embedded
Processors") Note that when you download and install the software it
will be in a directory that has a version-dependent name, such as
__<tt>gcc-arm-none-eabi-4\_7-2013q1</tt>__. This directory has simply been
renamed __<tt>gnutools</tt>__ so that upgrading the gcc version does not break
the pointers and hard-coded paths in the Eclipse settings and launchers.

#### Utilities

The __<tt>yagarto-tools</tt>__ directory contains a few utility programs including
__<tt>make</tt>__ and __<tt>rm</tt>__. The Eclipse project includes a
manually-written make file, so there must be an executable version of
__<tt>make</tt>__ available somewhere in the executable search path. The make
file executes __<tt>rm</tt>__ when you clean the project so this program must
also be available.

This version of the Eclipse project uses the versions of these utilities that
were provided by Michael Fischer as part of yagarto-tools, but this package
is no longer being developed. It appears that they have been archived as
<http://www.emb4fun.de/archive/gabmt/download/emb4fun-tools-20140920-setup.exe>
but I have not verified this. It is not necessary to use special versions of
these utilities when developing ARM software, so if your system already has them
then you simply need to make sure that they can be found along the search path
used by Eclipse.

The make file provided with the project expects that the top-level
__<tt>main</tt>__ function will be provided in a C source file named
__<tt>main.c</tt>__. All C and assembly source files should be kept in the
project's __<tt>src</tt>__ directory and all C header files should be in the
__<tt>inc</tt>__ directory. The only source file suffixes recognized by the
make file are __<tt>.c</tt>__, __<tt>.s</tt>__, and __<tt>.h</tt>__.

### OpenOCD

The launcher for the OpenOCD gdb server (__<tt>OOCDWinSrvrTM4C123GH6PM.launch</tt>__)
defines the path to the OpenOCD executable to be

  - __<tt>C:\\armtools\\openocd\\bin-x64\\openocd-x64-0.8.0.exe</tt>__

If you have OpenOCD install along a different path then you must manually modify the launcher.

The documentation and source code for OpenOCD is available at
[sourceforge.](http://openocd.sourceforge.net/ "Open On-Chip Debugger")
Linux and OSX users can probably obtain a recent version from the
software repositories for their distribution, and Freddie Chopin
makes pre-compiled Windows versions of OpenOCD available at [his web
page.](http://www.freddiechopin.info/en/download/category/4-openocd "OpenOCD
Downloads") Make sure you have a version that supports the TI ICDI interface
(version 0.7.0 or higher), and that the file __<tt>ek-tm4c123gxl.cfg</tt>__
exists in the board scripts directory.

### USB drivers

Under Windows it will probably be necessary to manually install the
drivers for the USB debugging interface to the LaunchPad. There are two
drivers that must be installed to accommodate debugging via ICDI and a
third that enables the virtual COM port from the LaunchPad. The required
drivers and installation information can be obtained from TI on [this
page.](http://www.ti.com/tool/stellaris_icdi_drivers "Stellaris ICDI Drivers")


