Overview
========
The Module migration example application provides flash configuration files for the different modules built around the RW61x.
The code prints the "Board migration test." header then the selected board. The status of the LittleFS initialization and finally it prints the number of times the example has booted. This count is kept in flash.

SDK version
===========
- Version: 2.16.000

Toolchain supported
===================
- MCUXpresso  11.10.0

Hardware requirements
=====================
- Micro USB cable
- FRDM-RW612 board
- Personal Computer

Board settings
==============
No special settings are required.

Prepare the Demo
================
1.  Connect a micro USB cable between the PC host and the MCU-Link USB port (J10) on the board.
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Launch the debugger in your IDE to begin running the example.

Running the demo
================
The log below shows the output of the demo in the terminal window:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Board migration test.
Board selected: FRDM-RW612
Flash and littleFS drivers initialization complete.
boot_count: 1 (Read from Flash)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
