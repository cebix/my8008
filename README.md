# my8008
Arduino-based memory emulator for Intel 8008 processors

## What is this?
This is a small Arduino IDE project for building a working Intel 8008 system using an original 8008-1 chip and an Adafruit ItsyBitsy M4 microcontroller board. The M4 interfaces to the bus of the 8008, providing clock and reset signals, emulating 16K of RAM memory, and implementing a virtual UART to allow 8008 code to communicate with the outside world. It also includes a simple BASIC interpreter as a demo application for the 8008.

The [Intel 8008](https://en.wikipedia.org/wiki/Intel_8008) was the world's first commercially available 8-bit microprocessor, introduced in April 1972. Chip packages were still limited in size at the time, so the 8008 comes in a rather tiny 18-pin package with a highly multiplexed bus interface which, using conventional 74-series logic, requires some quite extensive circuitry to decode. Nowadays you would put this into an FPGA, but because I'm a programmer at heart I decided to throw software at the problem. :wink:

## Hardware
This is not an 8008 emulator. You need a real 8008 chip to build this project, specifically the 8008-1 variant which runs at a clock speed of 800 kHz. Although I guess it can be made to run with the earlier 8008 versions clocked at 500 kHz by changing the source code, function ```initPhiClock()```. Despite its age this chip is not super-rare today; you can still find it for sale on auction sites for about EUR 100. You may be able to obtain one cheaper by scavenging for data terminals or terminal interface cards for 70s/80s minicomputer systems (PDP-11 and the like). That's how I originally got mine, anyway.

The 8008 is connected to the ItsyBitsy M4 as shown in this diagram:
![my8008 connection diagram](/images/schematics.png)

Note that the 8008 requires a −9V power rail which needs to be supplied externally. The 3.3V and 5V rails can in theory be provided by the M4 via its USB connection, but you may get more stable operation if you also use a dedicated 5V power supply for the 8008 side. The 8008 draws about 60mA during operation.

The 8008 uses 5V TTL logic levels while the M4 uses 3.3V for most signals, so some level shifting is required between them. Any level shifters designed for use with I2C buses should be fine. I used simple [bi-directional level shifters](https://www.penguintutor.com/electronics/mosfet-levelshift) based on discrete MOSFETs. Signal integrity seems OK for the intended purpose, but more sophisticated designs are also possible.

The circuitry also uses the RX/TX serial lines of the M4 which are intended to be connected to an external USB UART adapter (I used an MCP2221A part which I had lying around for this purpose, although any 3.3V TTL adapter should work fine). This serial connection is passed through to the 8008 side by the emulator, so 8008 programs can use it to send and receive characters. The integrated USB UART device of the M4 is used for controlling the software running on the M4.

The whole system is small enough to be built on a breadboard:
![my8008 built on a breadboard](/images/breadboard.jpg)
(blue = bus data lines, green = bus state lines, white = clock signals, purple = interrupt signal)

## Operation
You need two serial terminals to operate the system:
* One conneced to the M4 via its on-board USB port. This is used for flashing the software to the M4, and for monitoring and controlling the execution of the 8008 code.
* A second one connected via the external USB UART adapter to interact with the running 8008 program.

Both terminals use a data rate of 115200 bit/s. This can be changed in the source code, function ```initSerial()```.

After flashing the program to the M4 and powering up the system, the Intel 8008 is supplied with clock signals but initially held in reset state. The on-board RGB LED of the M4 lights up in orange in this state, while the M4 waits for commands issued over its on-board serial connection (the first terminal mentioned above).

The M4 runs a little monitor program which allows you to examine the contents of the 16K of emulated RAM memory, and disassemble 8008 code:
```
h              Show list of commands
g              Run 8008 program
m xxxx [yyyy]  Dump memory
d xxxx [yyyy]  Disassemble
```

There are no provisions to load code or data to memory yet. The code to be run by the 8008 needs to be defined in the source code of the ```my8008.ino``` file (look for the definition of the ```mem[]``` array). There are some examples provided, including a "Hello world" program, a UART echo routine, and a copy of the [SCELBAL BASIC interpreter](http://www.willegal.net/scelbi/scelbal.html) which has been modified by me to use the emulated UART of my8008 for I/O, and the source code of which is included in the [scelbal](/scelbal) directory. When trying the BASIC interpreter, make sure that the ```SCELBI``` define in the source code is active to enable the special ASCII character code conversion needed by the interpreter. For the other demo programs this macro should be undefined.

Enter the ```g``` command to take the 8008 out of reset and start execution at address 0. The RGB LED of the M4 lights up in green while the 8008 is running code. During this time, the M4 cannot be controlled via the USB serial terminal any more. In fact, the serial device may disconnect from your PC (I had to disable USB interrupt handling due to timing conflicts with the 8008 bus emulation).

When the 8008 executes a ```HLT``` instruction, the M4 stops the 8008, switches the LED back to orange, and re-enters monitor mode so you can examine memory contents or re-run the 8008 program. When the ```TRACE``` define in the source code is active you will get a traceback of the last cycles of 8008 bus activity, which can be useful for debugging dodgy wiring or bus timing issues.

When my8008 detects an error in the expected sequence of bus states of the 8008, it will halt and show a ```seq error``` message on the control terminal. This can be caused by several things:
* Incorrect wiring between the M4 and the 8008
* Signal integrity issues, probably caused by the level shifters
* Power supply issues
* Bus timing issues on the M4 side

## Future Enhancements
* Extend the monitor program to allow uploading code to the 8008 RAM.
* Implement breakpoints and single-stepping features.
* Rework this project to use a board based on an RP2040 microcontroller, such as the Raspberry Pi Pico. Its PIO coprocessor is perfect for implementing the lower levels of the 8008 bus interface, and its dual-core design would make the emulator controllable while the 8008 is running.

## Resources
* [The Intel 8008 support page](https://petsd.net/8008.php)
* [8008/SCELBI Applications](https://www.willegal.net/scelbi/apps8008.html)
