Overview
========

This project is a simple demonstration of how to communicate with a car via the OBD-II (on board diagnostic)
connector.  This supports the ISO-9141-2 K-Line interface.  The more modern CAN bus is not supported yet - that's a different project.

This has been tested on a 2005 Toyota Corolla and a 1999 Honda Civic (D16Y7 engine).

This design has been used to read DTC codes out of the Civic.

The [code is here](https://github.com/brucemack/hello-obd2/blob/main/hello-obd2.ino).  This is a C program developed using the Arduino platform.

Hardware
========

Here's what the development platform looks like:

![](images/IMG_1673.jpg)

The purple PCB was designed and manufactured specifically for this application.  Please see [this related project](https://github.com/brucemack/iso9141-interface) for more information about the hardware implementation.

This interface uses the Teensy 3.2 microcontroller.  The Teensy controller is helpful here because it supports more than one hardware serial port.

The K-Line interface is challenging because it sends and receives on the same wire.  This is a 12V interface pulled up by a 560 ohm resistor and pulled down by a 2N3904 NPN transistor.  An extra transistor stage is used as an inverter.  Special arrangements need to be made in software to ignore the "echo" of the transmitted data on the single wire.

One other challenge is that the ISO-9141 specification defines a slow handshake at the start of the connection: the so-called "5 Baud" interface.  Because the Teensy 3.2 (or most any controller) is not capable of generating such a slow transmission rate, we use "bit banging" to generate that 2 second handshake.  A separate I/O pin is used for this purpose to avoid conflicts with the UART.  Please see the "CTL0" input in the schematic below.

Normal transmission happens at 10,400 baud (8/none) as defined in the ISO spec.

Here's the schematic, as simulated in LT-Spice:

![](images/SC1.png)

Here's the initial protoctype:

![](images/IMG_1607.jpg)

A standard SAE J1962 OBD2 connector is used (male).  Pins used are:
* 5 - Signal ground 
* 7 - ISO9141 K-Line
* 12 - +12V

![](images/IMG_1609.jpg)

Software Notes
==============

The ISO document that describes the physical interface is [here](https://andrewrevill.co.uk/ReferenceLibrary/OBDII%20Specifications%20-%20ISO-9141-2%20(Physical).pdf)

Please see [this page](https://en.wikipedia.org/wiki/OBD-II_PIDs) for a list of the PIDs used.


Standards
=========

* Electrical: https://cdn.standards.iteh.ai/samples/16737/e6b719fd44c345a792656f6d19e6cee4/ISO-9141-1989.pdf (note: probably an unauthorized copy)
* Protocol: http://forum.amadeus-project.com/index.php?act=attach&type=post&id=9491 (note: probably an unauthorized copy)
* Good detail: https://www.irjet.net/archives/V4/i7/IRJET-V4I7181.pdf

Example
=======

Here's the initial output as seen on a 2005 Toyota Corolla:

![](images/tc01.png)

Analysis (using this: https://en.wikipedia.org/wiki/OBD-II_PIDs):
* Status:
    * No check-engine light (MIL)
    * No DTCs saved
    * Tests availble for Components, Fuel System, Misfire
    * Tests availble for Oxygen sensor, Evaporative system, at Catalyst
    * No tests are incomplete
* PIDs supported
    * 01 - Status
    * 02 - Freeze DTC
    * 03 - Fuel system status
    * 04 - Engine load
    * 09 - Long term fuel trim (2)
    * 0A - Fuel pressure
    * 0B - Intake manifold pressure
    * 0C - Engine speed
    * 21 - Distance traveled with MIL indicator
    * 22 - Fuel rail pressure
    * 23 - Fuel rail gauge pressure
    * 24 - Oxygen sensor 1


