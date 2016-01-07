# FMU Communications Board

Initial development performed by partnership of [Sentera, LLC](https://sentera.com/) and [University of Minnesota](http://www.uav.aem.umn.edu/).

## System Overview
![System Diagram](/doc/System%20Diagram.jpg?raw=true "System Diagram")

The FMU Communications Board is designed as a component of the Flight Management System.  The FMU Communications Board is a Microchip PIC32 based module with the primary function of managing communications between the FMU Processing Board and the rest of the Flight Management System.  The Ethernet and CAN communication protocols are detailed in file [UMN FMU Communication Protocol](/doc/UMN%20FMU%20Communication%20Protocol.docx).

Besides the described primary function, the FMU Communications Board also:

* Monitors battery voltage.

* Monitors and annunciates status of Flight Management System components.

* Provides an Ethernet interface for:

  * Configuration of CAN Servo-Nodes.
  
  * Testing of CAN Servo-Nodes.
  
  * Monitoring of CAN Servo-Nodes' status and connection to CAN bus.

The software design for the CAN Servo-Node can be found at GitHub repository: [https://github.com/SenteraLLC/dspic33-servo-can-node.X](https://github.com/SenteraLLC/dspic33-servo-can-node.X).

## Project Folder Structure
The project includes the following folders:

>[doc](/doc) - documentation for the project.

>[doxygen](/doxygen) - project file for generating Doxygen documentation.

>[http](/http) - files for definition of HTTP interface.

>[hw](/hw) - hardware design schematics.

>[inc](/inc) - include (i.e. header) code files.

>[ipe](/ipe) - instructions for a device programming method which maintains unique serial numbers.

>[nbproject](/nbproject) - MPLAB X IDE project files.

>[src](/src) – source code files.

## Hardware Overview
The hardware contains the following physical interfaces:

>**Ethernet** - Communication with the FMU Processing Board.  Note: the FMU Communications Board (IP: 192.168.143.130) communicates with the FMU Processing Board (required IP: 192.168.143.11) over port number 55455.

>**UART** - Communication with GPS module.

>**S-Bus** - Communication with radio receiver.

>**SPI** - Communication with IMU module.

>**CAN** - Communication with CAN Servo-Nodes.

>**High-Density Connectors** - Communication with Microhard wireless Ethernet digital data link.

*Note: Power for the FMU Communications Board must be in the range of 3.6V to 36V.*

### Hardware Stackup
![Stackup Build](/doc/Hardware%20Stack.jpg?raw=true "Stackup Build")

The above picture shows a constructed hardware stackup.  The boards on the stackup (from bottom to top) are:

1. FMU Processing Board (BeagleBone Black)
2. BeagleBone Cape - with IMU
3. GPS receiver
4. Sentera Comms Board
5. uHard Datalink

Additional hardware photos can be found on the University of Minnesota [media page](http://www.uav.aem.umn.edu/wiki/Media).

*Note: BeagleBone Cape hardware design included in [hw](/hw) folder.*

## Software Overview
Source code is commented using Doxygen style formatting.  Therefore, Doxygen can be used to generate an easily navigable document which provides greater detail into the software's operation than the overview which is provided here.

### Software Executive
The software implements a preemptive, cyclic executive using eight threads.  The software threads are:

1. **Reset**: Thread is executed following reset.  Within the MPLAB environment this is implemented as "main" which provides C-environment control-flow entry.

2. **Core Timer**: Thread is executed every ~107s and provides a granular time reference for determining relative time.

3. **IMU SPI**: Thread is executed based on SPI operation to service the hardware.

4. **Ethernet SPI**: Thread is executed based on SPI operation to service communication with an internal Ethernet switch.

5. **Temp I2C**: Thread is executed based on I2C operation to service communication with an internal temperature sensor.

6. **GPS UART**: Thread is executed based on UART operation to service the hardware.

7. **S-Bus**: Thread is executed based on UART operation to service the hardware.

4. **Default**: Thread is executed if any unexpected interrupts occur.

All main software processing is performed in the 'Reset' thread.  All other threads have a higher priority (i.e. can preempt the 'Reset' thread) and purely handle servicing of the hardware.  The 'Reset' thread implements cooperative multitasking, where tasks are performed continuously, with the execution time of individual tasks being in the micro-second order of magnitude.  This allows for immediate response to system inputs and a small jitter in system output timing.

### Software Modules
The software is a modular design.  The software modules are explained below, and map directly to [source code](/src) file or folder names:

>**adc**: Analog to Digital Converter (ADC) driver.

>**can**: Controller Area Network (CAN) driver.

>**coretime**: Management of timer for granular relative time.

>**emc1412**: Temperature sensor driver.

>**exceptions**: 'Default' thread instantiation.

>**fmucomm**: Communication definition to/from the FMU Processing Board.

>**i2c**: Inter-Integrated Circuit (I2C) driver.

>**init**: Board initialization routines.

>**ksz8895**: Management of Ethernet switch.

>**oemstar**: Management of message forwarding between the GPS module and the FMU Processing Board.

>**pic32fmu**: Software executive and C-environment control-flow entry.

>**rc**: Radio controller decoding and message forwarding to the FMU Processing Board.

>**sbus**: S-Bus communication driver.

>**snode**: Management of communication and configuration to the CAN bus Servo-Node network.

>**softspi**: Bit-banged Serial Peripheral Interface (SPI) driver.

>**spi**: Serial Peripheral Interface (SPI) driver.

>**status**: Node identification and status annunciation.

>**system_config**: Definition of micro-controller's configuration registers.

>**tcpip**: Definition of HTTP interface and TCP/IP stack.

>**uart**: Universal Asynchronous Receiver Transmitter (UART) driver.

>**util**: Utility functions.

>**vn100**: VectorNav VN100 driver.

## Building the Software
The recommended method for building the software is using tool [MPLAB X IDE](http://www.microchip.com/mplabx/) with compiler [MPLAB XC32](http://www.microchip.com/xc32/).  The IDE and compiler are both free software provided by Microchip.

*Note: v3.0 of MPLAB X IDE was used during development.*

*Note: v1.34 of MPLAB XC32 was used during development.*