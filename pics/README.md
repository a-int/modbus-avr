# Table of contents

1. [Overview](#overview)
2. [Demonstration](#demo)

## Overview <a name="overview"></a>

### What

The focus of this project was to develop a practical knowledge of using Modbus protocol with Atmel ATmega32 microcontroller using C language.

### Why

In today's world, humans are surrounded by intelligent devices and devices that have a certain logic. In addition to the hardware component, every intelligent device requires software to define its operation and link the individual components of the device. 

### How 
In this work it was necessary to write software code for a ATmega32 microcontroller to communicate over Modbus protocol with external device (PC). Supported commands are: 
- 0x01 - read a coil;
- 0x03 - read holding registers;
- 0x04 - read input registers;
- 0x05 - write a coil;
- 0x10 - write holding registers.
The holding registers are used to hold serial number and for 4 bytes for user data. Input registers hold 2 integer words, the first is data for current voltage level calculated by using ADC code and another for current temperature from the same ADC code. The timer-counter #2 is used to support Modbus timings and Timer-counter 1 to update ADC code periodically.

## Demonstration <a name="demo"></a>
<img src="/pics/RW-coils.png" width="600" height="200"/>
<img src="/pics/RW-holding.png" width="600" height="200"/>
<img src="/pics/RW-input.png" width="600" height="200"/>
<img src="/pics/err-checks.png" width="600" height="200"/>
