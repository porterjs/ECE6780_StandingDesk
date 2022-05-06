# ECE6780 - Standing Desk Controller

---

## Developers

Jason Porter
Corey Buchanan

---

## Project Description

Embedded systems design for motorized standing desk

*Our presentation file shows the best overview of the project*

This project was created to replace Jason's standing desk controller which no longer works.

It features two motors which give independent control of the table legs, lower limit switches, and a touchscreen interface

## Challenges and features hope to implement in future

Our PCB from Oshpark got lost and didn't deliver on time. We had to quickly assemble the necessary components on a protoboard. We still have the components and can solder it together in the future. The chip going on this PCB also is bluetooth capable, so it would be possible to implement control via phone app.

The motor encoder hardware failed and gave undesirable behavior, making it impossible to implement an accurate targetting mode. We could change this with new encoders in the future.

---

## File structure

A - Preplanning - Contains some of our intial concept design for the system.
B - Equipment Images - Images of the equipment used
C - BOM - Bill of Materials for Project / PCB Design
D - Drawings - Archive of our PCB design files
	- Standing Desk Controller (version2) v31 - PCB Complete (under the New subdirectory) is the one we sent to Oshpark.
E - Submittals - Milestone submissions
F - Programming - HMI and STM32 Code
	- The working STM code is under the Final_Code folder.

## Setup instructions

As is, our PCB design is unproven, but the design files and bill of materials are in the archive as explained above, so it could be ordered from Oshpark or any other PCB manufacturer. We do however, recommend a modification to the design to use relays and an enable transistor instead of the transistor bridge design currently on the PCB. The software will also have to be modified for the chip that is to be soldered on the PCB

Otherwise, if using the STM32Discovery, to run the code files, use the following pins:

PA10 - UART RX from HMI
PA9 - UART TX to HMI

PA8 - Buzzer

PB4 - MTR1 Encoder A
PB5 - MTR1 Encoder B
PB2, PB3 - MTR1 Relays

PA0 - MTR2 Encoder A
PA1 - MTR2 Encoder B
PB7, PB8 - MTR2 Relays

PC0 - Limit Switch 1
PC1 - Limit Switch 2

Our code was modified to exclude the encoder operation as our encoders were broken. That functionality could be added back in.

The motors should be powered using a 24v supply. Hardware assembly can be seen with the in the images of the completed project.

The HMI controller can be programmed via USB with the <FIX> software.