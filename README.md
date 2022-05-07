# ECE6780 - Standing Desk Controller

---

## Developers

Jason Porter
Corey Buchanan

---

## Project Description

I was donated this desk last year because it wasn’t functioning properly.  After some investigation, I discovered that the motors were working, but the control board had failed.  Here is my desk which I rigged with some spare motor starters and push buttons.  Although this worked, it had no safety features, automatic positioning, or speed control.  This made a great opportunity for an embedded systems project.

![IMG_2692](https://user-images.githubusercontent.com/19315982/167232393-c4316b30-5477-4dfa-b993-ba316ce1f40b.jpg)

The desk features two motors which give independent control of the table legs, lower limit switches, and a touchscreen interface.

See our presentation slides for greater detail on our project development which is found in the root folder.  Our final schematic designs are found in our "Drawings" folder and our HMI and STM32 code is located in "Programs."

## Challenges and features hope to implement in future

Our PCB from Oshpark got lost and didn't deliver on time. We had to quickly assemble the necessary components on a protoboard. We still have the components and can solder it together in the future. The chip going on this PCB also is bluetooth capable, so it would be possible to implement control via phone app.

The motor encoder hardware failed and gave undesirable behavior, making it impossible to implement an accurate targetting mode. We could change this with new encoders in the future.

---

## File structure

A - Preplanning 
- Contains some of our intial concept design for the system. 

B - Equipment Images 
- Images of the equipment used

C - BOM 
- Bill of Materials for Project / PCB Design

D - Drawings 
- Archive of our PCB design files
- Standing Desk Controller (version2) v31 
- PCB Complete (under the New subdirectory) is the one we sent to Oshpark.

E - Submittals 
- Milestone submissions

F - Programming 
- HMI and STM32 Code
- The working STM code is under the Final_Code folder.

G - Result Media 
- Photos and the demo of our results.

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

The motors should be powered using a 24v supply. Hardware assembly can be seen with the images of the completed project.

The HMI controller can be programmed via USB with a copy of the Nextion Editor software.


## Methods

Tearing apart the desk for a closer look at the telescoping leg, we found that the motors were rated 20V, and had a built-in incremental encoder with two hall-effect sensors.    I believe the original controller used a current sensor to detect the homing position, but we instead considered a limit switch.  There isn’t a great way to detect overtravel when extending without implementing some kind of pull-cable, so we decided the encoders would limit our stroke length. 
The design challenge was to control the motors in a way to overcome the weight of the desk and any supported equipment.  The weight will not be evenly distributed, so having feedback to maintain a level surface would be ideal.  
The motors did not have nameplates, so we don’t know the current rating.  However, we loaded the motor and measured the inline current which was about 4.3A at 24Vdc.  We used this information to spec our motor controller hardware.
Entering our design stage, our main components, other than the motors and microcontroller, were 
-	Inductive NPN Proximity Sensors to act as our limit switches for each leg.
-	A programmable HMI touchscreen with a native UART protocol
-	And an alarm buzzer to signal overtravel.


Preliminary testing was done for proof of concept.  Each limit switch interfaces with an optocoupler in order to separate the 24V circuit from the 3.3V MCU inputs.  Also note that we implemented the proxy sensors with a fail-safe strategy.  If the limit switch loses power, the input signal goes to logic LOW, preventing the desk from lowering.

We ran into some issues with the H-bridge.  The original design utilized the L298N, but they were back ordered so we had to find another solution.  We decided to try making an H-bridge using BJT transistors.  This was successful, but even in an unloaded state, the transistors heated up.  We also required Darlington pairs to amplify the MCU max output current of 20mA to 4.5A.  This required careful selection of base transistors.  This design required 8 transistors and 2 PWM outputs per H-bridge.  Ultimately, to simplify the design, we replaced the transistors with two relays.  Because the relays have a 10ms switching speed, we added a Darlington NPN pair in series and added an enable PWM input.  This resolved our heating issue and simplified our design.

Without a PCB, we decided to split the master board into two boards: one for the proxy sensors and the other for the motor controls.  Here is the final protoboard for the proxy sensors.  

![IMG_2666](https://user-images.githubusercontent.com/19315982/167232841-b92a5c31-d217-44db-bb0b-4126c64667c4.jpg)

And here is the motor control board.  This was a tight fit, but we managed to get all the components on the board.  Each side has a full H-bridge.  The enable transistors did get a little warm during loaded testing, so we added heat sinks as a precaution.

![IMG_2676](https://user-images.githubusercontent.com/19315982/167232830-d8ec011d-9a3b-4e22-93bf-7c31d51f1e3d.jpg)

Each motor had a PWM output for the enable transistors. The transistors have a minimum switching frequency of 2Mhz.  The motor’s time constant would be in the order of milliseconds, so a 25us PWM period was sufficient.

Early on we were able to determine that at full speed, the quadrature controller count was one tick per 3.3mS (unloaded).  This was far to slow to implement a reliable PID speed controller.  So, we just used a set speed with ramp up to reduce torque stress.  But we still planned on using the quadrature control to position the height of the desk.  

Unfortunately, the encoder was behaving erratically which resulted in a migrating position counter.  We discovered that we were no longer getting clean signals as we did during preliminary testing.  The signals had induced spiking.  Relocating the encoder wires to a separate board and using a separate power source helped a little, but we were still getting too much noise, so we had to disable our overtravel feature. 

Overall, we were able to meet all our milestones with the exception of our custom PCB and the failed encoders.  That being said, all the code was implemented and validated, but some of it was commented out for the final demo due to the encoder issues.  

