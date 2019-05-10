# System and Controls Final Project: Closed Loop Temperature Control System

This project uses an MSP430f5529 microprocessor controller to monitor and adjust the temperature of a 5V voltage regulator measured by a temperature sensor. The desired temperature is set with a potentiometer the microprocessor uses a combination of a PID controller and and a lowpass filter to determine the PWM to output to the fan based on the feedback of the error in temperature. 


## Recreating the System
To replicate the system used,

* Consult the Bill of Materials for the required hardware.
* Using the System Schematic.JPG as reference, rebuild the circuit.
* Review the component explanation video to understand the function of each component.
* Download the main.c file for the MSP430f5529
* Connect the pins of the MSP430f5529 to the circuit as described in the code, or change the names in the main.c to use alternative pins on the board.
* Run the code on the MSP430f5529
* Set a desired temperature by varying the potentiometer.
* Review the video demonstration to compare results.

## **CAUTION the Voltage regulator will get HOT**


