This folder contains code I wrote as a member of Longhorn Racing Electric for the control systems of our vehicles.

The inverter driver is a piece of software I wrote originally for our 2016 vehicle "Danger Zone". It interfaces with the vehicle's Tritium Wavesculptor200 motor inverter via CAN bus and is executed by the vehicle's Rear CPU board. The software supports reading status messages and data from the inverter and sending drive and power commands from the Rear CPU to the inverter to move the motor. The code is comprehensive and robust, capable of handling every flag and error that could be raised by the inverter. Because of this, it was able to be seamlessly ported over to our 2018 vehicle "Leap of Faith" without requiring any modifications.

The Rear CPU main code is a program for Leap of Faith for which I was a primary contributor. As the name suggests, it is the main code running on the vehicle's Rear CPU, which is the main board of the control system and essentially the "brain" of the car. It determines the vehicle's current state and behaviors by traversing a software state machine based on the information it receives from other boards throughout the vehicle. The state machine consists of 6 states: 

1) DISCONNECT - The board is receiving power and begins verifying communication with other critical areas of the car.

2) SLEEP - The GLV system is working correctly, but the tractive system has not been powered on.

3) PRECHARGE - The tractive system is first enabled, and the battery pack begins precharging the motor inverter.

4) IDLE - The inverter is precharged successfully, and the BMS is now fully active, meaning the battery is supplying high         voltage. The vehicle is ready to begin driving.

5) DRIVE - The driver has successfully initiated the Ready-to-Drive sequence, and Rear CPU begins sending power commands to the motor inverter based on received pedal data, driving the car.

6) DEAD - The control system software has encountered a fatal error. The shutdown relay is opened, shutting down the vehicle's tractive system, and the vehicle is forced to a halt for the protection of the driver and the car.

The code is also responsible for controlling various smaller items and subsystems throughout the vehicle; it activates the brake light when it detects the brakes have been applied, and it supports control of the cooling system through switches located on the dashboard.
