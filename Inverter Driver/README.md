This folder includes code I wrote as part of my work with the Longhorn Racing Electric Formula SAE team. 

inverter_driver.c is code for driver software that would interface with the vehicle's motor inverter. It communicated over CAN bus, and its purpose included reading and transmitting data from the inverter and sending drive and power commands from the rear CPU to the inverter.

rear_cpu_main.c is the main code running on the vehicle's rear CPU that I contributed to. It is included to provide context for the operation of inverter_driver.c.
