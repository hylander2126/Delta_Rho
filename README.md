# Delta_Rho
Code and 3D models for the Delta-Rho swarm robotic platform at WPI's Soft Robotics Laboratory

### General Structure:
The robot currently has a custom PCB housing a ATMega1284p chip. Flashing firmware is done over standard AVRISP programmer using the Microchip Studio software. Firmware is coded in C, and the system is set to operate in a decentralized fashion, however centralized functionality is readily available. Communication is done through Xbee RF modules, both between individual robots and between the robots and the computer. A five-bar force sensor has recently been implemented onboard for measuring force magnitude and direction. The force sensor's code is available at ________________. 


Reach out to Steven Hyland at smhyland@wpi.edu for further questions or concerns.
