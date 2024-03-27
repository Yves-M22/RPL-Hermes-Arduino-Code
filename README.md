# RPL-Hermes-Arduino-Code
## General Information/Updates
Code still needs to be finished and tested before moving hardware out of the breadboard. Consists of multiple sensors connected to an Arduino Mega.  

*3/27/24 - Further improvements with SD reader, added a button to close file*. 

*3/26/24 - Code updated to work with Arduino Mega*. 

*3/24/24 - GitHub made for code*.

BMP 280: Measures temperature, pressure, and altitude - Finished. 

MPU 6050: Measures acceleration and rotation - Finished.

Adafruit Breakout GPS: Tracks location coordinates - Mostly finished.

MicroSD Card Reader: Saves data recorded onto a microSD card - Seems to be working, but needs continued testing since the card reader seems to be finicky. 

Xbee Transistors: Allows data recorded to be transmitted via radio transmission - Need to determine how it will work and how the sender and recipient Arduino (Ground Station) will communicate with each other.

Notes: The output format could be made more compact if necessary to reduce the amount of data being transmitted and written onto SD. Data being transmitted and written will be around 300 bytes currently every 3 seconds.  

## Breadboard Testing

![Components on a breadboard](/images/testing.jpg "Arduino Mega")

![Fritzing Diagram](/images/fritzing.png "Wiring Diagram")
