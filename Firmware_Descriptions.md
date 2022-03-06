# Firmware Descrptions

Here you'll find more information about the firmare of each of the boards throughout the car. You can expect to find documentation for things like each function used, a program flow chart, CAN addresses and message content, and more. 

## Diver Control Board
### Overview: 
This board controls all of the hardware that the driver needs to interface with to drive the car, as well as gathers sensor data from the forward sensors. For the driver, this board manages the LCD heads up display and each of the controls that modifies the way the car drives. The coast regeneration and brake regeneration knobs, start button, accelerate and brake pedals all send their data to the Driver Control Board. The HUD that provides information to the driver about the is driven by this board as well. Like all boards on the car, this board communicates with the rest of the vehicle using the CAN protocol. 
### Flowchart:

### Function Definitons:

### CAN Information: 
#### Outgoing Messages:
#### Incoming Messages:
#### Addresses:
0x90: Base - [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
0x91:[,,,,,,]
0x92: []
0x93: []


## Rear Control Board
### Overview: 
### Flowchart:

### Function Definitons:

### CAN Information: 


## High Voltage Board
### Overview: 
### Flowchart:

### Function Definitons:

### CAN Information: 
