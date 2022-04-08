# Firmware Descrptions

Here you'll find more information about the firmware of each of the boards throughout the car. You can expect to find documentation for things like each function used, a program flow chart, CAN addresses and message content, and more. 

## Diver Control Board
### Overview: 
This board controls all of the hardware that the driver needs to interface with to drive the car, as well as gathers sensor data from the forward sensors. For the driver, this board manages the LCD heads up display and each of the controls that modifies the way the car drives. The coast regeneration and brake regeneration knobs, start button, accelerate and brake pedals all send their data to the Driver Control Board. The HUD that provides information to the driver about the is driven by this board as well. Like all boards on the car, this board communicates with the rest of the vehicle using the CAN protocol. 
### Flowchart:

### Function Definitons:
#### pollSensorData:
Input Parameters: void  
Return Type & Value: void  
Description: Reads each sensor that the board monitors and updates the appropriate global variable holding the sensor value.  
### HAL_GPIO_EXTI_Callback
Input Parameters: uint16_t GPIO_PIN
Return Type & Value: void
Description: The IRQ handler for the start button, this will send a ready to drive CAN message to rinehart, enabling the car to drive!

### accel_pedal_compare
Input Parameters: uint8_t pedal0, uint8_t pedal1
Return Type & Value: uint8_t pedal_average (the average pedal value computed)
Description: compares the two samples from the pedal potentiometeres to ensure that the values are within 10 millivolts of eachother to prevent unintended acceleration (very dangerous!)

### CAN Information: 
#### Outgoing Messages:
Front Left Wheel Speed: int 0 - 255 (rpm)  
Front Right Wheel Speed: int 0 - 255  
Front Right Suspension: int 0 - 255 (spring depression scaled to 0 - 255)  
Front Left Suspension: int 0 - 255 (spring depression scaled to 0 - 255)  
Brake 1: 
Brake 2: 
Pedal 1: 
Pedal 2: 

#### Incoming Messages:
Rear Left Wheel Speed: Rear Control Board - int 0 - 255 (rpm)  
Rear Right Wheel Speed: Rear Control Board - int 0 - 255 (rpm)  
Rear Right Suspension: Rear Control Board - int 0 - 255 (spring depression scaled to 0 - 255)  
Rear Left Suspension: Rear Control Board - int 0 - 255 (spring depression scaled to 0 - 255)  
Read to Drive State: High Voltage Board - int 0 or 1 

#### Addresses:
0x90: Base - [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]  
0x91: Prog - [Torque Setting, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]  
0x92: DAQ-DATA - [Left Wheel Speed, Right Wheel Speed, Left Suspension, Right Suspension, Brake 1, Brake 2, Pedal 1, Pedal 2]    
0x93: CONTROL DATA - [Brake Regen, Coast Regen, Cooling, Drive Direction, 0x04, 0x05, 0x06, 0x07]  


## Rear Control Board
### Overview: 
### Flowchart:

### Function Definitons:
#### pollSensorData:
Input Parameters: void  
Return Type & Value: void  
Description: Reads each sensor that the board monitors and updates the appropriate global variable holding the sensor value.

### CAN Information: 

#### Outgoing Messages:
Rear Left Wheel Speed: int 0 - 255 (rpm)  
Rear Right Wheel Speed: int 0 - 255  
Rear Right Suspension: int 0 - 255 (spring depression scaled to 0 - 255)  
Rear Left Suspension: int 0 - 255 (spring depression scaled to 0 - 255)  

#### Incoming Messages:
Brake Light State: Dash Control Board - int 0 or 1
Fan State: Dash Control Board - int 0 or 1

#### Addresses:
0x80: Base - [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]  
0x81: DAQ DATA - [Left Wheel Speed, Right Wheel Speed, Left Suspension, Right Suspension, Water Temperature Inlet, Water Temperature Outlet, 0x06, 0x07]  
0x82: CONTROL DATA - [Brake Signal, Pump Signal, Fan Signal, 0x04, 0x05, 0x06, 0x07]  


## High Voltage Board
### Overview: 
### Flowchart:

### Function Definitons:
#### pollSensorData:
Input Parameters: void  
Return Type & Value: void  
Description: Reads each sensor that the board monitors and updates the appropriate global variable holding the sensor value.

### CAN Information: 
