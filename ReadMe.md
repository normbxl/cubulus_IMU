#6 DOF IMU wireless game controller

Cubulus was a student project of mine, which makes use of a cube-shaped controller with an integrated wireless IMU and a receiver. The IMU translates his rotation into a quaternion and transmits to the receiver who then sends it over the serial interface to the computer running Cubulus.  
The game was made with Unity 3.4.2.  
Watch the game footage here: [https://vimeo.com/49825189](https://vimeo.com/49825189).

## Hardware

**game controller cube**  
- Arduino Pro 3.3V
- sparkfun step-up converter
- 2 AAA battery holders
- Sparkfun ITG3200 + ADXL345 I²C daughterboard
- HopeRF RFM12b wireless 433 MHz SPI transceiver 

The gyro- and acceleration-sensors have an I²C interface, so they are connected to A4 and A5 of the Arduino.
  
**receiver**  
- any Arduino running at 3.3V because the RFM12b transceiver has a max voltage rating of 3.8V
- HopeRF RFM12b wireless 433 MHz SPI transceiver

### Connecting the RFM12b

The RFM12b comes with a SPI interface, connect it accordingly:
  
| Arduino    | RFM12b   |  
| -----------|--------- |  
| INT0 (D2)  | IRQ (2)  |  
| SS (D10)   | SEL (8)  |  
| MOSI (D11) | SDI (10) |  
| MISO (D12) | SDO (1)  |  
| SCK (D13)  | SCK (9)  |  

Power, ground and antenna wiring speak for themself.

## Software
Copy the contents of the libraries folder into your arduino sketchfolders library folder.  

Folder Unity contains the C# script as it's used in the game to communicate with the receiver. 
