# Sandsara
Sandsara Firmware
## Features version 1.0.0
- Commands for Bluetooth communication. 
- Interpolation for THR file is implemented.
- RGB LEDs Control is implemented.
- .txt .bin and .thr files can be read via sd card.
- routine for motors calibration to find the initial position. 
- computes inverse and direct kinematics.
- Interpolation of a straight line for point separate for more than 1 mm.
- Hallo and Stelle working areas are restricted.
- the rgb^2 algorithm is implemented to blend the color palettes.
- Motor Thread is implemented for the control of the motors.
- LEDs Thread is implemented for the control of the LEDs.
- BLE comunication is implemented.
- Important parameters of Sandsara are store in EEPROM.
- Testing class is implemented to verify the status of the components.
- acceleration and deceleration to prevent abrupt movements in the motors.

## Project compilation
in order to compile as easy as possible, we use platformIO and Visual Studio Code. The platformio.ini contains all the dependencies. Follow the next steps
1. Download Visual Studio Code.
2. Install the PlatformIO extension.
3. Clone the repository.
4. open the folder repository in VS Code.
5. Follow the considerations.
6. Press "platformIO: Build".
7. done.

## Considerations
It is important to modify the next file.

modify the next lines in the HardwareSerial.cpp file (this file is part of the Core of the ESP32)  
`line 10: #define RX1 9 change to #define RX1 26`  
`line 14: #define TX1 10 change to #define TX1 27`

## License

<a rel="license" href="http://creativecommons.org/licenses/by-nc-nd/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-nd/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-nd/4.0/">Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License</a>

