# ELEC3020
Project for ELEC3020, interfacing a model car with a microcontroller and implement actuating steering and drive system.

## TODO
- [X] Connect your embedded controller to the selected model car.
- [ ] Implement “drive-by-wire” from the controller for steering and drive system
- [X] Connect a GPS sensors to the model car
- [ ] Implement a program to drive the car to a location given by GPS-coordinates
and then come back to the start.

## Parts used  

### RC car/parts
[Tamiya 58694 Toyota GR 86 RC Kit (TT-02)](https://hobbytechtoys.com.au/products/tamiya-58694-toyota-gr-86-rc-kit-tt-02)  

[Hitec HS-311 Standard Servo 3.7kg](https://hobbytechtoys.com.au/collections/servos/products/hitec-hs-311-standard-servo-3-7kg)  

### Chips/Electronics
[Arduino Nano or variant of clone](https://core-electronics.com.au/nano-v3-0-board.html)  

[U-blox NEO-6M GPS Module](https://core-electronics.com.au/u-blox-neo-6m-gps-module.html)  

[LCD Keypad Shield For Arduino](https://core-electronics.com.au/lcd-keypad-shield-for-arduino.html)  

[Male Pin Header 2.54mm (1x20)](https://core-electronics.com.au/header-male-pin-01x20.html)  

### PCB  

[ProjectRC.zip](https://github.com/Bustanutrhyme/ELEC3020/files/9694906/ProjectRC.zip)

### Build
```make all``` to compile all files.
```make (filename)``` filenames: gps/motor/servo to compile individual files.
### Clean
```make clean``` to clean up all files.

