---
title: Firmware: How to start
---

# Firmware: How to start

## 0. Before starting
Follow the instructions to assemble the robot [here](https://rhinestone-star-fa9.notion.site/Robot-Assembly-f31e5685cc46427f9f4b832dc4a56b95). Do not install the propellers.

## 1. Install the robot firmware
For this part, you must have two ESP32 microchips (one with pins and one without pins). The setup looks like the following fitgure
![](https://github.com/LehighBlimpGroup/BlimpSwarm/raw/main/doc/figures/setup.png)

 Follow the following steps to install the firmware on both microchips:
1. Install [Arduino IDE 2.3.0](https://www.arduino.cc/en/software) or latest on your computer.
2. Download [BlimpSwarm](https://github.com/LehighBlimpGroup/BlimpSwarm) into the Arduino/libraries folder.
3. In Arduino open the menu ```File->Examples->BlimpSwarm->Bicopter1raw```.
4. Connect the ESP32 (with pins) to your computer through USB. 
   * Select the device XIAO_ESP32S3
   * Select the port of the ESP32. E.g., COM1 in Windows, /dev/ttyACM0 in Linux, or /dev/tty.usbmodem14301 in MacOS.
5. Click on Upload
6. Click on serial Monitor, and copy the MAC address (e.g., 34:85:18:AC:BB:70). Then, close the Serial Monitor.
7. Disconnect your ESP32 and Repeat the same process to install ```File->Examples->BlimpSwarm->BaseTranseiver``` in the ESP32 (without pinS). That is the ESP32 that will be connected to your computer and act as a radio communicator.




## 2. Control the actuators (servos and motors using the Joystick)

1. Connect the Joystick to the computer
2. Edit the Python file BlimpSwarm/swarmbase/RawBicopterJoystick.py
```
    Update the ROBOT_MAC of your robot
    Update the port of the ESP32 (same as the one in Arduino)
```
3. Run the file ```RawBicopter.py```. 
4. Move the left vertical joystick. 
   * The LED in your ESP32 should turn on when you move the vertical joystick