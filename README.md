# Motion Sensing Glove with ESP32

Dual ESP32-S3 based motion sensing glove system with:

-   5x flex sensors per glove\
-   ICM-20948 9-axis IMU (DMP quaternion mode)\
-   BLE (Nordic UART Service) streaming\
-   Python multi-device BLE receiver

The system streams finger bend data + 3D orientation (roll, pitch, yaw)
in real time to a PC.

------------------------------------------------------------------------

## System Overview

Two independent ESP32 gloves:

-   `ESP32_GLOVE_L`
-   `ESP32_GLOVE_R`

Each glove: - Reads 5 analog flex sensors - Reads IMU quaternion from
DMP over SPI - Converts quaternion → roll/pitch/yaw - Streams formatted
data over BLE - Supports remote calibration via BLE command

Python script connects to both gloves simultaneously.

------------------------------------------------------------------------

## Data Format (BLE Stream)

Each glove continuously sends:

IMU,roll,pitch,yaw,S,r1,r2,r3,r4,r5,N,n1,n2,n3,n4,n5

Where:

-   roll, pitch, yaw → degrees\
-   r1..r5 → raw ADC values\
-   n1..n5 → normalized values (0.0 -- 1.0)

Example:

IMU,4.73,8.50,-86.57,S,1845,1087,1693,1786,1789,N,0.305,0.261,0.030,0.449,0.561

------------------------------------------------------------------------

## Calibration Protocol

Calibration is triggered over BLE:

CAL

Sequence:

1.  5s preparation\
2.  5s hold all fingers bent (max values)\
3.  5s hold all fingers straight (min values)\
4.  Calibration values stored\
5.  Streaming resumes

Other commands:

START\
STOP

------------------------------------------------------------------------

## Repository Structure

motion-sensing-glove-with-esp32/ │ ├── left/ \# Left glove firmware
(PlatformIO) │ ├── src/main.cpp │ └── platformio.ini │ ├── right/ \#
Right glove firmware (PlatformIO) │ ├── src/main.cpp │ └──
platformio.ini │ ├── python/ │ └── read_two_gloves.py \# Dual BLE reader
│ └── README.md

------------------------------------------------------------------------

## Hardware

Microcontroller: - ESP32-S3 (4D Systems Gen4 R8N16)

IMU: - ICM-20948 (SPI mode) - DMP enabled - Game Rotation Vector (Quat6)

Flex Sensors: - 5 analog inputs - 12-bit ADC resolution

------------------------------------------------------------------------

## BLE Architecture

-   Nordic UART Service (NUS)
-   TX → Notify
-   RX → Write / Write Without Response
-   Chunked transmission to avoid MTU overflow

Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E

------------------------------------------------------------------------

## Python Receiver

Install dependencies:

pip install bleak

Run:

python read_two_gloves.py

Features: - Scans for both gloves - Connects simultaneously - Sends
calibration automatically (optional) - Logs to CSV (optional) -
Real-time dual stream monitoring

------------------------------------------------------------------------

## Use Cases

-   VR hand tracking
-   Gesture recognition
-   Robotics teleoperation
-   Motion capture research
-   Rehabilitation monitoring

------------------------------------------------------------------------

## Future Improvements

-   Quaternion streaming instead of Euler
-   BLE packet optimization
-   Sensor fusion refinement
-   Magnetometer integration
-   Kalman filtering
-   3D visualization GUI
