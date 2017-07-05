# smart-pool-pump app

## Overview

This application is built around the ESP32 and a custom pcb which implements a pool timer function. The application uses 
aws-shadow libraries to connect to an aws iot thing. It updates the thing by providing pump voltage, current, power,
pressure and temperature. It also allows the user to remotely turn on/off the pump. 
Through a web application the user can control the pump from anywhere. 

Please note the application is still in development.

The application integrates the BMP180 sensor (using code provided by OEM).
It also uses ADE7912 via SPI to measure
AC voltage, and current. Finally it commands GPIO ports to turn AC relays on/off. 

The application also implements a webserver which allows the user
to change the wifi settings.

To provision with AWS IoT, build and flash the app, then go to the
device configuration and finish AWS IoT provisioning. Alternatively,
do `mos aws-iot-setup` from the console.

## How to install this app

- Install and start [mos tool](https://mongoose-os.com/software.html)
- In the fs folder, you will need to put a certificate file, a private key, and root certificate for AWS.
- You also need an AWS policy
- Modify the mos.yml file to put your wifi ssid and password. Also update your aws certificate, public key, and root Certificate files.

<p align="center">
  <img src="https://mongoose-os.com/images/app1.gif" width="75%">
</p>
