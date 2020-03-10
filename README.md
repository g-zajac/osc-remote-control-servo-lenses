# Remote lens control with mini servo and OSC

## About The Project

This project solves a problem of remote controling non-motorized lens. Three camera lens rings: focus, aperture and zoom rings are moved by mini servos. The servos are controlled by an arduino with ethernet module. OSC protocol is used for communication with the controller over ethernet cat5. The controler receives motor position and sends back feedback OSC messages: uptime and current servo position.

## Concept
![flow](doc/flow.png)
<!-- replace woith SVG? -->

## Hardware
* ~~arduino nano~~ (not enough flash) moved to teensy LC
* ethernet ENC28J60 module
* PCA9685 I2C servo driver
* 3 x mini servo 180deg (focus, zoom, aperture)
* rotary encoder with RGB led

## Built With
* [Drakon](http://drakon-editor.sourceforge.net) - visual language for flowcahrts
* [Atom](https://atom.io) - An amazing text editor
* [PlatformIO](https://platformio.org) - An ecosystem for embedded development


## License
This project is licensed under [MIT license](http://opensource.org/licenses/mit-license.php)

## Project status
- Prototype under development.
- Firmware under development.
