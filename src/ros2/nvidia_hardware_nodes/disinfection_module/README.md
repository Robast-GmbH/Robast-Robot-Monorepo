## Disinfection Module

This directory contains code to interact with the disinfection module

### Disinfection Publisher

This node uses the Jetson.GPIO library to read the voltage on an GPIO pin.

#### Electrical Setup

Currently the GPIO is connected with an 1kΩ resistor (higher values, e.g. 5.1k, caused problems so don't choose this resistor to high) to 3.3V and has a current limiting resistor of 100Ω.
When the disinfection module is triggered, the module closes a switch, which is pulling the GPIO pin to GND.