# Engine Auto Stop Eliminator Firmware for SUBARU Levorg VN5

This repository contains sources for the engine start-stop system eliminator firmware for SUBARU Levorg VN5, based off of the [EngineAutoStopEliminator](https://github.com/kz1000a1/EngineAutoStopEliminator).
This firmware developed with [TWAI driver by Espressif](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html) for compatibility.
Therefore, this firmware may works with most ESP32 based system(M5Stack,M5Atom,ESP32 Dev Module,ESP32C3 Dev Module...)
.

## Safety disclaimer

CAN bus is like a "nervous system" in a car. It is a network that connects various ECUs, sensors, etc. Connecting a new device to this network poses risks such as data corruption, packet losses, etc., that can negatively affect the performance of some or all components of a car.
Same applies to incorrect connections and alternations to the CAN bus wiring. This can cause various undesirable effects, such as "Check engine" lights, electrical and mechanical damage, loss of control, injuries and even death.

By using any code in this project you assume any and all risk, and release any liability from the author.

## Building & Flashing

Firmware builds with [Arduino IDE](https://www.arduino.cc/en/software). 

- Open source code with Arduino IDE. 
- Select board and port.
- Edit code, if GPIO ports are different.
- Compile and upload this code to your Arduino Board.

## License

See LICENSE
