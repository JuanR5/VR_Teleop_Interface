# BotaSerialDriver

This repository is a minimum driver to read sensor data from Bota Systems serial sensors. The driver is consist of a header file and source file. The driver is implemented as abstract class in C++ as it requires access to hardware specifically a serial port and C++ does not offer an abstracted serial port access. For this reason the user will have to derive its own class from the driver and implement the `serialReadBytes()`, `serialAvailable()` functions.
In the examples folder, such implementation can be found for different platforms (Arduino and Linux)

## Usage

### Linux

Compile
```
g++ examples/read_sensor_linux.cpp BotaForceTorqueSensorComm.cpp -o serial_driver
```
Run
```
./serial_driver
```
