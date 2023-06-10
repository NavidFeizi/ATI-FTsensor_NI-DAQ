# ATI-FTsensor_NI-DAQ

This repository contains simple class construte on the ATI C library for reading and processing data from the ATI force/torque sensor using a PCI NI-DAQ card. The library provides functionality to convert the sensor readings to force and torque units and also cancels out the offset.

To use this library, you will need the following:

- A compatible PCI NI-DAQ card
- The NI-DAQmx driver installed on your system
- The ATI force/torque sensor connected to the NI-DAQ card

Please note:
* The calibration file (.cal) needs to be replaced with the correct file according to the sensor's model.
* The channel number may need to be changed based on the physical channel used. For example, to Dev0 ot Dev1 or Dev2.

