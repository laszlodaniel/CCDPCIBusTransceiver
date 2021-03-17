# CCDPCIBusTransceiver
Development board for interfacing Chrysler's legacy CCD/PCI-bus.

To be used with [CCDLibrary](https://github.com/laszlodaniel/CCDLibrary) and [J1850 VPW Arduino Transceiver Library](https://github.com/matafonoff/J1850-VPW-Arduino-Transceiver-Library).

![ccdpcibustransceiver_v200_topbottom_render.png](https://chryslerccdsci.files.wordpress.com/2021/03/ccdpcibustransceiver_v200_topbottom_render.png)

This is an attempt to make a hybrid transceiver board to handle CCD-bus (SAE J1567) and PCI-bus (SAE J1850 VPW) communication at the same time.
It is especially useful when a vehicle module gets replaced with an otherwise matching one that uses the other communication bus.
In rare cases the only thing to do is to insert a 2-way gateway between the original vehicle modules and the foreign module.

The development board contains a custom CCD-bus transceiver with simple current drivers switched by N/P-MOSFETs for transmission and a voltage comparator for reception.
PCI-bus is handled by the MC33390 chip.

An external microcontroller is needed, preferably an Arduino Mega, to handle the translations themselves and to provide the J1850 VPW protocol decoding function.

Pinout description and example Arduino sketch are coming soon.

As an extension of the [CCDBusTransceiver](https://github.com/laszlodaniel/CCDBusTransceiver) it can be used as CCD-bus transceiver or PCI-bus transceiver only.

Available for sale in April 2021.
