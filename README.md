# CCDPCIBusTransceiver
Development board for interfacing Chrysler's legacy CCD/PCI-bus.

To be used with [CCDLibrary](https://github.com/laszlodaniel/CCDLibrary) and [J1850VPWCore](https://github.com/laszlodaniel/J1850VPWCore).

![ccdpcibustransceiver_v200_topbottom_render.png](https://chryslerccdsci.files.wordpress.com/2021/03/ccdpcibustransceiver_v200_topbottom_render.png)

PCI-bus RX/TX-pin labels on the PCB are accidentally reversed! MC33390 has its pins named backwards.

This is an attempt to make a hybrid transceiver board to handle CCD-bus (SAE J1567) and PCI-bus (SAE J1850 VPW) communication at the same time.
It is especially useful when a vehicle module gets replaced with an otherwise matching one that uses the other communication bus.
In rare cases the only thing to do is to insert a 2-way gateway between the original vehicle modules and the foreign module.

The development board contains a custom CCD-bus transceiver with simple current drivers switched by N/P-MOSFETs for transmission and a voltage comparator for reception.
PCI-bus is handled by the MC33390 chip.

An external microcontroller is needed, preferably an Arduino Mega, to handle the translations themselves and to provide the J1850 VPW protocol decoding function.

Pinout:

| PCB      | Arduino Mega | OBD2 connector                        |
|----------|--------------|---------------------------------------|
| J1 GND   | GND          |                                       |
| J1 +5V   | +5V          |                                       |
| J1 TBEN  | D4           |                                       |
| J1 TX    | RXD1         |                                       |
| J1 RX    | TXD1         |                                       |
| J1 INT   | D3           |                                       |
| J1 CCD-  |              | J1962-11                              |
| J1 CCD+  |              | J1962-3                               |
| J1 GND   |              | J1962-4                               |
| J2 +12V  |              | J1962-16                              |
| J2 TX    | D10          |                                       |
| J2 RX    | D11          |                                       |
| J2 SLEEP | D5           |                                       |
| J2 4XLP  | D6           |                                       |
| J2 PCI   |              | J1962-2                               |
| J2 GND   |              | J1962-4 (ignore if already connected) |

Example Arduino sketch is coming soon.

As an extension and successor of the [CCDBusTransceiver](https://github.com/laszlodaniel/CCDBusTransceiver) it can be used as CCD-bus transceiver or PCI-bus transceiver only.
