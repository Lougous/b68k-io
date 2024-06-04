# b68k-io
IO extension board for b68k computer.

Features:
- local bus peripheral to b68k-cpu computer
- a UART with DB9 connector (RS232, no hardware flow control)
- an I2C bus with onboard RTC chip (DS1307+) with crystal and backup battery, and 4-pins pin header extension connector
- a SPI bus with 7 chip select lines: 2x SD cards slots, 1x through an 8-pins pin header extension connector, 4x through DB15 connector for external equipements (external SPI slaves are electrically isolated from internal slaves)

Board fonctions are implemented with a PIC18F27K42 microcontroller.
