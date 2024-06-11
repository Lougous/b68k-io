# b68k-io
IO extension board for b68k computer.

## Features
- local bus peripheral to [b68k-cpu](https://github.com/Lougous/b68k-cpu) computer
- a UART with DB9 connector (RS232, no hardware flow control)
- an I2C bus with onboard RTC chip (DS1307+) with crystal and backup battery, and 4-pins pin header extension connector
- a SPI bus with 7 chip select lines: 2x SD cards slots, 1x through an 8-pins pin header extension connector, 4x through DB15 connector for external equipements (external SPI slaves are electrically isolated from internal slaves)

Board fonctions are implemented with a PIC18F27K42 microcontroller.

The board has only two memory locations accessible through the expansion bus: one to set the actual register to access, the other to read or write data from/to the register.

| register | address | description             |
|:--------:|:-------:|:------------------------|
| DATA     | XX0000h | register data |
| ADDRESS  | XX0001h | register index |

## Register set

| register    | index   | description             |
|:-----------:|:-------:|:------------------------|
| SPI_SEL     | 0       | [5..3] : enabled SPI chip select line<br>0: external #0<br>1: external #1<br>2: external #2<br>3: external #3<br>4: SD card A<br>5: SD card B<br>6: internal pin header<br>7: none (reset) |
| SPI_CTRL    | 1       | bit 7: BUSY   r<br>bit 2: CLRBF  s<br>bit 1: TXR    rw<br>bit 0: RXR    rw |
| SPI_DATA    | 2       | SPI data in/out |
| UART_STS    | 4       | UART status<br>bit 7: TX ready<br>bit 1: RX FIFO overflow<br>bit 0: RX FIFO underflow |
| UART_DATA   | 5       | UART data in/out |
| I2C_CTRL    | 6       | bit 7: RXIF<br>bit 5: TXBE<br>bit 2: reset<br>bit 1: start<br>bit 0: RXBF |
| I2C_CNT     | 7       | |
| I2C_DATA    | 8       | |
| I2C_CMD     | 9       | bit 7..1: address<br>bit 0:    RWn |
| IRQCFG      | 14      | interrupt controller configuration<br>bit 0: enable UART RX interrupt |
| IRQSTS      | 15      | interrupt controller status<br>bit 0: UART RX interrupt pending |
