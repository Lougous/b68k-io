
// PIC18F24K42 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator Selection (HS (crystal oscillator) above 8 MHz; PFM set to high power)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be cleared and set only once)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT enabled regardless of sleep)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTC = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// CONFIG5H

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include <stdint.h>
#include <stdio.h>
#include <pic18f27k42.h>
//#include <string.h>

#define _XTAL_FREQ       64000000UL  // 16 MHz (default configuration)

// debug UART
void putch(unsigned char data) {
    while( ! U1ERRIRbits.TXMTIF)          // wait until the transmitter is ready
        continue;

    U1TXB = data;                     // send one character
}

#define CEn              (PORTBbits.RB0)

//#define RSTn             (PORTBbits.RB3)
//#define ASSERT_RSTn      TRISBbits.TRISB3 = 0;   // output - RST# asserted
//#define DEASSERT_RSTn    TRISBbits.TRISB3 = 1;   // input - RST# deasserted
//
#define ACKn             (PORTCbits.RC4)
#define ASSERT_ACKn      TRISCbits.TRISC4 = 0;   // output - ACK# asserted
#define DEASSERT_ACKn    TRISCbits.TRISC4 = 1;   // input - ACK# deasserted

#define IRQn             (PORTCbits.RC3)
#define ASSERT_IRQn      TRISCbits.TRISC3 = 0;   // output - ACK# asserted
#define DEASSERT_IRQn    TRISCbits.TRISC3 = 1;   // input - ACK# deasserted

#define WEn              (PORTCbits.RC2)

#define A0               (PORTCbits.RC1)

#define SPR              (PORTCbits.RC5)

// RB3, RB4, RB5
#define SET_SEL(val)     LATB = ((LATB & ~0x38) | (val & 0x38))
#define GET_SEL()        (LATB & 0x38)


// IRQ management
volatile unsigned char irq_cfg;
volatile unsigned char irq_sts;

#define IRQ_SERIAL_MASK  0x01

volatile unsigned char uart_rxbuf[256];
volatile unsigned char uart_rxb_w;
volatile unsigned char uart_rxb_r;
volatile unsigned char uart_sts;

#define UART_RUDF_BIT    0x01

// interrupt handler for serial input
void __interrupt (irq(U1RX),high_priority) _Interrupt(void) {
    uart_rxbuf[uart_rxb_w++] = U1RXB;

    //uart_sts &= ~UART_REMPTY_BIT;
    irq_sts |= IRQ_SERIAL_MASK;
    
    if (irq_cfg & IRQ_SERIAL_MASK) {
        ASSERT_IRQn;
    }
}

// shift: first is RB3
#define CS_EXT0    (0 << 3)
#define CS_EXT1    (1 << 3)
#define CS_EXT2    (2 << 3)
#define CS_EXT3    (3 << 3)
#define CS_SDC     (4 << 3)  // internal connector (J11)
#define CS_SDA     (5 << 3)  // SDcard (J7)
#define CS_SDB     (6 << 3)  // SDcard (J2)
#define CS_NONE    (7 << 3)

void main(void) {
    
    ////////////////////////////////////////////////////////////////////////////
    // pins configuration
    ////////////////////////////////////////////////////////////////////////////
    // enable digital mode for all IOs
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    
    // local bus
    DEASSERT_IRQn;
    DEASSERT_ACKn;
    LATCbits.LATC4 = 0;     // GND = ACK# asserted (when output)
    LATCbits.LATC3 = 0;     // GND = IRQ# asserted (when output)
    
    // chip select
    SET_SEL(CS_NONE);  // no chip
    // note: while input, pull-ups oper
    TRISBbits.TRISB3 = 0;   // output - SEL0
    TRISBbits.TRISB4 = 0;   // output - SEL1
    TRISBbits.TRISB5 = 0;   // output - SEL2

    ////////////////////////////////////////////////////////////////////////////
    // config & status
    ////////////////////////////////////////////////////////////////////////////
    irq_cfg = 0;
    irq_sts = 0;
    
    uart_rxb_w = 0;
    uart_rxb_r = 0;
    uart_sts   = 0;
    
    ////////////////////////////////////////////////////////////////////////////
    // chip configuration
    ////////////////////////////////////////////////////////////////////////////
    OSCCON1 = 0x70;

    ////////////////////////////////////////////////////////////////////////////
    // SPI configuration
    ////////////////////////////////////////////////////////////////////////////
    // SCK : RB2
    TRISBbits.TRISB2 = 0;   // output
    RB2PPS = 0b011110;
    // MOSI : RB1
    TRISBbits.TRISB1 = 0;   // output
    RB1PPS = 0b011111;
    // MISO : RC0
    TRISCbits.TRISC0 = 1;   // input
    SPI1SDIPPS = 0b010000;
    
    SPI1BAUD = 160;  // 400kbps TBC
    SPI1CLK = 0;  // FOSC

    SPI1CON1bits.SMP  = 0;  // SDI input is sampled in the middle of data output time
    SPI1CON1bits.CKE  = 0;
    SPI1CON1bits.CKP  = 1;  // idle SCK is high
    SPI1CON1bits.FST  = 0;
    SPI1CON1bits.SSP  = 0;  // not used, anyway
    SPI1CON1bits.SDIP = 0;  // no polarity inversion
    SPI1CON1bits.SDOP = 0;  // no polarity inversion

    SPI1CON2bits.RXR  = 1;  // enable receiver
    SPI1CON2bits.TXR  = 1;  // enable transmitter
 
    SPI1CON0bits.BMODE = 1;
    SPI1CON0bits.MST = 1;    // master mode
    SPI1CON0bits.LSBF = 0;
    SPI1CON0bits.EN = 1;     // SPI enable
    
    ////////////////////////////////////////////////////////////////////////////
    // UART init
    ////////////////////////////////////////////////////////////////////////////
    // init uart 1
    U1CON0bits.MODE=0x0; //0x0=8bit
    U1CON0bits.RXEN=0; //disable receiver
    U1CON0bits.TXEN=0; //disable transmitter
    U1CON0bits.ABDEN=0; //disable auto baud
    U1CON0bits.BRGS=0; //baud high speed mode (4 clocks) 0=low speed (16 clocks) (recommended 0)

    //setup control register 1
    U1CON1bits.SENDB=0; //break transmission disabled
    U1CON1bits.BRKOVR=0; //break transmission disabled
    U1CON1bits.RXBIMD=0; //receive break interrupt mode 
    U1CON1bits.WUE=0; //wake up enabled bit 0=receiver operates normally
    U1CON1bits.ON=0; //serial port disabled for now

    //setup control register 2
    U1CON2bits.FLO=0x0; //no flow control
    U1CON2bits.TXPOL=0; //transmit polarity control bit 0=not inverted
    U1CON2bits.C0EN=0; //checksum mode select 
    U1CON2bits.STP=0x0; //stopbit mode control 0=1 stopbit
    U1CON2bits.RXPOL=0; //receive polarity control bit 0=not inverted 
    U1CON2bits.RUNOVF=0; //run during overflow control bit

    //setup baud rate
    U1BRGL=34;             // BRGS = 0 => BRG = (fCPU / (16 * baud)) - 1
    U1BRGH=0;

    //set uart 1 pins
    // TX: PGC / RB6 (pin 27)
    TRISBbits.TRISB6 = 0;   // output
    RB6PPS=0b00010011;
  
    // RX: PGD / RB7 (pin 28)
    TRISBbits.TRISB7 = 1;   // input
    U1RXPPS=0b00001111;
    
    // enable interrupt
    PIE3bits.U1RXIE = 1;
    
    // enable 
    U1CON1bits.ON=1;   //serial port enabled
    U1CON0bits.TXEN=1;
    U1CON0bits.RXEN=1;
    
    ////////////////////////////////////////////////////////////////////////////
    // I2C configuration
    ////////////////////////////////////////////////////////////////////////////
    // SCL : RC7
    TRISCbits.TRISC7 = 0;   // output
    ODCONCbits.ODCC7 = 1;   // open drain
    I2C1SCLPPSbits.I2C1SCLPPS = 0b010111;
    RC7PPS = 0b100001;
    // SDA : RC6
    TRISCbits.TRISC6 = 0;   // output
    ODCONCbits.ODCC6 = 1;   // open drain
    I2C1SDAPPSbits.I2C1SDAPPS = 0b010110;
    RC6PPS = 0b100010;

    I2C1CON0bits.MODE = 0b100;  // I2C Master mode 7-bit address
    I2C1CON1bits.ACKCNT = 1;
    I2C1CON2bits.ABD = 0;
    I2C1CLKbits.CLK = 0b0011;   // 
    I2C1CON0bits.EN = 1;        // enable

    ////////////////////////////////////////////////////////////////////////////
    // general interrupt
    ////////////////////////////////////////////////////////////////////////////
    INTCON0bits.IPEN = 0;  // high/low priority
    INTCON0bits.GIE = 1;   // enable unmasked interrupts
            

#if 0
    /////////////////////////////////////////////////////////////////////////
    // I2C test
    while (1) {
        I2C1PIR = 0;// ;Clear all the error flags
        I2C1ERR = 0;

        printf("start\n");
        I2C1ADB1 = 0xD0;
        I2C1CNT = 2;
        I2C1TXB = 0x10; // write address
        I2C1CON0bits.S = 1;
        
        while (! I2C1STAT1bits.TXBE);
        
        I2C1TXB = 0xab; // write data
        
        while(! I2C1STAT1bits.TXBE);
    
        while(!I2C1PIRbits.PCIF);     //  while (I2C1STAT0bits.MMA);
      
        printf("end\n");
        printf("I2C1CNT=%02Xh\n", I2C1CNT);
        
        char i;
       
        for (i = 0; i < 30; i++) __delay_ms(100);
      }
#endif
    
#define ACK_PULSE_READ  { TRISA = 0;  ASSERT_ACKn; while (! CEn); TRISA = 0xFF;  DEASSERT_ACKn; }
#define ACK_PULSE_WRITE  { ASSERT_ACKn; while (! CEn); DEASSERT_ACKn; }

    unsigned char address = 0;
    
    while (! CEn);
    
    while (1) {
        while (CEn);

        if (A0 == 0) {
            // data mode
            switch (address) {
                case 0:
                    // SPI chip select
                    if (WEn) {
                        // read
                        LATA = GET_SEL();
                        ACK_PULSE_READ;
                    } else {
                        // write
                        SET_SEL(PORTA);
                        ACK_PULSE_WRITE;
                    }

                    break;
                    
                case 1:
                    // SPI conf
                    if (WEn) {
                        // read
                        LATA = (SPI1CON2 & 0x80) | (SPI1STATUS & 0x04) | (SPI1CON2 & 0x03);
                        ACK_PULSE_READ;
                    } else {
                        // write
                        SPI1STATUS = PORTA & 0x04;  // clear buffers
                        SPI1CON2 = PORTA & 0x03;     // TXR / RXR
                        ACK_PULSE_WRITE;
                    }

                    break;
                    
                case 2:
                    // SPI data
                    if (WEn) {
                        // read
                        LATA = SPI1RXB;
                        ACK_PULSE_READ;
                    } else {
                        SPI1TXB = PORTA;
                        ACK_PULSE_WRITE;
                    }
                    
                    break;
                    
                case 4:
                    // MFP_REG_UART_STS, do only read
                    if (WEn) {
                        //LATA = (U1ERRIR & 0x80) + (U1FIFO & 0x02);
                        LATA = (U1ERRIR & 0x80) + uart_sts;
                        ACK_PULSE_READ;
                    } else {
                        // cannot write
                        ACK_PULSE_WRITE;
                    }

                    break;
                    
                case 5:
                    // MFP_REG_UART_DATA
                    if (WEn) {
                        LATA = uart_rxbuf[uart_rxb_r];
                        ACK_PULSE_READ;
                        
                        // disable interrupts while messing around with irq_sts
                        INTCON0bits.GIE = 0;
                        
                        if (uart_rxb_w != uart_rxb_r) {
                            uart_rxb_r++;
                        } else {
                            // underflow
                            uart_sts |= UART_RUDF_BIT;
                        }
                        
                        if (uart_rxb_w == uart_rxb_r) {
                            // no more char in FIFO, clear serial status bit
                            irq_sts &= ~IRQ_SERIAL_MASK;
                            
                            if ((irq_sts & irq_cfg) == 0) {
                                // no more pending and unmasked interrupts
                                DEASSERT_IRQn;
                            }
                        }
                        
                        INTCON0bits.GIE = 1;
                    } else {
                        U1TXB = PORTA;
                        ACK_PULSE_WRITE;
                    }
                    
                    break;
                    
                case 6:
                    // I2C status
                    //  bit 0: RXBF
                    //  bit 1: start
                    //  bit 2: reset
                    //  bit 5: TXBE
                    //  bit 7: RXIF
                    if (WEn) {
                        LATA = (I2C1STAT1 & 0x21 /* TXBE, RXBF */) | (PIR2 & 0x80 /* RXIF */);
                        ACK_PULSE_READ;
                    } else {
                        // bit 0: reset
                        unsigned char wval = PORTA;
                        ACK_PULSE_WRITE;
                        
                        if (wval & 2) {
                            // start
                            I2C1CON0bits.S = PORTA >> 1;
                        }
                        
                        if (wval & 4) {
                            I2C1STAT1bits.CLRBF = 1;
                            I2C1PIR = 0;
                            I2C1ERR = 0;
                        }
                        
                    }

                    break;
                    
                case 7:
                    // I2C cnt
                    if (WEn) {
                        LATA = I2C1CNT;
                        ACK_PULSE_READ;
                    } else {
                        I2C1CNT = PORTA;
                        ACK_PULSE_WRITE;
                    }
                    
                    break;
                    
                case 8:
                    // I2C data
                    if (WEn) {
                        LATA = I2C1RXB;
                        ACK_PULSE_READ;
                    } else {
                        I2C1TXB = PORTA;
                        ACK_PULSE_WRITE;
                    }
                    
                    break;
                    
                case 9:
                    // I2C command/address
                    if (WEn) {
                        LATA = I2C1ADB1;
                        ACK_PULSE_READ;
                    } else {
                        I2C1ADB1 = PORTA;
                        ACK_PULSE_WRITE;
                    }
                    
                    break;
                    
                case 14:
                    // B68K_IO_REG_IRQCFG
                    if (WEn) {
                        LATA = irq_cfg;
                        ACK_PULSE_READ;
                    } else {
                        irq_cfg = PORTA;
                        ACK_PULSE_WRITE;
                    }
                    
                        
                    break;
                    
                case 15:
                    // B68K_IO_REG_IRQSTS
                    if (WEn) {
                        LATA = irq_sts;
                        ACK_PULSE_READ;
                    } else {
//                        // UART input interrupt cleared by reading UART data
//                        unsigned char sts = PORTA;
//                        
//                        // disable interrupts while messing around with irq_sts
//                        INTCON0bits.GIE = 0;
//
//                        irq_sts &= ~sts;
//
//                        if (irq_sts == 0) {
//                            // no more pending interrupts
//                            DEASSERT_IRQn;
//                        }
//                        
                        ACK_PULSE_WRITE;

//                        INTCON0bits.GIE = 1;
                    }
                        
                    break;
                    
                default:
                    // 
                    if (WEn) {
                        ACK_PULSE_READ;
                    } else {
                        ACK_PULSE_WRITE;
                    }
                    break;
            }
        } else {
            // change/read address
            if (WEn == 0) {
                // write
                address = PORTA;
                ACK_PULSE_WRITE;
            } else {
                // read
                LATA = address;
                ACK_PULSE_READ;
            }
        }
        
    }
        
}
