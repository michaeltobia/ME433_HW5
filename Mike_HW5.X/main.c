#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h" // I2C2 init and communication functions

// DEVCFG0
#pragma config DEBUG = 0b00 // no debugging
#pragma config JTAGEN = 0b0 // no jtag
#pragma config ICESEL = 0b11 // use PGED1 and PGEC1
#pragma config PWP = 0b111111111 // no write protect
#pragma config BWP = 0b1 // no boot write protect
#pragma config CP = 0b1 // no code protect

// DEVCFG1
#pragma config FNOSC = 0b011 // use primary oscillator with pll
#pragma config FSOSCEN = 0b0 // turn off secondary oscillator
#pragma config IESO = 0b0 // no switching clocks
#pragma config POSCMOD = 0b10 // high speed crystal mode
#pragma config OSCIOFNC = 0b1 // disable secondary osc
#pragma config FPBDIV = 0b00 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = 0b10 // do not enable clock switch
#pragma config WDTPS = 0b00000 // use slowest wdt
#pragma config WINDIS = 0b1 // wdt no window mode
#pragma config FWDTEN = 0b0 // wdt disabled
#pragma config FWDTWINSZ = 0b11 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = 0b001 //(2x: 8->4) divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = 0b111 // (24x: 4->96) multiply clock after FPLLIDIV
#pragma config FPLLODIV = 0b001 // (2: 96->48) divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = 0b001 // (2: 8->4, 4*12->48) divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = 0b0 // USB clock on

// DEVCFG3
#pragma config USERID = 0xFFFF // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = 0b0 // allow multiple reconfigurations
#pragma config IOL1WAY = 0b0 // allow multiple reconfigurations
#pragma config FUSBIDIO = 0b1 // USB pins controlled by USB module
#pragma config FVBUSONIO = 0b1 // USB BUSON controlled by USB module

#define SLAVE_ADDR 0x20

void initExpander(char);
void setExpander(char, char);
unsigned char getExpander(char);


int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB4 = 1;
    
    // init I2C2, used as master
    i2c_master_setup();

    __builtin_enable_interrupts();
    
    initExpander(0b10000000);   // init expander w/ only GP7 as input
    
    unsigned char cur_GPIO = 0, not_but_press = 0;
    while(1){
        _CP0_SET_COUNT(0);                  // reset clock for 1kHz loop

        cur_GPIO = getExpander(0x09);       // get GPIO register

        if((getExpander(0x09)>>7)==1){      // if GP7 is high
            setExpander(0,0);                   // set GP0 low
        }else{                              // if GP7 is low
            setExpander(0,1);                   // set GP0 high
        }
        while(_CP0_GET_COUNT() < 24000000/1000){;} // do nothing until 1ms passes
        LATAINV = 0x10;                     // invert USER LED signal
    }
}

void initExpander(char iodir_set) {
    i2c_master_restart();
    // Set IOCON register 
    i2c_master_start();               // send start
    i2c_master_send(SLAVE_ADDR << 1 | 0); // send slave opp (left shift 1 for write [0])
    i2c_master_send(0x05);            // send IOCON address
    // 0b00111110-> 0x3E
    i2c_master_send(0x3E);            // send config byte
    i2c_master_stop();                // send stop
    
    // Set IODIR register
    i2c_master_start();               // send start
    i2c_master_send(SLAVE_ADDR << 1 | 0); // send slave opp (w/ write)
    i2c_master_send(0x00);            // send IODIR address
    i2c_master_send(iodir_set);       // send i/o direction byte
    i2c_master_stop();                // send stop
    
    // Set GPPU register
    // (use internal pull up resistors on input channels)
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(0x06);
    i2c_master_send(iodir_set);
    i2c_master_stop();
}

void setExpander(char pin, char level) {    
    char output_message = level << pin;
    // Send message to OLAT
    i2c_master_start();                    // send start
    i2c_master_send(SLAVE_ADDR << 1 | 0);  // send slave opp (w/ write)
    i2c_master_send(0x0A);                 // send OLAT address
    i2c_master_send(output_message);       // send message to OLAT
    i2c_master_stop();                     // send stop   
}

unsigned char getExpander(char reg_add) {
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(reg_add);
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR << 1 | 1);
    unsigned char recieved = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    return recieved;
}