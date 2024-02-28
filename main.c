
#define _XTAL_FREQ 5000000 //define crystal frequency to 20MHz


#include <xc.h>
#include <htc.h>
#include "constants.h"
#include "externs.h"


#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #############################################################################

void InitializeRegistors_v(void) //Begin IIC as master
{
  TRISC3 = 1;  TRISC4 = 1;  //Set SDA and SCL pins as input pins
  TRISD  = 0;
  PORTD  = 0;
  SSPADD = 0xD2;          // This value is shifted left one bit
  SSPSTAT = 0x80; //  set standard slew rate
  SSPCON  = 0x36; // Select & Enable I2C (Slave Mode)
  SSPCON2 = 0x01; // Enable Clock Stretching  
  SSPSTAT = 0b10000000;    //set standard slew rate
  
  OPTION_REG = 0b00000001;  // Timer0 with internal Osc and 4 as pre-scalar
  TMR0=101;                 // Load the time value for 100 = 1 Ms delay
  TMR0IE=1;                 //Enable timer interrupt bit in PIE1 register
    
  PIR1bits.SSPIF = 0;      // Enable Interrupts
  PIE1bits.SSPIE = 1;     // Enable  PICI2C module interrupt
  INTCONbits.PEIE = 1;    // Enable peripheral interrupt
  INTCONbits.GIE = 1;    // Enable global interrupt
 
}
// #############################################################################

void __interrupt() ISR(void)
{
    unsigned char i2cCommandComplete_uc = FALSE;
    
     if(PIR1bits.SSPIF)
    {
        CKP = 0; // stretch the clock signal to low
        if (SSPCONbits.SSPOV || SSPCONbits.WCOL) // check errors such as collision and buffer overflow
        {
            RX_Data_0 = SSPBUF; // read the value from SSPBUF to clear it 
            SSPCONbits.SSPOV = 0; // reset overflow detection flag bit
            SSPCONbits.WCOL = 0; // reset collision detection flag bit
            SSPCONbits.CKP = 1; // Release Clock from low
        }
        if(!(SSPSTATbits.R_nW)) // Read
        {
            RX_data_uc = SSPBUF; // Read The Received Data Byte

            if (SSPSTATbits.D_nA){     // not the address byte
                RX_data_auc[RX_data_index_uc++] = RX_data_uc;
                //RX_data_index_uc++;     
                switch(RX_data_auc[0]){
                    case RESET:
                        i2cCommandComplete_uc = TRUE;
                        break;
                    default:
                        i2cCommandComplete_uc = TRUE;
                        break;
                }
                if(i2cCommandComplete_uc == TRUE){
                    RX_data_index_uc = 0;
                    RX_data_received = TRUE;                                     
                }
            }
        }

        CKP = 1; // Release Clock Line SCL
        SSPIF = 0; // clear I2C interrupt flag
    }
    /* ********************************************************************** */
    if(TMR0IF==1)
    {
        stepDir_uc=~stepDir_uc;   // complement the value for blinking the LEDs
        RD0 = stepDir_uc & 0x01;
        TMR0 = 1;     /*Load the timer Value, (Note: Timer value is 101 instead of 100 as the
                          TImer0 needs two instruction Cycles to start incrementing TMR0 */
        TMR0IF=0;       // Clear timer interrupt flag
    }  
}

// #############################################################################
// #############################################################################
// #############################################################################

void main( void )
{
    InitializeRegistors_v(); //Initialize I2C Master with 100KHz clock
    
    RD1 = 1;  // Disable /ENABLE
    RD1= 0;     // /Motor ENABLE = 0
    RD2= 0;     // /Motor Direction
        
    while(1){

        if (RX_data_received == TRUE){
            RX_data_received = FALSE;
            RD1 = 0;    // /ENABLE
            RD2 = RX_data_auc[0] & 0b00000001;
            
            //i2c.write_block_data(device_address, 0 0xFF, [2, 3, 4, 5, 6, 7, 8])  # resets command index on PIC
            
            steps_i = (RX_data_auc[1] << 8 ) + RX_data_auc[2];
            speed_i = (RX_data_auc[3] << 8 ) + RX_data_auc[4]; 
            
            //__delay_ms(1000);

            for(i=0; i<steps_i; i++){
                RD0= 1;     // /ENABLE
                for(j=0;j<speed_i;j++){
                    __delay_us(10);
                }
                RD0 = 0;
                for(j=0;j<speed_i;j++){
                    __delay_us(10);
                }            
                if (RX_data_received ==TRUE){
                    i = steps_i;
                    //break;
                }
            }
            if(RX_data_auc[0] & 0b01000000){
                RD1 = 1;  // Disable /ENABLE
            }

                

        }
    }
}


/*
 0b10000000         Command included - else speed - RUN
 0b11000000         Special Command
 0b10100000         Direction 0/1 bit sets direction read when not special command
 0b11xxxxxx         Steps to execute
 
 0b0xxxxxxx         Motor Speed 
  
 */
//void CheckForCommandReceived_uc(void){
//    commandReceived_uc = FALSE;
//    if (RX_data_received == TRUE){
//        RX_data_received = FALSE;              
//        steps_i = (RX_data_auc[2] << 8 ) + RX_data_auc[3];
//        speed_i = (RX_data_auc[4] << 8 ) + RX_data_auc[5];
//        commandReceived_uc = TRUE;
//    }    
//}