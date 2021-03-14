/*******************************************************************************
*                       MSP432 DAC Display                                     *
*                                                                              *
* Author:  Long Tran                                                           *
* Device:  MSP432P401R LaunchPad                                               *
* Program: Generate a 2kHz sine wave                                           *
*                                                                              *
* Important Ports:                                                             *
* P9 is OUTPUT to drive potentiometer                                          *                                                                            
* for external DAC:                                                            *
* P6, P7, P10                                                                  *
*                                                                              *
* Demo: https://youtu.be/S_Ci-Jl91bw                                           *
*******************************************************************************/


// Include header file(s) and constants
#include "msp.h"
#include "math.h"

// Sine wave constants
#define SINE_WAVE_FREQ 2000
#define SAMPLE_FREQ 400000
#define POINTS_PER_PERIOD 200

// external AD7247 DAC Documentation: https://www.analog.com/media/en/technical-documentation/data-sheets/AD7237A_7247A.pdf
// 12-bit DAC constants
#define REF_PLUS 5.0
#define REF_MINUS 0.0

// Define prototypes
void init_clock(void);        // initialize PCM, FLCTL, CS
void configure_DAC(void);     // configure DAC using GPIO
void pull_CS_A_to_low(void);  //
void build_sine_wave(void);
void output_to_DAC(uint16_t result);
void wait(uint32_t t);        // busy wait

// Define DAC global variables
uint16_t results[POINTS_PER_PERIOD];   // 12-bit DAC

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // disable watchdog
    init_clock();
    configure_DAC();
    pull_CS_A_to_low();
    build_sine_wave();  uint8_t n;

    while(1){
        for(n = 0; n < POINTS_PER_PERIOD; n++){
            output_to_DAC(results[n]);
        }
    }

}

void output_to_DAC(uint16_t result){    // result is 12-bit
    // To ouput analog voltage, output to 12 bit DBx, select channel A or B
    // toggle /WR pin
    P10->OUT |= (BIT5);;        // disable write to DAC
    P10->OUT = result >> 8;           // DB11-8, write to P10.3-0
    P7->OUT  = (result << 8) >> 8;    // DB7-0,  write to P7.7-0
    P10->OUT &= ~(BIT5);       // enable write to DAC

}

void build_sine_wave(void){

    float temp;
    uint8_t n;

    for(n = 0; n < POINTS_PER_PERIOD; n++){
        // buffer[k] = sin(2PI*f/fs*k)
        temp = 2.0 + sin(2 * M_PI * SINE_WAVE_FREQ * n / SAMPLE_FREQ );    // decimal result
        // convert to 12-bit DAC output
        if(temp > REF_PLUS){
            temp = 5.0;
        }
        else if(temp < REF_MINUS){
            temp = 0.0;
        }
        results[n] = temp * 4095/REF_PLUS;   // 4095 = (2^12) - 1

    }
}

void configure_DAC(void){

    // Drive voltage to potentiometer
    P9->DIR |= BIT4;     // set P9.4 as OUTPUT to drive voltage to potentiometer
    P9->OUT |= BIT4;     // set potentiometer voltage high

    // External AD7247 DAC Documentation: https://www.analog.com/media/en/technical-documentation/data-sheets/AD7237A_7247A.pdf
    // DAC configuration: connect DBx, /WR, and /CSAB to OUTPUT GPIO pins
    P10->DIR |= 0x0F;    // DB11-8, most significant 4-bit latch
    P7->DIR  |= 0xFF;    // DB7-0 , least significant 8-bit latch
    P10->DIR |= BIT4 | BIT5;   // /CSA and /WR  pins
    P10->OUT &= ~BIT4;         // select /CSA

}

void pull_CS_A_to_low(void){
    P10->OUT = ~BIT4; // enable DAC by pulling /CSA (active low)
}
void init_clock(void){
    // MSP432 Technical Reference Manual
    // Set power level for the desired clock frequency (48 MHz)
    // Switches LDO VCORE0 to LDO VCORE1; mandatory for 48 MHz setting
    while((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;  // AM_LDO_VCORE1
    while((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 &= 0x0000FFFF;    // lock PCM

    //  Flash read wait state number change
    FLCTL->BANK0_RDCTL &= ~(BIT(12) | BIT(13) | BIT(14) | BIT(15) ); // reset bits
    FLCTL->BANK0_RDCTL |=   BIT(12);                                 // set 1 wait state
    FLCTL->BANK1_RDCTL &= ~(BIT(12) | BIT(13) | BIT(14) | BIT(15) );
    FLCTL->BANK1_RDCTL |=   BIT(12);                                 // set 1 wait state

    // Enable DCO, set frequency to 48 MHz
    CS->KEY = 0x0000695A;           // unlock clock system registers (MSP432 Ref. Manual, pg.394)
    CS->CTL0 |= BIT(16)| BIT(18);   // set DCO frequency range to 48 MHz
    CS->CTL0 |= BIT(23);            // enable DCO oscillator

    // Select DCO as the source for MCLK
    CS->CTL1 |= BIT0 | BIT1;        // MCLK source: DCOCLK
    CS->CLKEN |= BIT1;              // enable MCLK
    CS->KEY =0x0;                   // lock CS registers

}

void wait(uint32_t t){
    while(t > 0){t--;}
}
