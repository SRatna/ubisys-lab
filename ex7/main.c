/******************************************************************************
 *                                                                            *
 * Exercise 7                                                                 *
 *                                                                            *
 * Task 1: Leave a Light On                X /  8 Points                      *
 * Comprehension Questions                 X /  2 Points                      *
 *                                        ----------------                    *
 *                                        XX / 10 Points                      *
 *                                                                            *
 ******************************************************************************
 *                                                                            *
 * name:                    Sushanta Ratna Adhikari                           *
 * matriculation number:    1532852                                           *
 * e-mail:                  sushanta.adhikari@student.uni-siegen.de           *
 *                                                                            *
 ******************************************************************************
 *                                                                            *
 * Hardware Setup                                                             *
 *                                                                            *
 *             VCC               MSP430FR5969                                 *
 *              |             -----------------                               *
 *           (SENSOR)        |                 |                              *
 *              +-------- -->|P4.2/A10     P4.6|--> (LED1)                    *
 *              |            |             P1.0|--> (LED2)                    *
 *             |R|           | 1 MHz           |                              *
 *              |             -----------------                               *
 *             GND                                                            *
 *                                                                            *
 ******************************************************************************/

#include <msp430fr5969.h>
#include <stdint.h>



#define THRS_SHADE 800                 // Threshold value for shade.
#define THRS_DARK 5                   // Threshold value for darkness.

// Transform threshold value for shade from 12 to 8 bit mode.
// Divide threshold value by 2^(12-8) = 2^4 = 16.
#define THRS_SHADE_SLOW ((uint16_t) THRS_SHADE / 16)

volatile struct
{
    uint8_t Adc : 3;                    // Current ADC state:
                                        // 0:   Uninitialized after reset.
                                        // 1:   Transition to fast sampling.
                                        // 2:   Fast sampling.
                                        // 3:   Transition to slow sampling
                                        // 4:   Slow sampling.
} System;

typedef enum                            // State values of main FSM:
{
    ADC_RST,                            // 0:   Uninitialized after reset.
    ADC_T_FAST,                         // 1:   Transition to fast sampling.
    ADC_FAST,                           // 2:   Fast sampling.
    ADC_T_SLOW,                         // 3:   Transition to slow sampling
    ADC_SLOW                            // 4:   Slow sampling.
} SYSTEM_ADC;

volatile uint16_t adc_val = 0;


void uartTx(uint16_t data);

/* MAIN PROGRAM */
void main(void)
{
    // Stop watchdog timer.
    WDTCTL = WDTPW | WDTHOLD;


    // Initialize the clock system to generate 1 MHz DCO clock.
    CSCTL0_H    = CSKEY_H;              // Unlock CS registers.
    CSCTL1      = DCOFSEL_0;            // Set DCO to 1 MHz, DCORSEL for
                                        //   high speed mode not enabled.
    CSCTL2      = SELA__VLOCLK |        // Set ACLK = VLOCLK = 10 kHz.
                  SELS__DCOCLK |        //   Set SMCLK = DCOCLK.
                  SELM__DCOCLK;         //   Set MCLK = DCOCLK.
                                        // SMCLK = MCLK = DCOCLK = 1 MHz.
    CSCTL3      = DIVA__1 |             //   Set ACLK divider to 1.
                  DIVS__1 |             //   Set SMCLK divider to 1.
                  DIVM__1;              //   Set MCLK divider to 1.
                                        // Set all dividers to 1.
    CSCTL0_H    = 0;                    // Lock CS registers.


    // Initialize unused GPIOs to minimize energy-consumption.
    // Port 1:
    P1DIR = 0xFF;
    P1OUT = 0x00;
    // Port 2:
    P2DIR = 0xFF;
    P2OUT = 0x00;
    // Port 3:
    P3DIR = 0xFF;
    P3OUT = 0x00;
    // Port 4:
    P4DIR = 0xFF;
    P4OUT = 0x00;
    // Port J:
    PJDIR = 0xFFFF;
    PJOUT = 0x0000;

    // Initialize port 1:
    P1DIR |= BIT0;                      // P1.0 - output for LED2, off.
    P1OUT &= ~BIT0;

    // Initialize port 4:
    P4DIR |= BIT6;                      // P4.6 - output for LED1, off.
    P4OUT &= ~BIT6;


    /* INIT ADC PIN */

    // Initialize ADC input pins:
    P4SEL1 |= BIT2;                    // P4.2/A10
    P4SEL0 |= BIT2;                    // SET BOTH TO 1

    // Initialize port 2:
    // Select Tx and Rx functionality of eUSCI0 for hardware UART.
    // P2.0 - UART Tx (UCA0TXD).
    // P2.1 - UART Rx (UCA0RXD).
    P2SEL0 &= ~(BIT1 | BIT0);
    P2SEL1 |= BIT1 | BIT0;

    // Disable the GPIO power-on default high-impedance mode to activate the
    // previously configured port settings.
    PM5CTL0 &= ~LOCKLPM5;


    /* Initialization of the serial UART interface */

    UCA0CTLW0 |= UCSSEL__SMCLK |        // Select clock source SMCLK = 1 MHz.
                 UCSWRST;               // Enable software reset.
    // Set Baud rate of 9600 Bd.
    // Recommended settings available in table 30-5, p. 779 of the User's Guide.
    UCA0BRW = 6;                        // Clock prescaler of the
                                        //   Baud rate generator.
    UCA0MCTLW = UCBRF_8 |               // First modulations stage.
                UCBRS5 |                // Second modulation stage.
                UCOS16;                 // Enable oversampling mode.
    UCA0CTLW0 &= ~UCSWRST;              // Disable software reset and start
                                        //   eUSCI state machine.


    /* INIT ADC REF */

    /* Initialization of the reference voltage generator */

    // Configure the internal voltage reference for 2.5 V:

    REFCTL0 |= REFVSEL1;                // 00100000b(0x0020) = 2.5 V
    REFCTL0 |= REFON;


    // Initialization of the system variables.
    System.Adc = ADC_RST;

    // Enable interrupts globally.
    __bis_SR_register(GIE);
    /* MAIN LOOP */
    while(1)
    {
        uartTx(adc_val);

        // FSM to control the ADC sampling rate.
        switch(System.Adc)
        {
        case ADC_RST:                   // Initialize ADC right after reset.

            // Go to the initialization of slow sampling.
            System.Adc = ADC_T_SLOW;

            break;

        case ADC_T_SLOW:                // Initialize ADC to sample slow.

            /* INIT ADC SLOW */

            // Configure the analog-to-digital converter such that it samples
            // at a sampling frequency of 4 Hz, triggered by timer TA0.1.
            ADC12CTL0 = 0;                      // Disable ADC trigger.
            ADC12CTL0 |= ADC12ON;               // Turn on ADC.
            ADC12CTL1 = ADC12SHP;              // Use sampling timer.
            ADC12CTL1 |= ADC12CONSEQ_2;         // Repeat-single-channel mode.
            ADC12CTL1 |= ADC12SHS_1;            // Use timer TA0.1 as trigger.
            ADC12CTL2 = ADC12RES__8BIT;        // 8 bit conversion resolution.
            ADC12IER0 |= ADC12IE0;              // Enable ADC conversion interrupt.
            ADC12MCTL0 |= ADC12INCH_10;         // Select ADC input channel A10.
            ADC12MCTL0 |= ADC12VRSEL_1;         // Reference voltages V+ = VREF buffered
                                                // = 2.5V and V- = AVSS = GND.
            ADC12CTL0 |= ADC12ENC;              // Enable ADC trigger.
            ADC12CTL0 |= ADC12SC;               // Trigger sampling.

            // Initialize timer A0 to trigger ADC12 at 4 Hz:
            TA0CTL |= TACLR;                     // Timer A counter clear
            TA0CTL |= MC__UP;                    // Timer A mode control: Up to CCR0
            TA0CTL |= TASSEL__SMCLK;             // Timer A clock source select: SMCLK (1MHZ)
            TA0CTL |= ID__4;                     // Timer A input divider: /4 (prescaler)
            TA0EX0 = TAIDEX_4;                   // Secondary input divider stage of 5.
            TA0CCR0 = 12500;                     // set ccr0 for 4hz 10^6/(20*4)
            TA0CCTL1 = OUTMOD_3;                 // output mode set/reset.

            TA0CCR1 = 12375;                     // Duty cycle: 2.5 ms (12500 - 125)
                                                 //   -> 2.5 ms sample-and-hold time.


            // Let LED 1 blink according to trigger:
            TA0CCTL0 |= CCIE;                    // Enable interrupt of CCR0.

            while(ADC12IFGR0 & ADC12IFG0);       // Wait for first 8 bit sample.

            // Go to the slow sampling process.
            System.Adc = ADC_SLOW;

            break;

        case ADC_SLOW:                  // Process using slow sampling.

            // Check threshold:
            if(adc_val < THRS_SHADE_SLOW)       // Shaded.
            {
                // Go to the initialization of fast sampling.
                System.Adc = ADC_T_FAST;
            }

            break;

        case ADC_T_FAST:                // Initialize ADC to sample fast.

            // Stop triggering timer A0.1 of slow sampling.
            TA0CTL = MC__STOP;                  // Keep the timer halted.
            TA0CCR0 = 0;                        // Disable the comparator.
            TA0CCTL0 &= ~CCIE;                  // Disable interrupt of CCR0.


            /* INIT ADC FAST */

            // Configure the analog-to-digital converter such that it samples
            // as fast as possible.

            ADC12CTL0 = 0;                      // Disable ADC trigger.
            ADC12CTL0 |= ADC12ON;               // Turn on ADC.
            ADC12CTL0 |= ADC12MSC;              // Multiple sample and conversion mode.
            ADC12CTL0 |= ADC12SHT0_6;           // Sample-and-hold time of 128 cycles of MODCLK.
            ADC12CTL1 = ADC12SHP;               // Use sampling timer.
            ADC12CTL1 |= ADC12CONSEQ_2;         // Repeat-single-channel mode.
            ADC12CTL2 = ADC12RES__12BIT;        // 12 bit conversion resolution.
            ADC12IER0 |= ADC12IE0;              // Enable ADC conversion interrupt.
            ADC12MCTL0 |= ADC12INCH_10;         // Select ADC input channel A10.
            ADC12MCTL0 |= ADC12VRSEL_1;         // Reference voltages V+ = VREF buffered
                                                // = 2.5V and V- = AVSS = GND.
            ADC12CTL0 |= ADC12ENC;              // Enable ADC trigger.
            ADC12CTL0 |= ADC12SC;               // Trigger sampling.

            while(ADC12IFGR0 & ADC12IFG0);     // Wait for first 12 bit sample.

            // Go to the fast sampling process.
            System.Adc = ADC_FAST;

            break;

        case ADC_FAST:                  // Process using fast sampling.

            // Check threshold:
            if(adc_val < THRS_DARK)             // Dark.
            {
                P1OUT &= ~BIT0;                 // Turn off LED 2.
                P4OUT |= BIT6;                  // Turn on LED 1.
            }
            else if(adc_val < THRS_SHADE)       // Shaded.
            {
                P1OUT |= BIT0;                  // Turn on LED 2.
                P4OUT &= ~BIT6;                 // Turn off LED 1.
            }
            else                                // Bright.
            {
                P1OUT &= ~BIT0;                 // Turn off LED 2.
                P4OUT &= ~BIT6;                 // Turn off LED 1.

                // Go to the initialization of slow sampling.
                System.Adc = ADC_T_SLOW;
            }

            break;
        }
    }
}


/* ### ADC ### */

/* ISR ADC */
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    switch(__even_in_range(ADC12IV, ADC12IV_ADC12RDYIFG))
    {
    case ADC12IV_NONE:        break;        // Vector  0:  No interrupt
    case ADC12IV_ADC12OVIFG:  break;        // Vector  2:  ADC12MEMx Overflow
    case ADC12IV_ADC12TOVIFG: break;        // Vector  4:  Conversion time overflow
    case ADC12IV_ADC12HIIFG:  break;        // Vector  6:  ADC12BHI
    case ADC12IV_ADC12LOIFG:  break;        // Vector  8:  ADC12BLO
    case ADC12IV_ADC12INIFG:  break;        // Vector 10:  ADC12BIN
    case ADC12IV_ADC12IFG0:                 // Vector 12:  ADC12MEM0 Interrupt

        adc_val = ADC12MEM0;

        break;
    case ADC12IV_ADC12IFG1:   break;        // Vector 14:  ADC12MEM1
    case ADC12IV_ADC12IFG2:   break;        // Vector 16:  ADC12MEM2
    case ADC12IV_ADC12IFG3:   break;        // Vector 18:  ADC12MEM3
    case ADC12IV_ADC12IFG4:   break;        // Vector 20:  ADC12MEM4
    case ADC12IV_ADC12IFG5:   break;        // Vector 22:  ADC12MEM5
    case ADC12IV_ADC12IFG6:   break;        // Vector 24:  ADC12MEM6
    case ADC12IV_ADC12IFG7:   break;        // Vector 26:  ADC12MEM7
    case ADC12IV_ADC12IFG8:   break;        // Vector 28:  ADC12MEM8
    case ADC12IV_ADC12IFG9:   break;        // Vector 30:  ADC12MEM9
    case ADC12IV_ADC12IFG10:  break;        // Vector 32:  ADC12MEM10
    case ADC12IV_ADC12IFG11:  break;        // Vector 34:  ADC12MEM11
    case ADC12IV_ADC12IFG12:  break;        // Vector 36:  ADC12MEM12
    case ADC12IV_ADC12IFG13:  break;        // Vector 38:  ADC12MEM13
    case ADC12IV_ADC12IFG14:  break;        // Vector 40:  ADC12MEM14
    case ADC12IV_ADC12IFG15:  break;        // Vector 42:  ADC12MEM15
    case ADC12IV_ADC12IFG16:  break;        // Vector 44:  ADC12MEM16
    case ADC12IV_ADC12IFG17:  break;        // Vector 46:  ADC12MEM17
    case ADC12IV_ADC12IFG18:  break;        // Vector 48:  ADC12MEM18
    case ADC12IV_ADC12IFG19:  break;        // Vector 50:  ADC12MEM19
    case ADC12IV_ADC12IFG20:  break;        // Vector 52:  ADC12MEM20
    case ADC12IV_ADC12IFG21:  break;        // Vector 54:  ADC12MEM21
    case ADC12IV_ADC12IFG22:  break;        // Vector 56:  ADC12MEM22
    case ADC12IV_ADC12IFG23:  break;        // Vector 58:  ADC12MEM23
    case ADC12IV_ADC12IFG24:  break;        // Vector 60:  ADC12MEM24
    case ADC12IV_ADC12IFG25:  break;        // Vector 62:  ADC12MEM25
    case ADC12IV_ADC12IFG26:  break;        // Vector 64:  ADC12MEM26
    case ADC12IV_ADC12IFG27:  break;        // Vector 66:  ADC12MEM27
    case ADC12IV_ADC12IFG28:  break;        // Vector 68:  ADC12MEM28
    case ADC12IV_ADC12IFG29:  break;        // Vector 70:  ADC12MEM29
    case ADC12IV_ADC12IFG30:  break;        // Vector 72:  ADC12MEM30
    case ADC12IV_ADC12IFG31:  break;        // Vector 74:  ADC12MEM31
    case ADC12IV_ADC12RDYIFG: break;        // Vector 76:  ADC12RDY
    default: break;
    }
}


/* ### TRIGGER TIMER ### */

/* ISR TIMER A0 - CCR0 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
{                                       // TA0 CCR0
    P4OUT ^= BIT6;                      // Toggle LED1.
}


/* ### SERIAL INTERFACE ### */

void uartTx(uint16_t data)
{
    // Higher byte
    while((UCA0STATW & UCBUSY));            // Wait while module is busy.
    UCA0TXBUF = (0xFF00 & data) >> 8;       // Transmit data byte.
    // Lower byte
    while((UCA0STATW & UCBUSY));            // Wait while module is busy.
    UCA0TXBUF = 0x00FF & data;              // Transmit data byte.
}
