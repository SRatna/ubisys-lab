/******************************************************************************
 *                                                                            *
 * Exercise 2                                                                 *
 *                                                                            *
 * Task 1: Push Da Button                   X /  4 Points                     *
 * Task 2: Advanced Button Handling         X /  4 Points                     *
 * Comprehension Questions                  X /  2 Points                     *
 *                                        ----------------                    *
 *                                         XX / 10 Points                     *
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
 *                               MSP430FR5969                                 *
 *                            -----------------                               *
 *                           |                 |                              *
 *                   (S1) -->|P4.5         P4.6|--> (LED1)                    *
 *                   (S2) -->|P1.1         P1.0|--> (LED2)                    *
 *                           |                 |                              *
 *                            -----------------                               *
 *                                                                            *
 ******************************************************************************/

// Select the task you are working on:
//#define TASK_1
#define TASK_2

#include <msp430fr5969.h>



#ifdef TASK_1
/*******************************************************************************
** Task 1
*******************************************************************************/

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

    /* INITIALIZATION */
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

    P1DIR |= BIT0;                      // P1.0 - output for LED2, off.
    P1OUT &= ~BIT0;

    P4DIR |= BIT6;                      // P4.6 - output for LED1, off.
    P4OUT &= ~BIT6;

    P1DIR &= ~BIT1;                     // P1.1 - input for S2.

    P4DIR &= ~BIT5;                     // P4.5 - output for S1.

    // enable pull up resistor for P1.1
    P1REN |= BIT1;
    P1OUT |= BIT1;

    // enable pull up resistor for P4.5
    P4REN |= BIT5;
    P4OUT |= BIT5;

    // Disable the GPIO power-on default high-impedance mode to activate the
    // previously configured port settings.
    PM5CTL0 &= ~LOCKLPM5;

    /* MAIN LOOP */
    while(1)
    {
        /* POLLING */
        if (!(P1IN & BIT1) && !(P1OUT & BIT0)) {    // check if button is pushed and LED2 is off
            P1OUT |= BIT0;                          // then Turn LED2 on.
        } else if (P1OUT & BIT0) {                  // check if LED2 is on when button is not pushed
            P1OUT &= ~BIT0;                         // then Turn LED2 off.
        }

        if (!(P4IN & BIT5) && !(P4OUT & BIT6)) {    // check if button is pushed and LED1 is off
            P4OUT |= BIT6;                          // then Turn LED1 on.
        } else if (P4OUT & BIT6) {                  // check if LED1 is on when button is not pushed
            P4OUT &= ~BIT6;                         // then Turn LED1 off.
        }

    }
}

/******************************************************************************/
#endif /* TASK_1 */



#ifdef TASK_2
/*******************************************************************************
** Task 2
*******************************************************************************/

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

    /* TODO INITIALIZATION */
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

    P1DIR |= BIT0;                      // P1.0 - output for LED2, off.
    P1OUT &= ~BIT0;

    P4DIR |= BIT6;                      // P4.6 - output for LED1, off.
    P4OUT &= ~BIT6;

    P1DIR &= ~BIT1;                     // P1.1 - input for S2.

    P4DIR &= ~BIT5;                     // P4.5 - output for S1.

    // enable pull up resistor for P1.1
    P1REN |= BIT1;
    P1OUT |= BIT1;

    // enable pull up resistor for P4.5
    P4REN |= BIT5;
    P4OUT |= BIT5;

    // select interrupt edge
    P1IES |= BIT1;                       // P1.1 high to low
    P4IES |= BIT5;                       // P4.5 high to low

    // set interrupt enable flags
    P1IE |= BIT1;                        // For P1.1
    P4IE |= BIT5;                        // For P4.5

    // Disable the GPIO power-on default high-impedance mode to activate the
    // previously configured port settings.
    PM5CTL0 &= ~LOCKLPM5;

    // Clear interrupt flags that have been raised due to high-impedance settings.
    P1IFG &= ~BIT1;
    P4IFG &= ~BIT5;

    // Enable interrupts globally.
    __bis_SR_register(GIE);

    /* MAIN LOOP */
    while(1)
    {

    }
}

/* ISR PORT 1 */
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    switch(P1IV)
    {
    case P1IV_P1IFG0:                   // P1.0
        break;
    case P1IV_P1IFG1:                   // P1.1

        /* PORT ISR */

        P1IES ^= BIT1;                  // switch the edge
        __delay_cycles(250000);         // delay for 0.25 s
        P1IFG &= ~BIT1;                 // clear the possibly raised interrupt flag
        if ((P1IN & BIT1)) {           // validate the switch: check whether S2 is pushed or not
            P1OUT ^= BIT0;              // then Turn on LED2.
        }
        break;
    case P1IV_P1IFG2:                   // P1.2
        break;
    case P1IV_P1IFG3:                   // P1.3
        break;
    case P1IV_P1IFG4:                   // P1.4
        break;
    case P1IV_P1IFG5:                   // P1.5
        break;
    case P1IV_P1IFG6:                   // P1.6
        break;
    case P1IV_P1IFG7:                   // P1.7
        break;
    }
}

/* ISR PORT 4 */
#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
{
    switch(P4IV)
    {
    case P4IV_P4IFG0:                   // P4.0
        break;
    case P4IV_P4IFG1:                   // P4.1
        break;
    case P4IV_P4IFG2:                   // P4.2
        break;
    case P4IV_P4IFG3:                   // P4.3
        break;
    case P4IV_P4IFG4:                   // P4.4
        break;
    case P4IV_P4IFG5:                   // P4.5

        /* PORT ISR */

       P4IES ^= BIT5;                  // switch the edge
       __delay_cycles(250000);         // delay for 0.25 s
       P4IFG &= ~BIT5;                 // clear the possibly raised interrupt flag
       if ((P4IN & BIT5)) {           // validate the switch: check whether S1 is pushed or not
           P4OUT ^= BIT6;              // then TOGGLE LED1.
       }

        break;
    case P4IV_P4IFG6:                   // P4.6
        break;
    case P4IV_P4IFG7:                   // P4.7
        break;
    }
}

/******************************************************************************/
#endif /* TASK_2 */
