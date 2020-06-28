/******************************************************************************
 *                                                                            *
 * Exercise 5                                                                 *
 *                                                                            *
 * Task 1: Step By Step                    X /  8 Points                      *
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
 *                               MSP430FR5969                                 *
 *                            -----------------                               *
 *                           |                 |                              *
 *                   (S1) -->|P4.5         P4.6|--> (LED1)                    *
 *                   (S2) -->|P1.1         P1.0|--> (LED2)                    *
 *                           |                 |                              *
 *                            -----------------                               *
 *                                                                            *
 ******************************************************************************/

#include <msp430fr5969.h>
#include <stdint.h>



volatile struct                         // State variables of the FSMs:
{
    uint8_t Main : 2;                   // FSM of main routine:
                                        // 0:   Idle
                                        // 1:   LED1
                                        // 2:   LED2
    uint8_t Led1 : 1;                   // FSM of LED 1:
                                        // 0:   Off
                                        // 1:   On
    uint8_t Led2 : 1;                   // FSM of LED 2:
                                        // 0:   Off
                                        // 1:   On
    uint8_t But1 : 1;                   // Event at button S1:
                                        // 0:   No button event
                                        // 1:   Button S1 has been pushed
    uint8_t But2 : 1;                   // Event at button S2:
                                        // 0:   No button event
                                        // 1:   Button S2 has been pushed
    uint8_t Time : 1;                   // Timeout event:
                                        // 0:   No timeout event
                                        // 1:   Timeout event released
} States;

typedef enum                            // State values of main FSM:
{
    MAIN_IDLE,                          // 0:   Idle
    MAIN_LED1,                          // 1:   LED1
    MAIN_LED2                           // 2:   LED2
} STATES_MAIN;


/* TIMEOUT TIMER */

void start_timeout(void);               // Initialize timeout watchdog timer.
void clear_timeout(void);               // Clear timeout watchdog timer.
void stop_timeout(void);                // Stop timeout watchdog timer.

/* BLINKING LEDS */

void blink_led1(void);                  // Blink LED1 considering its status.
void blink_led2(void);                  // Blink LED2 considering its status.



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
    P1DIR &= ~BIT1;                     // P1.1 - input for S2, pullup.
    P1REN |= BIT1;
    P1OUT |= BIT1;
    // Initialize port 4:
    P4DIR |= BIT6;                      // P4.6 - output for LED1, off.
    P4OUT &= ~BIT6;
    P4DIR &= ~BIT5;                     // P4.5 - input for S1, pullup.
    P4REN |= BIT5;
    P4OUT |= BIT5;

    // Initialize port interrupts:
    P1IES |= BIT1;                      // P1.1 - falling edge detection.
    P1IE |= BIT1;                       //   Port interrupt enabled.
    P4IES |= BIT5;                      // P4.5 - falling edge detection.
    P4IE |= BIT5;                       //   Port interrupt enabled.

    // Disable the GPIO power-on default high-impedance mode to activate the
    // previously configured port settings.
    PM5CTL0 &= ~LOCKLPM5;

    // Clear interrupt flags that have been raised due to high-impedance settings.
    P1IFG &= ~BIT1;
    P4IFG &= ~BIT5;


    /* Initialize button debouncing */

    // Initialize timer A0 for button debouncing with a delay of 25 ms:
    // 1,000,000 Hz / 1 / 1 = 1,000,000 Hz (cycles per second)
    // -> 1 digit = 1 us
    // 25 ms / 1 us = 25,000 cycles @ 1 MHz
    TA0CTL = TASSEL__SMCLK |            // Select clock source SMCLK, 1 MHz.
             ID__1 |                    // First input divider stage of 1.
             MC__STOP;                  // Keep the timer halted.
    TA0EX0 = TAIDEX_0;                  // Secondary input divider stage of 1.
    TA0CCR0 = 0xFFFF;                   // Set compare value to full period.


    /* Initialize status bits */

    States.Main = MAIN_IDLE;
    States.Led1 = 0;
    States.Led2 = 0;
    States.But1 = 0;
    States.But2 = 0;
    States.Time = 0;


    // Enable interrupts globally.
    __bis_SR_register(GIE);


    /* MAIN LOOP */
    while(1)
    {

        /* FSM MAIN */

        switch(States.Main)
        {

        /* IDLE */
        case MAIN_IDLE:

            // Button S1 -> go to MAIN_LED1
            if(States.But1)
            {
                // Reset button S1 event.
                States.But1 = 0;
                start_timeout();        // Start timeout timer.

                // Go to state of LED 1 configuration.
                States.Main = MAIN_LED1;
            }

            // EXECUTION: IDLE

            break;

        /* CONFIGURE LED1 */
        case MAIN_LED1:

            // Button S1 -> go to MAIN_LED2
            if(States.But1)
            {
                // Reset button S1 event.
                States.But1 = 0;
                clear_timeout();        // Clear timeout timer.

                // Go to state of LED 2 configuration.
                States.Main = MAIN_LED2;
            }

            // Button S2 -> toggle LED1
            if(States.But2)
            {
                // Reset button S2 event.
                States.But2 = 0;
                clear_timeout();        // Clear timeout timer.

                // Toggle LED 1 status bit.
                States.Led1 ^= 1;
            }

            // Timeout -> go to MAIN_IDLE
            if(States.Time)
            {
                // Reset timeout event.
                States.Time = 0;
                stop_timeout();         // Stop timeout timer.

                // Go back to idle state.
                States.Main = MAIN_IDLE;
            }

            // EXECUTION: LED1

            blink_led1();               // Play configuration active blinking.

            break;

        /* CONFIGURE LED2 */
        case MAIN_LED2:

            // Button S1 -> go to MAIN_LED1
            if(States.But1)
            {
                // Reset button S1 event.
                States.But1 = 0;
                clear_timeout();        // Clear timeout timer.

                // Go to state of LED 1 configuration.
                States.Main = MAIN_LED1;
            }

            // Button S2 -> toggle LED2
            if(States.But2)
            {
                // Reset button S2 event.
                States.But2 = 0;
                clear_timeout();        // Clear timeout timer.

                // Toggle LED 2 status bit.
                States.Led2 ^= 1;
            }

            // Timeout -> go to MAIN_IDLE
            if(States.Time)
            {
                // Reset timeout event.
                States.Time = 0;
                stop_timeout();         // Stop timeout timer.

                // Go back to idle state.
                States.Main = MAIN_IDLE;
            }

            // EXECUTION: LED2

            blink_led2();               // Play configuration active blinking.

            break;
        }
    }
}


/* ### TIMEOUT TIMER ### */

void start_timeout(void)
{
    /* TIMEOUT START */

    // Initialize watchdog timer for timeout interrupt:
    // Timeout interrupt after interval 3.3 s.
    SFRIE1 |= WDTIE;                    // watchdog timer interrupt enable
    WDTCTL = WDTPW                      // Watchdog timer password
            | WDTTMSEL                  // Timer Mode Select: Interval mode
            | WDTSSEL__ACLK             // Timer Clock Source Select: ACLK; for us 10000 Hz
            | WDTIS__32K;               // Timer Interval Select: /32k; as 1/(10000/32768) = 3.3 sec (approx)

}

void clear_timeout(void)                // Clear the timeout counter.
{
    /* TIMEOUT CLEAR */
    WDTCTL = WDTPW
            | WDTCNTCL                  // Timer Clear
            | WDTTMSEL | WDTSSEL__ACLK | WDTIS__32K;
}

void stop_timeout(void)                 // Stop the timeout counter.
{
    /* TIMEOUT STOP */
    WDTCTL = WDTPW | WDTHOLD;
    SFRIE1 &= ~WDTIE;                   // watchdog timer interrupt disable
}

/* ISR WATCHDOG */
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
    // Raise status bit of timeout event.
    States.Time = 1;
}


/* ### BLINKING LEDS ### */

void blink_led1(void)
{
    // Play configuration active blinking.
    if(States.Led1)
    {
        P4OUT &= ~BIT6;
        __delay_cycles(40000);
        P4OUT |= BIT6;
        __delay_cycles(260000);
    }
    else
    {
        P4OUT |= BIT6;
        __delay_cycles(20000);
        P4OUT &= ~BIT6;
        __delay_cycles(280000);
    }
}

void blink_led2(void)
{
    // Play configuration active blinking.
    if(States.Led2)
    {
        P1OUT &= ~BIT0;
        __delay_cycles(40000);
        P1OUT |= BIT0;
        __delay_cycles(260000);
    }
    else
    {
        P1OUT |= BIT0;
        __delay_cycles(20000);
        P1OUT &= ~BIT0;
        __delay_cycles(280000);
    }
}


/* ### BUTTON DEBOUNCING ### */

/* ISR PORT 1 */
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    switch(__even_in_range(P1IV, P1IV_P1IFG7))
    {
    case P1IV_P1IFG0:                   // P1.0
        break;
    case P1IV_P1IFG1:                   // P1.1

        /* Start Debouncing Button S2 */

        P1IE &= ~BIT1;                  // Disable port interrupt during delay.

        // Start timer- and interrupt-based delay of 25 ms.
        TA0R = 0;                       // Reset counting register.
        TA0CCR1 = 25000;                // Set threshold to 25 ms.
        TA0CCTL1 |= CCIE;               // Enable interrupt capability.
        TA0CTL |= MC__CONTINUOUS;       // Start timer in up mode.

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
    switch(__even_in_range(P4IV, P4IV_P4IFG7))
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

        /* Start Debouncing Button S1 */

        P4IE &= ~BIT5;                  // Disable port interrupt during delay.

        // Start timer- and interrupt-based delay of 25 ms.
        TA0R = 0;                       // Reset counting register.
        TA0CCR2 = 25000;                // Set threshold to 25 ms.
        TA0CCTL2 |= CCIE;               // Enable interrupt capability.
        TA0CTL |= MC__CONTINUOUS;       // Start timer in up mode.

        break;
    case P4IV_P4IFG6:                   // P4.6
        break;
    case P4IV_P4IFG7:                   // P4.7
        break;
    }
}

/* ISR TIMER A0 - CCR0 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
{                                       // TA0 CCR0

}

/* ISR TIMER A0 - CCR1, CCR2 AND TAIFG */
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer0_A1_ISR (void)
{
    switch(__even_in_range(TA0IV, TA0IV_TA0IFG))
    {
    case TA0IV_TA0CCR1:                 // TA0 CCR1

        /* Complete Debouncing Button S2 */

        TA0CTL &= ~MC__CONTINUOUS;      // Stop timer.
        TA0CCTL1 &= ~CCIE;              // Disable interrupt capability.
        TA0CCR1 = 0;                    // Disable comparator.

        // Determine state of button:
        if(!(P1IN & BIT1))              // S2 pushed:
        {
            // Wait for button release.
            P1IES &= ~BIT1;             // Enable rising edge detection.
            P1IFG &= ~BIT1;             // Drop meanwhile raised flag.
            P1IE |= BIT1;               // Enable port interrupt again.
        }
        else                            // S2 released:
        {
            // Wait for pushed button again.
            P1IES |= BIT1;              // Enable falling edge detection.
            P1IFG &= ~BIT1;             // Drop meanwhile raised flag.
            P1IE |= BIT1;               // Enable port interrupt again.

            /* VALIDATE BUTTON S2 */

            // Validate current state of main FSM:
            // Button S2 only valid in states LED1 and LED2.
            if(States.Main == MAIN_LED1 || States.Main == MAIN_LED2)
            {
                States.But2 = 1;        // Raise button S2 event flag.
            }
        }

        break;
    case TA0IV_TA0CCR2:                 // TA0 CCR2

        /* Complete Debouncing Button S1 */

        TA0CTL &= ~MC__CONTINUOUS;      // Stop timer.
        TA0CCTL2 &= ~CCIE;              // Disable interrupt capability.
        TA0CCR2 = 0;                    // Disable comparator.

        // Determine state of button:
        if(!(P4IN & BIT5))              // S1 pushed:
        {
            // Wait for button release.
            P4IES &= ~BIT5;             // Enable rising edge detection.
            P4IFG &= ~BIT5;             // Drop meanwhile raised flag.
            P4IE |= BIT5;               // Enable port interrupt again.
        }
        else                            // S1 released:
        {
            // Wait for pushed button again.
            P4IES |= BIT5;              // Enable falling edge detection.
            P4IFG &= ~BIT5;             // Drop meanwhile raised flag.
            P4IE |= BIT5;               // Enable port interrupt again.

            // Validate current state of main FSM:
            // Button S1 valid in all three states.
            States.But1 = 1;            // Raise button S1 event flag.
        }

        break;
    case TA0IV_TA0IFG:                  // TA0 TAIFG

        break;
    }
}
