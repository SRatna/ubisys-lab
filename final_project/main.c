/******************************************************************************
 *                                                                            *
 *    Final Project                                                           *
 *                                                                            *
 ******************************************************************************
 *                                                                            *
 * name:                    Sushanta Ratna Adhikari                           *
 * matriculation number:    1532852                                           *
 * e-mail:                  sushanta.adhikari@student.uni-siegen.de           *
 *                                                                            *
 ******************************************************************************/

#include <msp430fr5969.h>
#include <stdint.h>

#define THRS_INTENSITY 200               // Threshold value for light sensor

volatile struct
{
    uint16_t state : 9;
} System;

typedef enum
{
    SLEEP,
    CHECK_CONTINUOUS_LIGHT,
    CHECK_PREAMBLE,
    WAKEUP,
    TRANSMISSION_ERROR,
    ACTIVATE_LIGHT_SOURCE,
    STRESS_MEASUREMENT,
    HUMIDITY_MEASUREMENT,
    FINAL_TRANSMISSION,
    ACKNOWLEDGEMENT,
} SYSTEM_STATES;

volatile uint16_t adc_val = 0;
volatile uint16_t continuous_light_counter = 0; // count till 3 sec of light source
uint8_t powers_of_two[] = {1, 2, 4, 8, 16, 32, 64, 128};
volatile uint8_t total_preamble = 0;
volatile int8_t preamble_counter = 7; // 7: as we start from msb first
volatile uint8_t measurement_counter = 0;
uint8_t stress_values[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t humidity_values[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void uartTx(uint16_t data);
void handleDataTransmission(uint8_t data[8]);
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

    P1DIR &= ~BIT1;                     // P1.1 - input for S2.

    P4DIR &= ~BIT5;                     // P4.5 - output for S1.

    // enable pull up resistor for P1.1
    P1REN |= BIT1;
    P1OUT |= BIT1;

    // enable pull up resistor for P4.5
    P4REN |= BIT5;
    P4OUT |= BIT5;
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
    while(REFCTL0 & REFGENBUSY);        // If reference generator is busy, wait
    REFCTL0 |= REFVSEL_2 | REFON;       // Select internal reference voltage = 2.5V and switch on the internal reference voltage
    while(!(REFCTL0 & REFGENRDY));      // Waiting for the reference generator to settle

    System.state = SLEEP;

    // Enable interrupts globally.
    __bis_SR_register(GIE);
    /* MAIN LOOP */
    while(1)
    {
        // uartTx(adc_val);
        switch(System.state)
        {
            case SLEEP:
                ADC12CTL0 = 0;                      // Disable ADC trigger.
                ADC12CTL0 |= ADC12ON;               // Turn on ADC.
                ADC12CTL0 |= ADC12MSC;              // Multiple sample and conversion mode.
                ADC12CTL0 |= ADC12SHT0_6;           // Sample-and-hold time of 128 cycles of MODCLK.
                ADC12CTL1 = ADC12SHP;               // Use sampling timer.
                ADC12CTL1 |= ADC12CONSEQ_2;         // Repeat-single-channel mode.
                ADC12CTL2 = ADC12RES__8BIT;         // 8 bit conversion resolution.
                ADC12IER0 |= ADC12IE0;              // Enable ADC conversion interrupt.
                ADC12MCTL0 |= ADC12INCH_10;         // Select ADC input channel A10.
                ADC12MCTL0 |= ADC12VRSEL_1;         // Reference voltages V+ = VREF buffered
                                                    // = 2.5V and V- = AVSS = GND.
                ADC12CTL0 |= ADC12ENC;              // Enable ADC trigger.
                ADC12CTL0 |= ADC12SC;               // Trigger sampling.

                while(ADC12IFGR0 & ADC12IFG0);       // Wait for first 8 bit sample.

                if (adc_val > THRS_INTENSITY)        // Bright.
                {
                    System.state = CHECK_CONTINUOUS_LIGHT;
                }
                else
                {
                    TA0CTL |= MC__CONTINUOUS;            // Timer A1 mode control: Up to CCR0
                    TA0CTL |= TASSEL__ACLK;      // Timer A1 clock source select: ACLK (10KHZ)
                    TA0CCR0 = 0;              // Setting necessary threshold value, this will start the timer
                    TA0CCR1 = 9999;
                    TA0CCTL1 |= CCIE;            // Enable interrupt of CCR0.
                    ADC12CTL0 = 0;                      // Disable ADC trigger.
                    __bis_SR_register(LPM3_bits + GIE);
                }
                break;
            case CHECK_CONTINUOUS_LIGHT:
                // start timer with 20ms frequency
                if (continuous_light_counter == 0) // init timer only once
                {
                    TA0CTL |= MC__UP;            // Timer A0 mode control: Up to CCR0
                    TA0CTL |= TASSEL__ACLK;      // Timer A0 clock source select: ACLK (10KHZ)
                    TA0CCR0 = 199;               // Setting necessary threshold value, this will start the timer: 10k/50hz
                    TA0CCTL0 |= CCIE;            // Enable interrupt of CCR0.
                }
                if (continuous_light_counter > 150) // 150: 3s/20ms; checked for 3 sec of light
                {                                   // note: continuous_light_counter is reset only after final transmission
                    // Stop and turn off timer TA0.
                    TA0CTL = 0;
                    TA0R = 0;
                    System.state = CHECK_PREAMBLE;
                }
                break;
            case TRANSMISSION_ERROR:
                P1OUT |= BIT0;               // Turn on LED 2.
                // start timer for 1sec
                TA1CTL |= MC__UP;            // Timer A1 mode control: Up to CCR0
                TA1CTL |= TASSEL__ACLK;      // Timer A1 clock source select: ACLK (10KHZ)
                TA1CCR0 = 9999;              // Setting necessary threshold value, this will start the timer
                TA1CCTL0 |= CCIE;            // Enable interrupt of CCR0.
                measurement_counter = 0;     // Reset measurement counter
                // reset preamble parameters
                total_preamble = 0;
                preamble_counter = 7;
                // go to sleep
                System.state = SLEEP;
                break;
            case CHECK_PREAMBLE:
                if (preamble_counter == 7) // only once init
                {
                    // every 0.4 sec
                    TA2CTL |= MC__UP;            // Timer A1 mode control: Up to CCR0
                    TA2CTL |= TASSEL__ACLK;      // Timer A1 clock source select: ACLK (10KHZ)
                    TA2CCR0 = 3999;              // Setting necessary threshold value, this will start the timer
                    TA2CCTL0 |= CCIE;            // Enable interrupt of CCR0.
                }
                if (total_preamble == 106) // check for preamble: 01101010 = 106 (in base 10)
                {
                    TA2CTL = 0;
                    TA2R = 0;
                    System.state = ACKNOWLEDGEMENT; // preamble has matched: go to next state
                    total_preamble = 0;
                    preamble_counter = 7;
                }
                else if (preamble_counter < 0) // only after 8 bits of preamble are read: preamble mis-match has occurred
                {
                    TA2CTL = 0;
                    TA2R = 0;
                    System.state = TRANSMISSION_ERROR;
                    total_preamble = 0;
                    preamble_counter = 7;
                }
                break;
            case ACKNOWLEDGEMENT:
                P1OUT |= BIT0;               // Turn on LED 2.
                // start timer for 250ms
                TA1CTL |= MC__UP;            // Timer A1 mode control: Up to CCR0
                TA1CTL |= TASSEL__ACLK;      // Timer A1 clock source select: ACLK (10KHZ)
                TA1CCR0 = 2499;              // Setting necessary threshold value, this will start the timer
                TA1CCTL0 |= CCIE;            // Enable interrupt of CCR0.
                __delay_cycles(250000);      // Delay for 250 ms.
                System.state = WAKEUP;
                break;
            case WAKEUP:
                // start timer to check for light every 20ms
                TA0CTL |= MC__UP;            // Timer A0 mode control: Up to CCR0
                TA0CTL |= TASSEL__ACLK;      // Timer A0 clock source select: ACLK (10KHZ)
                TA0CCR0 = 199;               // Setting necessary threshold value, this will start the timer: 10k/50hz
                TA0CCTL0 |= CCIE;            // Enable interrupt of CCR0.
                System.state = ACTIVATE_LIGHT_SOURCE;
                break;
            case ACTIVATE_LIGHT_SOURCE:
                P4OUT |= BIT6;               // Turn on LED 1.
                // start timer for 10ms
                TA1CTL |= MC__UP;            // Timer A1 mode control: Up to CCR0
                TA1CTL |= TASSEL__ACLK;      // Timer A1 clock source select: ACLK (10KHZ)
                TA1CCR0 = 99;                // Setting necessary threshold value, this will start the timer
                TA1CCTL0 |= CCIE;            // Enable interrupt of CCR0.
                System.state = STRESS_MEASUREMENT;
                break;
            case STRESS_MEASUREMENT:
                if (measurement_counter == 0) // only once init
                {
                    // every 1 sec
                    TA3CTL |= MC__UP;            // Timer A1 mode control: Up to CCR0
                    TA3CTL |= TASSEL__ACLK;      // Timer A1 clock source select: ACLK (10KHZ)
                    TA3CCR0 = 9999;              // Setting necessary threshold value, this will start the timer
                    TA3CCTL0 |= CCIE;            // Enable interrupt of CCR0.
                }
                if (measurement_counter > 7)
                {
                    System.state = HUMIDITY_MEASUREMENT;
                    measurement_counter = 0;
                }
                break;
            case HUMIDITY_MEASUREMENT:
                if (measurement_counter == 0) // only once init
                {
                    // every 1 sec
                    TA3CTL |= MC__UP;            // Timer A1 mode control: Up to CCR0
                    TA3CTL |= TASSEL__ACLK;      // Timer A1 clock source select: ACLK (10KHZ)
                    TA3CCR0 = 9999;              // Setting necessary threshold value, this will start the timer
                    TA3CCTL0 |= CCIE;            // Enable interrupt of CCR0.
                }
                if (measurement_counter > 7)
                {
                    System.state = FINAL_TRANSMISSION;
                    measurement_counter = 0;
                }
                break;
            case FINAL_TRANSMISSION:
                {
                    continuous_light_counter = 0; // reset the at least 3s light counter
                    uint8_t transmission_preamble[8] = {1, 0, 0, 1, 0, 1, 0, 1};
                    handleDataTransmission(transmission_preamble);
                    handleDataTransmission(stress_values);
                    handleDataTransmission(humidity_values);
                    uint8_t i;
                    for (i = 0; i < 8; i++)
                    {
                        stress_values[i] = 0;
                        humidity_values[i] = 0;
                    }
                    System.state = SLEEP;
                    break;
                }
        }
    }
}


/* ### TRIGGER TIMER ### */

/* ISR TIMER A0 - CCR0 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
{
    // this timer to check for continuously check presence of light every 20ms
    if (adc_val > THRS_INTENSITY)
    {
        if (System.state == CHECK_CONTINUOUS_LIGHT)
            continuous_light_counter++;
    }
    else // interruption in continuous light is seen
    {
        System.state = TRANSMISSION_ERROR;
        // Stop and turn off timer TA0.
        TA0CTL = 0;
        TA0R = 0;
    }
}

/* ISR TIMER A0 - CCR1, CCR2 AND TAIFG */
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer0_A1_ISR (void)
{
    switch(__even_in_range(TA0IV, TA0IV_TA0IFG))
    {
    case TA0IV_TA0CCR1:                 // TA0 CCR1
        // stop & reset timer
        TA0CTL = 0;
        TA0R = 0;
        __bic_SR_register_on_exit(LPM3_bits); // go to active mode
        break;

    case TA0IV_TA0IFG:                  // TA0 TAIFG

        break;
    }
}


/* ISR TIMER A1 - CCR0 */
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR (void)
{                                       // TA1 CCR0
    // Stop and turn off timer TA1.
    TA1CTL = 0;
    TA1R = 0;
    P1OUT &= ~BIT0;                     // Turn off LED 2.
    P4OUT &= ~BIT6;                     // Turn LED1 off: during light source activate state
}

/* ISR TIMER A2 - CCR0 */
#pragma vector = TIMER2_A0_VECTOR
__interrupt void Timer2_A0_ISR (void)   // TA2 CCR0
{
    uint8_t i;
    // here overflow has occurred after 400ms and then we check for light's presence 2 times via the loop in interval of 100ms.
    for (i = 0; i < 3; i++) // tolerate 20% deviation i.e. 100ms
    {
        if (adc_val > THRS_INTENSITY && preamble_counter >= 0)
        {
            total_preamble += powers_of_two[preamble_counter]; // powers_of_two array helps us to change binary bit to decimal value
            break;
        }
        __delay_cycles(100000);      // Delay for 100 ms.
    }
    preamble_counter--; // preamble_counter starts from 7 and get decreased here
}

/* ISR TIMER A3 - CCR0 */
#pragma vector = TIMER3_A0_VECTOR
__interrupt void Timer3_A0_ISR (void)
{
    if (measurement_counter > 7)
    {
        TA3CTL = 0;
        TA3R = 0;
    }
    else
    {
        if (System.state == STRESS_MEASUREMENT)
        {
            if (!(P4IN & BIT5))
                stress_values[measurement_counter] = 1; // check if btn1 is pressed
            System.state = ACTIVATE_LIGHT_SOURCE;
        }
        if (System.state == HUMIDITY_MEASUREMENT && !(P1IN & BIT1)) // check if btn2 is pressed
        {
            humidity_values[measurement_counter] = 1;
        }
        measurement_counter++;
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

void handleDataTransmission(uint8_t data[8])
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        P1OUT |= BIT0;               // Turn on LED 2.
        if (data[i])
        {
            __delay_cycles(375000);      // Delay for 375 ms.
            P1OUT &= ~BIT0;              // Turn off LED 2.
            __delay_cycles(125000);      // Delay for 125 ms.
        }
        else
        {
            __delay_cycles(125000);      // Delay for 125 ms.
            P1OUT &= ~BIT0;              // Turn off LED 2.
            __delay_cycles(375000);      // Delay for 375 ms.
        }
    }
}
