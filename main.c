#include <msp430.h>
#include <math.h>

double averager[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //Array for averaging values
double averager_input[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; //Array for averaging values
//Declarations
float average_TEMP = 0; //Average temperature sent through UART
float average_pot_temp = 0;
float ADC_Voltage = 0; //Calculated voltage from digital signal
float ADC_Temp = 0; //Temperature with decimal points
float pot_voltage = 0;
float pot_temp = 0;
volatile float error;
volatile float previousError;
volatile float gain;
volatile float integral;
volatile float derivative;
volatile float dt = 0.01;
float PID_pwm = 0;
float pwm = 0;
float perBit = 3.3 / 4096; //Constant for converting digital signal to analog sampled voltage

#define kp 15.5
#define ki 5.75
#define kd 6.75

void ADC_Setup();
void PWM_Setup();
void input_TEMP();
float movingAverager_Pot();
float movingAverager_Temp();

void ADC_Setup()
{
    P6SEL = 0x0F;                             // Enable A/D channel inputs
    ADC12CTL0 = ADC12ON+ADC12MSC+ADC12SHT0_8; // Turn on ADC12, extend sampling time
                                            // to avoid overflow of results
    ADC12CTL1 = ADC12SHP+ADC12CONSEQ_3;       // Use sampling timer, repeated sequence
    ADC12MCTL0 = ADC12INCH_0;                 // ref+=AVcc, channel = A0
    ADC12MCTL1 = ADC12INCH_1;                 // ref+=AVcc, channel = A1
    ADC12MCTL2 = ADC12INCH_2;                 // ref+=AVcc, channel = A2
    ADC12MCTL3 = ADC12INCH_3+ADC12EOS;        // ref+=AVcc, channel = A3, end seq.
    ADC12IE = 0x08;                           // Enable ADC12IFG.3
    ADC12CTL0 |= ADC12ENC;                    // Enable conversions
    ADC12CTL0 |= ADC12SC;                     // Start convn - software trigger

}

void PWM_Setup()
{
    P1DIR |= BIT3;                            // P2.5 output
    P1SEL |= BIT3;                            // 2.5 TA2.2
    TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, up mode, clear timer
    TA0CCR0 = 999;                            // PWM Period, 1kHz
    TA0CCR2 = 0;                            // Initial PWM set at 20%
    TA0CCTL2 = OUTMOD_3;                      // Hardware PWM, CCR2 toggle/set
}
float movingAverager_Temp(){
      averager[15] = averager[14];
      averager[14] = averager[13];
      averager[13] = averager[12];
      averager[12] = averager[11];
      averager[11] = averager[10];
      averager[10] = averager[9];
      averager[9] = averager[8];
      averager[8] = averager[7];
      averager[7] = averager[6];
      averager[6] = averager[5];
      averager[5] = averager[4];
      averager[4] = averager[3];
      averager[3] = averager[2];
      averager[2] = averager[1];
      averager[1] = averager[0];
      averager[0] = ADC_Temp; //First value in array is the current temperature reading
      //Average calculated by SUM/16
      return(averager[15] + averager[14] + averager[13] + averager[12] + averager[11] + averager[10] + averager[9] + averager[8] +
              averager[7] + averager[6] + averager[5] + averager[4] + averager[3] + averager[2] + averager[1] + averager[0]) / 16;
}

float movingAverager_Pot(){
    averager[15] = averager[14];
    averager[14] = averager[13];
    averager[13] = averager[12];
    averager[12] = averager[11];
    averager[11] = averager[10];
    averager[10] = averager[9];
    averager[9] = averager[8];
    averager[8] = averager[7];
    averager[7] = averager[6];
    averager[6] = averager[5];
    averager[5] = averager[4];
    averager[4] = averager[3];
    averager[3] = averager[2];
    averager[2] = averager[1];
    averager[1] = averager[0];
    averager[0] = ADC_Temp; //First value in array is the current temperature reading
    //Average calculated by SUM/16
    return(averager_input[7] + averager_input[6] + averager_input[5] + averager_input[4]
         + averager_input[3] + averager_input[2] + averager_input[1] + averager_input[0]) / 8;
}

void inputTEMP()
{
    ADC_Voltage = ADC12MEM1 * perBit; //Conversion from binary sampled voltage to actual
    ADC_Temp = ADC_Voltage * 100; //Equation for PTAT, voltage => temperature in C
    if (ADC_Temp >= 20)
    {
        ADC_Temp += 10;
    }

    pot_voltage = ADC12MEM0 * perBit; //Conversion from binary sampled voltage to actual
    pot_temp = pot_voltage * 80; //Conversion for potentiometer input voltage to desired temperature

    average_TEMP = movingAverager_Temp();

    average_pot_temp = movingAverager_Pot();

}

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    PWM_Setup(); //PWM function call
    ADC_Setup(); //ADC function call

    while (1)
    {
        ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
        __bis_SR_register(LPM0_bits + GIE);
    }
}

//  Analog to Digital Converter interrupt
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)

{
    switch (__even_in_range(ADC12IV, 34))
    {
    case 0:
        break;                           // Vector  0:  No interrupt
    case 2:
        break;                           // Vector  2:  ADC overflow
    case 4:
        break;                           // Vector  4:  ADC timing overflow
    case 6:                                  // Vector  6:  ADC12IFG0
        break;
    case 8:
        break;                           // Vector  8:  ADC12IFG1
    case 10:
        break;                           // Vector 10:  ADC12IFG2
    case 12:
        inputTEMP();
        previousError = error;
        error = average_pot_temp - average_TEMP; //Calculates difference between the desired and current temp
        integral += error * dt; // Integral
        derivative = (error - previousError) / dt; // Derivative
        gain = kp * error;

        if (error > 3.5)
        {
            pwm = 0; // Fan is off
        }
        else if (abs(error) <= 3.5)
        {
            PID_pwm = 600-(gain + (ki * integral) + (kd * derivative)); // PID Controller;
            pwm = PID_pwm;
            if (PID_pwm > 999)
            {
                pwm = 999;
            }
            else if (PID_pwm < 0)
            {
                pwm = 0;
            }
        }
        else if (error < -3.5)
        {
            pwm = 999; // Fan is on
        }
        TA0CCR2 = pwm;
        //__bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
        break;                           // Vector 12:  ADC12IFG3
    case 14:
        break;                           // Vector 14:  ADC12IFG4
    case 16:
        break;                           // Vector 16:  ADC12IFG5
    case 18:
        break;                           // Vector 18:  ADC12IFG6
    case 20:
        break;                           // Vector 20:  ADC12IFG7
    case 22:
        break;                           // Vector 22:  ADC12IFG8
    case 24:
        break;                           // Vector 24:  ADC12IFG9
    case 26:
        break;                           // Vector 26:  ADC12IFG10
    case 28:
        break;                           // Vector 28:  ADC12IFG11
    case 30:
        break;                           // Vector 30:  ADC12IFG12
    case 32:
        break;                           // Vector 32:  ADC12IFG13
    case 34:
        break;                           // Vector 34:  ADC12IFG14
    default:
        break;
    }
}
