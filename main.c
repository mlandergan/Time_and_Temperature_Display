/************** ECE2049 Lab 3 ******************/
/**************  9/27/2017   ******************/
/***************************************************/
#include <stdlib.h>
#include <msp430.h>
#include "peripherals.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#define mV_per_BIT 0.3662 // resolution
#define CALADC12_15V_30C  *((unsigned int *)0x1A1A)
// Temperature Sensor Calibration = Reading at 85 degrees C is stored at addr 1A1Ch
#define CALADC12_15V_85C  *((unsigned int *)0x1A1C)

// Function Prototypes
void runtimerA2(void);
__interrupt void Timer_A2_ISR(void);
void timerDelay(int);
void displayTime(unsigned long);
void temp(unsigned int);
void configureADC();
unsigned int movingAverage(char);
void BuzzerOnFrequency(unsigned int);
unsigned int wheelToTicks(unsigned int);
char button_state();
void configure_buttons();
void activateBuzzer(char, unsigned int);

// Declare globals variables
long int timer = 0;
volatile bool sample = false;
unsigned int in_temp, in_pot;
float temp_cel, temp_far;
unsigned int temp_vals[256];
long int time[256];
char count = 0;
unsigned int moving_avg_array[256];//creating the array for the moving avgd
unsigned int scroll_tick;
char currButton;

#pragma vector=TIMER2_A0_VECTOR
__interrupt void Timer_A2_ISR(void)
{
    timer++;
    sample = true; // flag that it is time to take ADC sample
}


// Main
void main(void)


{
    _BIS_SR(GIE); // Global interrupt enable
    WDTCTL = WDTPW | WDTHOLD;      // Stop watchdog timer
    runtimerA2(); // configure A2 timer
    configDisplay(); // configure display
    configure_buttons(); // configure buttons

    configureADC();
    int i;
    for(i=0; i<256; i++){
        moving_avg_array[i] = 0;
    }

    //Code setup
    Graphics_clearDisplay(&g_sContext); // Clear the display
    long long int time_copy = 0;
    while (1)    // Forever loop
    {
        if(sample) // only sample every second
        {
           // ADC12CTL0 &= ~ADC12SC; //clear start bit
           // ADC12CTL0 |= ADC12SC;
            ADC12CTL0 |= ADC12SC + ADC12ENC;

            // Poll busy bit waiting for conversion to complete
             while (ADC12CTL1 & ADC12BUSY)
             __no_operation();

            time_copy = timer;
            displayTime(time_copy);
            in_temp = ADC12MEM0 & 0x0FFF; // read in ADC value for temp
            in_pot = ADC12MEM1 & 0x0FFF; // read in ADC value for pot
            scroll_tick = wheelToTicks(in_pot);
            currButton = button_state();
            activateBuzzer(currButton,scroll_tick);
            temp(in_temp);
            sample = false;
        }
    }  // end while (1)
}

void activateBuzzer(char button, unsigned int scroll_tick){
    if(button == 0b1000){
    BuzzerOnFrequency(scroll_tick);
    }
    else if(button == 0b0100){
        BuzzerOff();
    }
}

// convert scroll wheel value to ACLK tick count
unsigned int wheelToTicks(unsigned int in_pot){
    unsigned int tick = 0;
    tick = ((in_pot * (0.008547)) + 40);
    return tick;
}


/*
 * Enable a PWM-controlled buzzer on P3.5
 * This function makes use of TimerB0.
 */
void BuzzerOnFrequency(unsigned int scroll)
{
    // Initialize PWM output on P3.5, which corresponds to TB0.5
    P3SEL |= BIT5; // Select peripheral output mode for P3.5
    P3DIR |= BIT5;

    TB0CTL  = (TBSSEL__ACLK|ID__1|MC__UP);  // Configure Timer B0 to use ACLK, divide by 1, up mode
    TB0CTL  &= ~TBIE;                       // Explicitly Disable timer interrupts for safety

    // Now configure the timer period, which controls the PWM period
    // Doing this with a hard coded values is NOT the best method
    // We do it here only as an example. You will fix this in Lab 2.
    TB0CCR0   = scroll + 1;                 // Set the PWM period in ACLK ticks
    TB0CCTL0 &= ~CCIE;                  // Disable timer interrupts

    // Configure CC register 5, which is connected to our PWM pin TB0.5
    TB0CCTL5  = OUTMOD_7;                   // Set/reset mode for PWM
    TB0CCTL5 &= ~CCIE;                      // Disable capture/compare interrupts
    TB0CCR5   = TB0CCR0/2;                  // Configure a 50% duty cycle
}


unsigned int movingAverage(char count){
    int M = 5;

    char j; //j will be used as a counter
    for (j = 0; j < (count+1); j++){
      if (j == 0){ //the first element in temp has nothing to avg in so it is put into the moving_avg_array
        moving_avg_array[j] = temp_vals[j];
      }
      else if (j < M){ //from element 1 to M the data is averaged with everything before it
        moving_avg_array[j] = (temp_vals[j] + (moving_avg_array[j-1]*j))/(j+1);
      }
      else {//there is now enough elements to avg the previous 7 and the current j element
        moving_avg_array[j] = ((moving_avg_array[j-1]*M) - (temp_vals[j-M]) + (temp_vals[j]))/M;
      }
    }
    return moving_avg_array[count];
}


void configureADC(void){
    // Reset REFMSTR to hand over control of internal reference
    // voltages to ADC12_A control registers
    REFCTL0 &= ~REFMSTR;

     // SHT0 = 1001b = 384 cycles = middle of the road, example value
     // ADC12ON = Turn on ADC
     // ADC12REFON = Internal reference on and set to 1.5V
     ADC12CTL0 = ADC12SHT0_9 | ADC12ON | ADC12REFON | ADC12MSC;
     ADC12CTL1 = ADC12SHP | ADC12CONSEQ_1;

     // configure ADC for temp sensor
     // Vref+ = 1.5 v, Vref- = GND
     ADC12MCTL0 = ADC12SREF_1 | ADC12INCH_10;

     // configure ADC for potentiometer
     // Vref+ = 3.3 v, Vref- = GND
     ADC12MCTL1 = ADC12SREF_0 | ADC12INCH_0 | ADC12EOS; //end of sequence;

     P6SEL |= (BIT0); // configure A0
     ADC12CTL0 &= ~ADC12SC; // clear start bit
}


void runtimerA2(void)
{
    // This function configures and starts Timer A2
    // Timer is counting 1 seconds
    // Use ACLK, 16 Bit, up mode, 1 divider
     TA2CTL = TASSEL_1 + MC_1 + ID_0;
     TA2CCR0 = 32767; // 32767+1 = 32768 ACLK tics = 1 second
     TA2CCTL0 = CCIE; // TA2CCR0 interrupt enabled
}

void displayTime(unsigned long global_time){ //global_time is in seconds
  //declare all variables to store all of the month,days, hour, min, sec in the correct format
   int seconds, minutes, hours;
   unsigned long days;
   char dateDisplay[7], timeDisplay[9], months[3];

   //convert to hours
   //convert to Minutes
   //convert to seconds seconds
   seconds = (global_time % 60);
   minutes = ((global_time/60) % 60);
   hours = ((global_time / (60*60)) % 24);
   days =  ((global_time / ((unsigned long)(60L*60L*24L))));  // check this calculation

  //Month, JAN(0-30), FEB(31-58), MAR(59-89), APR(90-119), MAY(120-150), JUN(151-180), JUL(181-211), AUG(212-242), SEP(243-272),
  //OCT(273-303), NOV(304-333), DEC(334-364)
  if ((0 <= days) && (days <=30)){
    months[0] = 'J';
    months[1] = 'A';
    months[2] = 'N';
    days = days + 1;
  }
  else if ((31 <= days) && (days <=58)){
      months[0] = 'F';
      months[1] = 'E';
      months[2] = 'B';
    days = days - 30;
  }
  else if ((59 <= days) && (days <=89)){
      months[0] = 'M';
      months[1] = 'A';
      months[2] = 'R';
    days = days - 58;
  }
  else if ((90 <= days) && (days <=119)){
      months[0] = 'A';
      months[1] = 'P';
      months[2] = 'R';
    days = days - 89;
  }
  else if ((120 <= days) && (days <=150)){
      months[0] = 'M';
      months[1] = 'A';
      months[2] = 'Y';
    days = days - 119;
  }
  else if ((151 <= days) && (days <=180)){
      months[0] = 'J';
      months[1] = 'U';
      months[2] = 'N';
    days = days - 150;
  }
  else if ((181 <= days) && (days <=211)){
      months[0] = 'J';
      months[1] = 'U';
      months[2] = 'L';
    days = days - 180;
  }
  else if ((212 <= days) && (days <=242)){
      months[0] = 'A';
      months[1] = 'U';
      months[2] = 'G';
    days = days - 211;
  }
  else if ((243 <= days) && (days <=272)){
      months[0] = 'S';
      months[1] = 'E';
      months[2] = 'P';
    days = days - 242;
  }
  else if ((273 <= days) && (days <=303)){
      months[0] = 'O';
      months[1] = 'C';
      months[2] = 'T';
    days = days - 272;
  }
  else if ((304 <= days) && (days <=333)){
      months[0] = 'N';
      months[1] = 'O';
      months[2] = 'V';
    days = days - 303;
  }
  else if ((334 <= days) && (days <=364)){
      months[0] = 'D';
      months[1] = 'E';
      months[2] = 'C';
    days = days - 333;
  }


  //AscII array
  //make an array in the correct format, write the correct info as characters into the array in the correct locations
  dateDisplay[0] = months[0];
  dateDisplay[1] = months[1];
  dateDisplay[2] = months[2];
  dateDisplay[3] = '-';
  dateDisplay[4] = (char) (days/10) % 10 + '0';
  dateDisplay[5] = (char) (days % 10) + '0';
  dateDisplay[6] = 0; // null terminator

  timeDisplay[0] = (char) (hours/10) % 10 + '0';
  timeDisplay[1] = (char) (hours % 10) + '0';
  timeDisplay[2] = ':';
  timeDisplay[3] = (char) (minutes/10) % 10 + '0';
  timeDisplay[4] = (char) (minutes % 10) + '0';
  timeDisplay[5] =  ':';
  timeDisplay[6] = (char) (seconds/10) % 10 + '0';
  timeDisplay[7] = (char) (seconds % 10) + '0';
  timeDisplay[8] = 0;

// now write to the display
Graphics_drawStringCentered(&g_sContext, dateDisplay, 7, 48, 15, OPAQUE_TEXT);
Graphics_drawStringCentered(&g_sContext, timeDisplay, 9, 48, 35, OPAQUE_TEXT);
Graphics_flushBuffer(&g_sContext);

}

void temp(unsigned int in_value)
{
    float temp_C, temp_F, degC_per_bit;
    unsigned int temp_C_avg;
    volatile unsigned int bits30,bits85;

    // Use calibration data stored in info memory
    bits30 = CALADC12_15V_30C;
    bits85 = CALADC12_15V_85C;
    degC_per_bit = ((float)(85.0 - 30.0))/((float)(bits85-bits30));

    temp_vals[count] = in_value;
    time[count] = timer-1;

    // call average function
    temp_C_avg = movingAverage(count);

    char temp[13] = {' ', ' ', ' ', '.', ' ', 'C', '/', ' ', ' ', ' ', '.', ' ', 'F'}; // ddd.f C / ddd.f F
    temp_C = (float)((long)temp_C_avg - CALADC12_15V_30C) * degC_per_bit +30.0;
    temp_F = (float)((temp_C* 1.8) + 32); // temp in degrees

    // now we want to format the voltage reading into V.DDD
    // get volts and add 0x30 since ascii '0' starts there
    // find digits in next decimal place
    temp[0] = (int)(temp_C / 100) % 10 + 0x30;
    temp[1] = (int)(temp_C / 10) % 10 + 0x30;
    temp[2] = (int)(temp_C) % 10 + 0x30;
    temp[4] = (int)(temp_C * 10) % 10 + 0x30;
    temp[7] = (int)(temp_F / 100) % 10 + 0x30;
    temp[8] = (int)(temp_F / 10) % 10 + 0x30;
    temp[9] = (int)(temp_F) % 10 + 0x30;
    temp[11] = (int)(temp_F * 10) % 10 + 0x30;

    // now write to the display
    Graphics_drawStringCentered(&g_sContext, temp, 13, 48, 65, OPAQUE_TEXT);
    Graphics_flushBuffer(&g_sContext);
    count++;
}


void timerDelay(int time)
{
  int time_elapsed = 0, timerDelayStart = 0;
  timerDelayStart = timer; // get the current time before entering delay
  while(time_elapsed < time)
  {
      // do nothing!
      time_elapsed = timer - timerDelayStart; // calculate elapsed time
  }
}

void configure_buttons(){
  //P7.0, P3.6, P2.2, P7.4 are the buttons
  P7SEL = P7SEL &~ (BIT0 | BIT4); //DIGITAL I/O P7.0 AND 7.4
  P3SEL = P3SEL &~ (BIT6); // DIGITAL I/O P3.6
  P2SEL = P2SEL &~ (BIT2); //DIGITAL I/O P2.2

  //SET ALL TO INPUTS
  P7DIR = P7DIR &~ (BIT0 | BIT4);
  P3DIR = P3DIR &~ (BIT6);
  P2DIR = P2DIR &~ (BIT2);

  //ENABLE PULLUP/DOWN RESISTORS
  P7REN = P7REN | (BIT0 | BIT4);
  P3REN = P3REN | (BIT6);
  P2REN = P2REN | (BIT2);

  //PULL UP RESISTORS
  P7OUT = P7OUT | (BIT0 | BIT4);
  P3OUT = P3OUT | (BIT6);
  P2OUT = P2OUT | (BIT2);
}


char button_state(){
  //define char for each button and for all together
  char button1_pressed;
  char button2_pressed;
  char button3_pressed;
  char button4_pressed;
  char buttons_pressed;

  //just in case clear all of the bits in the button#_pressed chars
  button1_pressed = button1_pressed &~ (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
  button2_pressed = button2_pressed &~ (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
  button3_pressed = button3_pressed &~ (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
  button4_pressed = button4_pressed &~ (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
  buttons_pressed = buttons_pressed &~ (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);

  //SET button1_pressed BIT 0 to 7.0
  button1_pressed = ~P7IN & BIT0; //in bit 0 position
  button1_pressed = button1_pressed << 3; //moved to bit 3 position


  //SET button2_pressed BIT 1 to P3.6
  button2_pressed = ~P3IN & BIT6; //x1xxxxxx
  button2_pressed = button2_pressed >> 4; //moved to bit 2 position

  //SET button3_pressed BIT 2 to P2.2
  button3_pressed = ~P2IN & BIT2; //in but 2 position
  button3_pressed = button3_pressed >> 1; //moved to bit 1 position

  //SET button4_pressed BIT 3 to P7.4
  button4_pressed = ~P7IN & BIT4;
  button4_pressed = button4_pressed >> 4; //moved to the bit 0 position

  //or the button#_pressed together to get the result
  buttons_pressed = button1_pressed | button2_pressed;// | button3_pressed | button4_pressed; //combine the statements
  return buttons_pressed;
}




