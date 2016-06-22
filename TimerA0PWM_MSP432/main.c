// main.c
// Runs on MSP432
// Squarewave on P7.3 using TimerA0 TA0.CCR0
// PWM on P2.4 using TimerA0 TA0.CCR1
// PWM on P2.5 using TimerA0 TA0.CCR2
// MCLK = SMCLK = 3MHz DCO; ACLK = 32.768kHz
// TACCR0 generates a square wave of freq ACLK/1024 =32Hz
// Derived from msp432p401_portmap_01.c in MSPware
// Jonathan Valvano
// July 24, 2015

/* This example accompanies the book
   "Embedded Systems: Interfacing to the MSP432 Microcontroller",
   ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2015
   Volume 2, Program 9.8

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
 // Wheel has 20 holes, so 20 ticks per revolution

#include <stdint.h>
#include "msp432p401r.h"
#include "ClockSystem.h"
#include "SysTickInts.h"
#include "PWM.h"
#include "InputCapture.h"
#include "ADC14.h"
#include <math.h>

#ifdef __TI_COMPILER_VERSION__
  //Code Composer Studio Code
  void Delay(uint32_t ulCount){
  __asm (  "dloop:   subs    r0, #1\n"
      "    bne     dloop\n");
}

#else
  //Keil uVision Code
  __asm void
  Delay(uint32_t ulCount)
  {
    subs    r0, #1
    bne     Delay
    bx      lr
  }
#endif
	
#define ARRAY_SIZE 5

void TimerCapture_Init01(void(*task)(uint16_t time), void(*task2)(uint16_t time));
void TimerCapture_Init3(void);
void TimerCapture_Init2(void);
void turn90Right(int);
void turn90Left(int);
int findError();
int getd();
volatile int Speed;
extern int inchesRB; //assuming inches is a global int in inputcapture.c
extern int inchesRF;
uint32_t measureSum;
uint16_t measureAvg;
void control();
void otherControl();
void turnRight();
void goStraight();
void initArray();
void updateArray();
int getAverageOfArray();
int getRFaverage();


void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

uint16_t Period;              // (1/SMCLK) units = 83.3 ns units
uint16_t Period2;              // (1/SMCLK) units = 83.3 ns units
uint16_t First;               // Timer A0 first edge
uint16_t First2;
int Done;                     // set each rising
int Done2;                     // set each rising
int counter = 0, counter2 = 0, measureCounter = 0;


int DESIRED_DISTANCE = 8; //inches
int error;
float integral;
float dt = .06;
float Kp1 = 20;
float Kp2 = 20;
float Kd = 8;
float Ki = .5;
int previousError;
int dErrorR;
int iErrorR;
int updateR;
int dErrorL;
int iErrorL;
int updateL;
int speedRight;
int speedLeft;
int oldError;
int E;
int oldInches;
int oldSpeedLeft;
int oldSpeedRight;
int noTurn = 0;
int averageDistance [ARRAY_SIZE];
int arrayIndex;
int averageRF [ARRAY_SIZE];
int arrayIndexRF;



// 50Hz squarewave on P7.3
// P2.4=1 when timer equals TA0CCR1 on way down, P2.4=0 when timer equals TA0CCR1 on way up
// P2.5=1 when timer equals TA0CCR2 on way down, P2.5=0 when timer equals TA0CCR2 on way up
void PeriodMeasure(uint16_t time){
	/* //light up when triggered
	if (counter >= 20)
	{
	P2OUT = P2OUT^0x02;              // toggle P2.1
  Period = (time - First)&0xFFFF;  // 16 bits, 83.3 ns resolution
  First = time;                    // setup for next
  Done = 1;
	counter = 0; 
	}
	else
	{
		P2OUT = P2OUT^0x04;
	}
	counter++;
	*/
	counter++;
	
	//ClockCounter++; 
}

void PeriodMeasure2(uint16_t time){
	/* //light up when triggered
	if (counter >= 20)
	{
	P2OUT = P2OUT^0x02;              // toggle P2.1
  Period = (time - First)&0xFFFF;  // 16 bits, 83.3 ns resolution
  First = time;                    // setup for next
  Done = 1;
	
	//turn90Right(counter);
	counter = 0;
	}
	else
	{
		P2OUT = P2OUT^0x04;
	}
	counter++;
	*/
	counter2++;
}

void control()
{
	float tempKp;
	error = inchesRB - inchesRF;
	E = E + error;
	
	dErrorR = (error - previousError) / dt;
	updateR = (Kp2 * error) + (Kd * dErrorR);// + (Ki * E);
	
	dErrorL = (error - previousError) / dt;
	updateL = (Kp1 * error) + (Kd * dErrorL);// + (Ki * E);
	
	if (!(speedLeft - updateL >= 14000 || speedLeft - updateL <= 6000))
	{
			speedLeft = speedLeft - updateL;
			previousError = error;
	}
	else
	{
		E = E - error;
	}
	if (!(speedRight + updateR >= 14000 || speedRight + updateR <= 6000))
	{
			speedRight = speedRight + updateR;
			previousError = error;
	}
	else
	{
		E = E - error;
	}

	int average = getAverageOfArray();
	if (inchesRB > 20 && inchesRF > 20 && noTurn != 1)
	{
		turnRight();
	}
	else if (average >= 35)
	{
		goStraight();
	}
	else
	{
		PWM_Duty1(speedRight * .95);
		PWM_Duty2(speedLeft);
	}
	updateArray();
}

void goStraight()
{
	P2OUT = P2OUT^0x04;              // toggle P2.1
	PWM_Duty1(9500);
	PWM_Duty2(10000);
}

void turnRight()
{
	noTurn = 1;
	PWM_Duty1(5000);
	PWM_Duty2(14000);
	Delay(10000000);
	E = 0;
	previousError = 0;
	P2OUT = P2OUT^0x01;              // toggle P2.0
}

void initArray()
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE; i++)
	{
		averageDistance[i] = inchesRB;
	}
	for (int i = 0; i < ARRAY_SIZE; i++)
	{
		averageRF[i] = inchesRF;
	}
}

void updateArray()
{
	if (arrayIndex >= 5)
	{
		arrayIndex = 0;
		arrayIndexRF = 0;
	}
	averageDistance[arrayIndex] = (inchesRB + inchesRF) / 2;
	averageRF[arrayIndexRF] = inchesRF / 2;
	arrayIndex++;
	arrayIndexRF++;
}

int getAverageOfArray()
{
	int i = 0, average = 0;
	for (i = 0; i < ARRAY_SIZE; i++)
	{
		average += averageDistance[i];
	}
	average = average / ARRAY_SIZE;
	return average;
}

int getRFaverage()
{
	int i = 0, avgRF = 0;
	for (i = 0; i < ARRAY_SIZE; i++)
	{
		avgRF += averageRF[i];
	}
	avgRF = avgRF / ARRAY_SIZE;
}

int main(void){int i; uint16_t duty;
	//SysTick_Init(10);
  PWM_Init(15000,9000,9000); //while(1){};
	speedRight = 9000;
	speedLeft = 9000;
  // Squarewave Period/4 (20ms)
  // P2.4 CCR1 75% PWM duty cycle, 10ms period
  // P2.5 CCR2 25% PWM duty cycle, 10ms period
  //PWM_Init(10,7,2);//   while(1){};
  // Squarewave Period/4 (20us)
  // P2.4 CCR1 66% PWM duty cycle, 13.3us period
  // P2.5 CCR2 33% PWM duty cycle, 10us period
	P4DIR |= 0x3A;          
  P4SEL0 &= ~0x3A;       
  P4SEL1 &= ~0x3A;  
	P4OUT |= 0x12; //choose 4.4 and 4.1
	P4OUT &= ~0x28; //4.3 and 4.5 
	
	
	First = 0;                       // first will be wrong
  Done = 0;                        // set on subsequent
  // initialize P2.2-P2.0 and make them outputs (P2.2-P2.0 built-in RGB LEDs)
  P2SEL0 &= ~0x07;
  P2SEL1 &= ~0x07;                 // configure built-in RGB LEDs as GPIO
  P2DS |= 0x07;                    // make built-in RGB LEDs high drive strength
  P2DIR |= 0x07;                   // make built-in RGB LEDs out
  P2OUT &= ~0x07;                  // RGB = off
  Clock_Init48MHz();               // 48 MHz clock; 12 MHz Timer A clock
  TimerCapture_Init01(&PeriodMeasure, &PeriodMeasure2);// initialize Timer A0 in capture mode
	TimerCapture_Init3();// initialize Timer A0 in capture mode
	TimerCapture_Init2();
	P5SEL0 &= ~0x06;
  P5SEL1 &= ~0x06;                 // configure P9.2 as TA0CCP0
	P5DS |= 0x06;
  P5DIR |= 0x06;
	
	initArray();
  while(1){
	
		
			P5OUT |= 0x02; //5.1
			Delay(359940); 
			P5OUT &= ~0x02;
			Delay(60); //59990  
		
			P5OUT |= 0x04; //5.2
			Delay(359940);
			P5OUT &= ~0x04;
			Delay(60); //59990
		
			control();			
			//goStraight();
  } 
}
