// InputCapture.c
// Runs on MSP432
// Use Timer A0 in capture mode to request interrupts on the rising
// edge of P7.3 (TA0CCP0), and call a user function.
// Daniel Valvano
// July 27, 2015

/* This example accompanies the book
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
   ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2015
   Example 6.1, Program 6.1

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

// external signal connected to P7.3 (TA0CCP0) (trigger on rising edge)

#include <stdint.h>
#include "msp.h"
#include "InputCapture.h"
#include "ADC14.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*CaptureTask)(uint16_t time);// user function
void (*CaptureTask2)(uint16_t time);
void (*CaptureTask3)(uint16_t time);
volatile uint32_t ClockCounter;
volatile uint32_t ClockCounter2;
uint16_t PW1;
uint16_t PW2;
int inchesRB;
int inchesRF;
double	inchesPrecise;

//------------TimerCapture_Init0------------
// Initialize Timer A0 in edge time mode to request interrupts on
// the rising edge of P7.3 (TA0CCP0).  The interrupt service routine
// acknowledges the interrupt and calls a user function.
// Input: task is a pointer to a user function called when edge occurs
//             parameter is 16-bit up-counting timer value when edge occurred
// Output: none
void TimerCapture_Init0(void(*task)(uint16_t time)){long sr;  //For timer 0 (Motor)
	
  sr = StartCritical();
  CaptureTask = task;              // user function
  // initialize P7.3 and make it input (P7.3 TA0CCP0)
  P7SEL0 |= 0x08;
  P7SEL1 &= ~0x08;                 // configure P7.3 as TA0CCP0
  P7DIR &= ~0x08;                  // make P7.3 in
  TA0CTL &= ~0x0030;               // halt Timer A0
	TA0CTL = 0x0200;
	
  TA0CCTL0 = 0x4910;
  TA0EX0 &= ~0x0007;       // configure for input clock divider /1
  NVIC_IPR2 |= (NVIC_IPR2&0xFFFFFF00)|0x00000040; // priority 2
	
// interrupts enabled in the main program after all devices initialized
  NVIC_ISER0 |= 0x00000100; // enable interrupt 8 in NVIC
  TA0CTL |= 0x0024;        // reset and start Timer A0 in continuous up mode
		
  EndCritical(sr);
	
}
void TA0_0_IRQHandler(void){
  TA0CCTL0 &= ~0x0001;             // acknowledge capture/compare interrupt 0
  (*CaptureTask)(TA0CCR0);         // execute user task
	//ClockCounter = ClockCounter+1;
}

void TimerCapture_Init01(void(*task)(uint16_t time), void(*task2)(uint16_t time)){long sr; //Timer 1 (Wheel encoders)
	
  sr = StartCritical();
  CaptureTask = task;              // user function
	CaptureTask2 = task2;              // user function

	P8SEL0 |= 0x01;
  P8SEL1 &= ~0x01;                 // configure P8.0 as TA0CCP0
  P8DIR &= ~0x01;                  // make P8.0 in RIGHT?
	
	P7SEL0 |= 0x10;
  P7SEL1 &= ~0x10;                 // configure P7.4 as TA0CCP0
  P7DIR &= ~0x10;                  // make P7.4 in LEFT?
	
  TA1CTL &= ~0x0030;               // halt Timer A1
	TA1CTL = 0x0200; 

	TA1CCTL0 = 0x4910;
	TA1CCTL4 = 0x4910;
  TA1EX0 &= ~0x0003;       // configure for input clock divider /1
  NVIC_IPR2 |= (NVIC_IPR2&0xFFFFFF00)|0x04040000; // priority 2
// interrupts enabled in the main program after all devices initialized
  NVIC_ISER0 = 0x00000C00; // enable interrupt 8 in NVIC
  TA1CTL |= 0x0024;        // reset and start Timer A1 in continuous up mode
		// these also work NVIC_IPR2 |= (NVIC_IPR2&0xFFFFFF00)|0x00000040; NVIC_ISER0 |= 0x00000100;
  EndCritical(sr);
}
void TA1_0_IRQHandler(void){
  TA1CCTL0 &= ~0x0001;             // acknowledge capture/compare interrupt 0
  (*CaptureTask)(TA1CCR0);         // execute user task
	//PW = (TA1CCR0 - TA1CCR1);
	//ClockCounter2 = ClockCounter2+1;
}
void TA1_N_IRQHandler(void){
  TA1CCTL4 &= ~0x0001;             // acknowledge capture/compare interrupt 0
  (*CaptureTask2)(TA1CCR4);         // execute user task
	//PW = (TA1CCR0 - TA1CCR1);
	//ClockCounter2 = ClockCounter2+1;
}


void TimerCapture_Init2(void){long sr;  //Timer 2 (sonar sensors)
	
 sr = StartCritical();
	
	P5SEL0 |= 0xC0;
  P5SEL1 &= ~0xC0;                 // configure P5.6 and p5.7 as TA0CCP0
  P5DIR &= ~0xC0;                  // make ports in
	
	
  TA2CTL &= ~0x0030;               // halt Timer A3
	TA2CTL = 0x0200;

	TA2CCTL1 = 0x8910;
	TA2CCTL2 = 0x49C0;
  TA2EX0 &= ~0x0007;       // configure for input clock divider /1
  NVIC_IPR3 |= (NVIC_IPR3&0xFFFFFF00)|0x00004000; // priority 2
// interrupts enabled in the main program after all devices initialized
  NVIC_ISER0 |= 0x00002000; // enable interrupt 8 in NVIC
  TA2CTL |= 0x0024;        // reset and start Timer A1 in continuous up mode
		
  EndCritical(sr);
}

void TA2_N_IRQHandler(void){
  TA2CCTL1 &= ~0x0001;             // acknowledge capture/compare interrupt 0
	PW2 = (TA2CCR1 - TA2CCR2);
	inchesRF = (PW2) / 1480;	//
}

  void TimerCapture_Init3(void){long sr;  //Timer 3 (Clock thing)
	
  sr = StartCritical();
	
	P10SEL0 |= 0x10;
  P10SEL1 &= ~0x10;                 // configure P10.4 as TA0CCP0
  P10DIR &= ~0x10;                  // make P10.4 in
	
	P8SEL0 |= 0x04;
  P8SEL1 &= ~0x04;                 // configure 8.2 as TA0CCP
  P8DIR &= ~0x04; 
	
  TA3CTL &= ~0x0030;               // halt Timer A3
	TA3CTL = 0x0200;

	TA3CCTL0 = 0x8910;
	TA3CCTL2 = 0x49C0;
  TA3EX0 &= ~0x0007;       // configure for input clock divider /1
  NVIC_IPR3 |= (NVIC_IPR3&0xFFFFFF00)|0x04040000; // priority 2
// interrupts enabled in the main program after all devices initialized
  NVIC_ISER0 |= 0x00006000; // enable interrupt 8 in NVIC
  TA3CTL |= 0x0024;        // reset and start Timer A1 in continuous up mode
		
  EndCritical(sr);
}

void TA3_0_IRQHandler(void){
  TA3CCTL0 &= ~0x0001;             // acknowledge capture/compare interrupt 0
	PW1 = (TA3CCR0 - TA3CCR2);
	inchesRB = (PW1) / 1480;	//
}
