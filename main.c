#include <stm32f0xx_gpio.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_exti.h>

/*
 * SR04 Ultrasonic range finder example program using
 * the STM32F0 Discovery board available from STMicroelectronics
 *
 * Author: Harris Shallcross
 * Year: ~18/8/2014
 *
 *A relatively simple library that implements external interrupts,
 *timers, the NVIC and GPIO peripherals to sent a 10us pulse to
 *SR04 ultrasonic rangefinder. This example then measures the returned
 *pulse and converts that pulse into a distance using various calculations
 *and precalculated constants. The distance is calculated in meters using
 *floating point calculations so it isn't particularly efficient and
 *someone could probably easily streamline it! The value of distance is
 *read from the microcontroller using the debug feature build into the
 *STLink and CooCox.
 *
 *Code and example descriptions can be found on my blog at:
 *www.hsel.co.uk
 *
 *The code is released under the CC-BY license.
 *
 *This code is provided AS IS and no warranty is included!
 */

//SR04 definitions
#define SR04_Echo 		GPIO_Pin_1
#define SR04_EchoLine 	EXTI_Line1
#define SR04_EchoIRQ 	EXTI0_1_IRQn

#define SR04_TIMIRQ 	TIM2_IRQn
#define SR04_Prescaler 	2
#define SR04_Trig 		GPIO_Pin_0

#define SR04_GPIO 		GPIOA
#define SR04_Timer 		TIM2

#define FallingEdge 0
#define RisingEdge 1

//Peripheral type definitions
GPIO_InitTypeDef G;
TIM_TimeBaseInitTypeDef TB;
NVIC_InitTypeDef N;
EXTI_InitTypeDef E;
RCC_ClocksTypeDef RC;

//Volatile interrupts
volatile uint8_t PulseEnded = 1, InterruptEdge = 0, TimerOverflow = 0;
volatile uint32_t PulseTime = 0;
volatile uint32_t MSec = 0;

//This function initializes the edge interrupt for the SR04 ultrasonic
//sensor and uses the SR04 timer (Timer 2 in the example) to create the
//10us pulse required for the sensor. Once the 10us pulse has been
//generated, the timer is disabled and the variable InterruptEdge
//is set to 0 to denote the next interrupt will be the rising edge.
void SR04_SendPulse(void){
	//Ensure that the timer is initially disabled
	TIM_Cmd(SR04_Timer, DISABLE);

	//Set the timer count to 0
	TIM_SetCounter(SR04_Timer, 0);

	//Ensure that PulseEnded and TimerOverflow are zero as the SR04
	//hasn't even been sent the pulse yet!
	PulseEnded = 0;
	TimerOverflow = 0;

	//Enable the timer
	TIM_Cmd(SR04_Timer, ENABLE);
	//Set the trigger pin high
	GPIO_SetBits(SR04_GPIO, SR04_Trig);

	//Wait until atleast 10us. With a prescalar of 1 the timer will
	//need to count up to atleast 250 until the trigger pin can go
	//low.
	while(TIM_GetCounter(SR04_Timer) < (500/(SR04_Prescaler+1))){
		GPIO_SetBits(SR04_GPIO, SR04_Trig);
	}

	//After 10us has passed, the trigger pin can go low.
	GPIO_ResetBits(SR04_GPIO, SR04_Trig);

	//Disable the timer so it doesnt carry on running and potentially
	//overflow.
	TIM_Cmd(SR04_Timer, DISABLE);

	//Set the current interrupt edge to be detected as rising. The SR04
	//ultrasonic sensor has a brief pause after the trigger signal where
	//8 40KHz pulses are sent. After these 40KHz pulses, the echo signal
	//goes high and the time that it is high is the time it took for the
	//8 pulses to leave the transducer, hit an object and return!
	InterruptEdge = RisingEdge;

	//Reset the counter back to 0 to that the counter can start counting
	//essentially as soon as the rising edge interrupt is detected.
	TIM_SetCounter(SR04_Timer, 0);
}

//In here is the external line interrupt handler. The external line used
//is defined by SR04_EchoLine. In the example, I'm using EXTI Line 1.
void EXTI0_1_IRQHandler(void){
	//If the interrupt has been triggered
	if(EXTI_GetITStatus(SR04_EchoLine) == SET){
		//Clear the interrupt pending bit
		EXTI_ClearITPendingBit(SR04_EchoLine);

		//On rising edge (Interrupt Edge will be 0 on rising edge)
		if(InterruptEdge == RisingEdge){
			//Start the counter to time the length of the returned pulse!
			TIM_Cmd(SR04_Timer, ENABLE);

			//Set the next edge to be falling. The 0 and 1 convention
			InterruptEdge = FallingEdge;
		}
		else{ //On falling edge (Interrupt Edge will be 1 on falling edge)
			//Disable the timer to stop the count
			TIM_Cmd(SR04_Timer, DISABLE);

			//Set PulseTime to the length the timer was counting for
			PulseTime = TIM_GetCounter(SR04_Timer);

			//Indicate that the SR04 returned pulse has ended.
			PulseEnded = 1;
		}
	}
}

//I have enabled the Timer overflow interrupt for the timer that I'm using
//which is timer 2 in the example. By enabling this interrupt, I can detect
//whether the timer timed out before a pulse was returned. Timer 2 is a 32
//bit timer and has a pretty long time to timeout but this also stops the
//main loop code from blocking if the SR04 somehow gets disconnected!

void TIM2_IRQHandler(void){

	//Timer overflowed before pulse ended (Update interrupt)
	if(TIM_GetITStatus(SR04_Timer, TIM_IT_Update) == SET){
		//Clear the pending interrupt bit
		TIM_ClearITPendingBit(SR04_Timer, TIM_IT_Update);

		//Disable the timer to stop further counting
		TIM_Cmd(SR04_Timer, DISABLE);

		//Set the TimerOverflow flag and PulseEnded flag
		TimerOverflow = 1;
		PulseEnded = 1;
	}
}

//Standard SysTick time keeping interrupt! Increments the variable MSec
//every millisecond.
void SysTick_Handler(void){
	MSec++;
}

//Simple millisecond delay function as explained in nearly every code upload
//I do! Nonetheless, the while loop will execure the "nop" instruction until
//"T" amount of MSec (milliseconds) has passed.
void Delay(uint32_t T){
	volatile uint32_t MSS = MSec;
	while((MSec-MSS)<T) asm volatile("nop");
}

int main(void)
{
	//Enable clocks to GPIO and Timer 2. Change these
	//if different pins/peripherals are used!
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//Configure the SysTick for millisecond interrupts
	SysTick_Config(SystemCoreClock/1000);

	//Configure the SR04_Trigger pin as an output
	G.GPIO_Pin = SR04_Trig;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_Mode = GPIO_Mode_OUT;
	G.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(SR04_GPIO, &G);

	//Configure the SR04_Echo pin as an input
	G.GPIO_Pin = SR04_Echo;
	G.GPIO_PuPd = GPIO_PuPd_UP;
	G.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(SR04_GPIO, &G);

	//Setup the SR04 timer to run as fast as possible with the longest
	//time period possible. Timer 2 which is used in this example allows
	//for a time period of 2^32 - 1 = ~4.29billion. This equates to a total
	//time-able period of 48e6/(2^32 - 1) = 11.2ms. If the sensor could
	//work at this kind of distance, the sensor would be able to measure up
	//to 1.92m! By increasing the prescaler, this maximum distance can be
	//increased. The example uses a prescaler of 2, increasing this distance
	//up to 3.84m.
	TB.TIM_ClockDivision = TIM_CKD_DIV1;
	TB.TIM_CounterMode = TIM_CounterMode_Up;
	TB.TIM_Prescaler = SR04_Prescaler;
	TB.TIM_Period = ((uint64_t)1<<32) - 1;
	TIM_TimeBaseInit(SR04_Timer, &TB);

	//Setup the worst case scenario overflow interrupt and clear the pending
	//bit incase is was previously set!
	TIM_ClearITPendingBit(SR04_Timer, TIM_IT_Update);
	TIM_ITConfig(SR04_Timer, TIM_IT_Update, ENABLE);

	//Enable an EXTI interrupt on our EchoLine. In this example, this line will
	//be line 1. Make sure the interrupt is present on both the rising and
	//falling edge of the input. This allows us to relatively precisely time
	//the output pulse from the SR04.
	E.EXTI_Line = SR04_EchoLine;
	E.EXTI_LineCmd = ENABLE;
	E.EXTI_Mode = EXTI_Mode_Interrupt;
	E.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&E);

	//Enable the timer update interrupt
	N.NVIC_IRQChannel = SR04_TIMIRQ;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPriority = 1;
	NVIC_Init(&N);

	//Enable the EXTI line interrupt
	N.NVIC_IRQChannel = SR04_EchoIRQ;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPriority = 0;
	NVIC_Init(&N);

	//Get the clock frequency of the timer. This is required for the calculation
	//of distance.
	RCC_GetClocksFreq(&RC);

	//Initialize the float (yuk!) variables.
	float Distance = 0.0f, TimeOfFlight = 0.0f;

	//The real life time in seconds for one timer tick will be stored in this
	//variable.
	float TTimePerCnt = (SR04_Prescaler+1)/(float)RC.PCLK_Frequency;

	//Everybody's favourite temperature dependent constant!
	const float SpeedOfSound = 343.0f;

	uint8_t PulseSent = 0;

	while(1)
	{
		//Initially send the pulse. Using a function makes this much easier
		//and more portable, simplifying the main code.
		if(PulseSent == 0){
			PulseSent = 1;
			SR04_SendPulse();
		}


		//Instead of using a blocking check, by simply using an if statement and
		//an additional variable named PulseSent, we can poll to check if the
		//returned pulse has been measured. This allows the program loops to do
		//other things while waiting for the returned pulse!
		if(PulseSent && PulseEnded){
			PulseSent = 0;
			//If the timer overflowed, set PulseTime to zero. The value stored in
			//PulseTime could be checked to see if the timer overflowed and a
			//suitable error message could be displayed in this condition.
			if(TimerOverflow){
				PulseTime = 0;
			}
			else{ //If the timer however didn't over flow, calculate the distance
				//Calculate the one way time of flight of the ultrasonic pulse.
				//To ensure that the time of flight is only one way, the PulseTime
				//is divided by two as PulseTime is the time taken for the pulses
				//from the SR04 to be emitted, hit the object and bounce back.
				//Therefore, by halving this, the one way pulsetime can be found.
				TimeOfFlight = TTimePerCnt*((float)PulseTime/2.0f);

				//SIDOT - Speed is Distance over Time or more so S = V*T
				Distance = SpeedOfSound*TimeOfFlight;
			}

			//Delay 1ms until next pulse to give the module a breather!
			Delay(1);
		}
	}
}
