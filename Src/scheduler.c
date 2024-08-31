/*
 * scheduler.c
 *
 *  Created on: Aug 30, 2024
 *      Author: karol
 */
#include <stdint.h>

#include "scheduler.h"
#include "stm32f407xx.h"
#include "seven_segment_display.h"


static uint8_t currentTaskNum=0;

typedef struct
{
	uint32_t pspValue;
	uint32_t lastDisplayDigit;
	void (*taskPointer)(void);
} TaskControlStruct_t;

static TaskControlStruct_t userTasks[TASKS_NUM];



uint32_t getPspValue(void)
{
	return userTasks[currentTaskNum].pspValue;
}

void setPspValue(uint32_t pspValue)
{
	userTasks[currentTaskNum].pspValue = pspValue;
}

//no prologue and epilogue of function
__attribute__((naked)) void switchSPtoPSP(void)
{
	__asm volatile("PUSH {LR}");
	//get PSP value of current task and store in R0 to avoid corrupting LR register
	__asm volatile("BL getPspValue");
	__asm volatile("MSR PSP, R0");
	__asm volatile("POP {LR}");
	__asm volatile("MOV R0, #0X02");
	//change SP to PSP by using CONTROL register
	__asm volatile("MSR CONTROL, R0");
	//back to main function
	__asm volatile("BX LR");
}

//no prologue and epilogue of function
__attribute__((naked)) void initSchedulerStack(uint32_t addressOfStackStart)
{
	//store addressOfStackStart in MSP register
	__asm volatile("MSR MSP,%0": :  "r" (addressOfStackStart)  :   );
	//return to previous function, copy LR register to PC register
	__asm volatile("BX LR");
}

void initTasksStack()
{
	uint32_t *pPSP;

	for(int i = 0; i < TASKS_NUM; i++)
	{
		pPSP = (uint32_t*)(userTasks[i].pspValue);
		//dekrement PSP due to full descending model of stack
		pPSP--;
		// XPSR reg
		*pPSP = INIT_VAL_OF_XPSR_REG;

		pPSP--;
		// PC reg
		*pPSP = (uint32_t)(userTasks[i].taskPointer);

		pPSP--;
		// LR reg
		*pPSP = INIT_VAL_OF_LR_REG;

		//init general purpose registers of core to 0
		for(int j = 0 ; j < 13 ; j++)
		{
			pPSP--;
			*pPSP = 0;
		}

		userTasks[i].pspValue = (uint32_t)pPSP;
	}
}

void initTasks()
{
	userTasks[0].lastDisplayDigit=1;
	userTasks[1].lastDisplayDigit=2;
	userTasks[2].lastDisplayDigit=3;

	userTasks[0].pspValue=TASK_1_STACK_START;
	userTasks[1].pspValue=TASK_2_STACK_START;
	userTasks[2].pspValue=TASK_3_STACK_START;

	userTasks[0].taskPointer=task1Handler;
	userTasks[1].taskPointer=task2Handler;
	userTasks[2].taskPointer=task3Handler;


	initTasksStack();
}

void chooseNextTask(void)
{
	userTasks[currentTaskNum].lastDisplayDigit = getCurrentDigitOnDisplay();

	if (BIT_READ(EXTI->PR, 7) == 1)
	{
		GPIO_IRQHandling(GPIO_PIN_NUM_7);
		currentTaskNum=0;
		//restore hardware setup
		setDigitOnDisplay((uint8_t)userTasks[currentTaskNum].lastDisplayDigit);
	}
	else if (BIT_READ(EXTI->PR, GPIO_PIN_NUM_8) == 1)
	{
		GPIO_IRQHandling(GPIO_PIN_NUM_8);
		currentTaskNum=1;
		//restore hardware setup
		setDigitOnDisplay((uint8_t)userTasks[currentTaskNum].lastDisplayDigit);
	}
	else if (BIT_READ(EXTI->PR, GPIO_PIN_NUM_9) == 1) {
		GPIO_IRQHandling(GPIO_PIN_NUM_9);
		currentTaskNum=2;
		//restore hardware setup
		setDigitOnDisplay((uint8_t)userTasks[currentTaskNum].lastDisplayDigit);
	}
}

//no prologue and epilogue of function
__attribute__((naked)) void EXTI9_5_IRQHandler(void)
{
	//part of the "state of the task" is already stored on the task stack, but R4-R11 not
	__asm volatile("MRS R0, PSP");
	//decrement address on the R0 and store registers on the task stack
	__asm volatile("STMDB R0!,{R4-R11}");
	//save the LR register to avoid corrupting
	__asm volatile("PUSH {LR}");

	__asm volatile("BL setPspValue");

	__asm volatile("BL chooseNextTask");

	__asm volatile("BL getPspValue");
	//retrieve R4-R11
	__asm volatile("LDMIA R0!, {R4-R11}");

	__asm volatile("MSR PSP, R0");

	__asm volatile("POP {LR}");

	__asm volatile("BX LR");
}
