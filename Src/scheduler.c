/*
 * scheduler.c
 *
 *  Created on: Aug 30, 2024
 *      Author: karol
 */
#include <stdint.h>

#include "scheduler.h"

static uint32_t pspRegValOfTasks[TASKS_NUM] = {TASK_1_STACK_START, TASK_2_STACK_START, TASK_3_STACK_START};

taskPointer taskHandlers[TASKS_NUM];

uint8_t currentTaskNum=0;



void addTaskHandlers(void)
{
	taskHandlers[0]=(taskPointer)task1Handler;
	taskHandlers[1]=(taskPointer)task2Handler;
	taskHandlers[2]=(taskPointer)task3Handler;
}

uint32_t getPspValue(void)
{
	return pspRegValOfTasks[currentTaskNum];
}

void setPspValue(uint32_t pspValue)
{
	pspRegValOfTasks[currentTaskNum] = pspValue;
}

__attribute__((naked)) void switchSPtoPSP(void)
{
	__asm volatile("PUSH {LR}");
	//get PSP value of current task, store in R0, corrupting LR
	__asm volatile("BL getPspValue");
	__asm volatile("MSR PSP, R0");
	__asm volatile("POP {LR}");
	__asm volatile("MOV R0, #0X02");
	//change SP to PSP by using CONTROL register
	__asm volatile("MSR CONTROL, R0");
	//connection to main function
	__asm volatile("BX LR");
}

//no prologue and epilogue of function
__attribute__((naked)) void initSchedulerStack(uint32_t addressOfStackStart)
{
	//argument of function is stored in R0
	__asm volatile("MSR MSP,%0": :  "r" (addressOfStackStart)  :   );
	//return to previous function, copy LR register to PC register
	__asm volatile("BX LR");
}

void initTasksStack()
{
	uint32_t *pPSP;

	for(int i = 0; i < TASKS_NUM; i++)
	{
		pPSP = (uint32_t*)(pspRegValOfTasks[i]);
		//due to full descending model
		pPSP--;
		// XPSR reg
		*pPSP = INIT_VAL_OF_XPSR_REG;

		pPSP--;
		// PC reg
		*pPSP = (uint32_t)(taskHandlers[i]);

		pPSP--;
		// LR reg
		*pPSP = INIT_VAL_OF_LR_REG;

		//init general purpose registers of core to 0
		for(int j = 0 ; j < 13 ; j++)
		{
			pPSP--;
			*pPSP = 0;
		}

		pspRegValOfTasks[i] = (uint32_t)pPSP;
	}
}

void chooseNextTask(void)
{
	currentTaskNum++;
	currentTaskNum %= TASKS_NUM;
}

__attribute__((naked)) void EXTI1_IRQHandler(void)
{
	//part of the "state of the task" is already stored on the task stack, but R4-R11 not
	__asm volatile("MRS R0, PSP");
	//decrement address on the R0 and store registers on the task stack
	__asm volatile("STMDB R0!,{R4-R11}");
	//save the current PSP value of the task
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
