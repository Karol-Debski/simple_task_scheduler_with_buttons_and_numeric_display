/*
 * scheduler.h
 *
 *  Created on: Aug 30, 2024
 *      Author: karol
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_


#define TASKS_NUM 3

#define SIZE_TASK_STACK          1024U
#define SIZE_SCHED_STACK         1024U

#define SRAM_START               0x20000000U
#define SIZE_SRAM                ( (128) * (1024))
#define SRAM_END                 ((SRAM_START) + (SIZE_SRAM) )

#define TASK_1_STACK_START 		SRAM_END
#define TASK_2_STACK_START 		( (SRAM_END) - (1 * SIZE_TASK_STACK) )
#define TASK_3_STACK_START 		( (SRAM_END) - (2 * SIZE_TASK_STACK) )
#define SHEDULER_STACK_START	( (SRAM_END) - (3 * SIZE_TASK_STACK) )


#define INIT_VAL_OF_XPSR_REG 0x1000000U

#define INIT_VAL_OF_LR_REG 0xFFFFFFFDU


__attribute__((naked)) void initSchedulerStack(uint32_t addressOfStackStart);

__attribute__((naked)) void switchSPtoPSP(void);

__attribute__((naked)) void SysTick_Handler(void);

void initTasksStack(void);

uint32_t getPspValue(void);

void chooseNextTask(void);


typedef void (*taskPointer)(void);

extern void task1Handler(void);
extern void task2Handler(void);
extern void task3Handler(void);

#endif /* INC_SCHEDULER_H_ */
