/**
 * @file rtos.c
 * @author ITESO
 * @date Feb 2018
 * @brief Implementation of rtos API
 *
 * This is the implementation of the rtos module for the
 * embedded systems II course at ITESO
 */

#include "rtos.h"
#include "rtos_config.h"
#include "clock_config.h"

#ifdef RTOS_ENABLE_IS_ALIVE
#include "fsl_gpio.h"
#include "fsl_port.h"
#endif
/**********************************************************************************/
// Module defines
/**********************************************************************************/

#define FORCE_INLINE 	__attribute__((always_inline)) inline

#define STACK_FRAME_SIZE			8

#define STACK_LR_OFFSET				2

#define STACK_PSR_OFFSET			1
#define STACK_PSR_DEFAULT			0x01000000

/**********************************************************************************/
// IS ALIVE definitions
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
#define CAT_STRING(x,y)  		x##y
#define alive_GPIO(x)			CAT_STRING(GPIO,x)
#define alive_PORT(x)			CAT_STRING(PORT,x)
#define alive_CLOCK(x)			CAT_STRING(kCLOCK_Port,x)
static void init_is_alive(void);
static void refresh_is_alive(void);
#endif

/**********************************************************************************/
// Name substitutions
/**********************************************************************************/

#define current_task_ptr &task_list.tasks[task_list.current_task]
#define newborn_task_ptr &task_list.tasks[task_list.nTasks]
#define idle_task_index 0
#define idle_task_addr &task_list.tasks[idel_task_index]

/**********************************************************************************/
// Type definitions
/**********************************************************************************/

typedef enum
{
	S_READY = 0, S_RUNNING, S_WAITING, S_SUSPENDED
} task_state_e;
typedef enum
{
	kFromISR = 0, kFromNormalExec
} task_switch_type_e;

typedef struct
{
	uint8_t priority;
	task_state_e state;
	uint32_t *sp;
	void (*task_body)();
	rtos_tick_t local_tick;
	uint32_t reserved[10];
	uint32_t stack[RTOS_STACK_SIZE];
} rtos_tcb_t;

/**********************************************************************************/
// Global (static) task list
/**********************************************************************************/

struct
{
	uint8_t nTasks;
	rtos_task_handle_t current_task;
	rtos_task_handle_t next_task;
	rtos_tcb_t tasks[RTOS_MAX_NUMBER_OF_TASKS + 1];
	rtos_tick_t global_tick;
} task_list =
{ 0 };

// Global static flag of first context switch
uint8_t first_context_switch = 0;

/**********************************************************************************/
// Local methods prototypes
/**********************************************************************************/

static void reload_systick(void);
static void dispatcher(task_switch_type_e type);
static void activate_waiting_tasks();
FORCE_INLINE static void context_switch(task_switch_type_e type);
static void idle_task(void);

/**********************************************************************************/
// API implementation
/**********************************************************************************/


void rtos_start_scheduler(void)
{

	/* Enable the "is alive" channel (Green LED). */
#ifdef RTOS_ENABLE_IS_ALIVE
	init_is_alive();
#endif


	/* Enable the SysTick timer. */
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
	        | SysTick_CTRL_ENABLE_Msk;


	/* Reload the SysTick with a 0. */
	reload_systick();


	/* Create idle task. */
	rtos_create_task(idle_task, 0, kAutoStart);


	/* Infinite loop. */
	for (;;);
}

rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority,
		rtos_autostart_e autostart)
{
	/* If there is no more space for tasks, return error. */
		if( (RTOS_MAX_NUMBER_OF_TASKS - 1) < task_list.nTasks )	return (-1);



	/* Task structure:

	uint8_t priority;
	task_state_e state;
	uint32_t *sp;
	void (*task_body)();
	rtos_tick_t local_tick;
	uint32_t reserved[10];
	uint32_t stack[RTOS_STACK_SIZE];
	*/

	/* Add the task to the task list by filling in the structure fields with the
	 * pertinent information.
	 * */
	newborn_task_ptr->priority 	 = priority;
	newborn_task_ptr->state      = (kAutoStart == autostart) ? (S_READY) : (S_SUSPENDED);
	newborn_task_ptr->sp		 = newborn_task_ptr->stack[RTOS_STACK_SIZE - STACK_FRAME_SIZE];
	newborn_task_ptr->task_body  = task_body;
	newborn_task_ptr->local_tick = 0;

	/* Initialize the newborn task's stack frame */
	newborn_task_ptr->stack[RTOS_STACK_SIZE - STACK_PSR_OFFSET] = STACK_PSR_DEFAULT;
	newborn_task_ptr->stack[RTOS_STACK_SIZE - STACK_LR_OFFSET ] = task_body;


	/* Increment the number of tasks on the system. */
	task_list.nTasks++;

	/* Return the newborn task index on the task list. */
	return task_list.nTasks;
}

rtos_tick_t rtos_get_clock(void)
{
	return ( (rtos_tick_t) CLOCK_GetCoreSysClkFreq() );
}

void rtos_delay(rtos_tick_t ticks)
{
	current_task_ptr->state      = S_WAITING;
	current_task_ptr->local_tick = ticks;
	dispatcher(kFromNormalExec);
}

void rtos_suspend_task(void)
{
	current_task_ptr->state = S_SUSPENDED;
	dispatcher(kFromNormalExec);
}

void rtos_activate_task(rtos_task_handle_t task)
{
	current_task_ptr->state = S_READY;
	dispatcher(kFromNormalExec);
}

/**********************************************************************************/
// Local methods implementation
/**********************************************************************************/

static void reload_systick(void)
{
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
}

static void dispatcher(task_switch_type_e type)
{
	uint8_t next_task_index = idle_task_index;
	uint8_t highest_priority_yet = 0;
	/* Search for the highest priority task among the READY/RUNNING tasks. */
	rtos_tcb_t * task_ptr;
	for(task_ptr = idle_task_addr; &task_list.tasks[RTOS_MAX_NUMBER_OF_TASKS] > task_ptr; task_ptr++)
	{

		if(
			(highest_priority_yet < task_ptr->priority)                         &&
			(S_READY == task_ptr->priority || S_RUNNING == task_ptr->priority)
		  )
		{
			next_task_index      = ((uint8_t) (task_ptr - task_list.tasks));
			highest_priority_yet = task_ptr->priority;
		}

	}

	if( task_list.current_task != next_task_index )
	{
		context_switch(kFromNormalExec);
	}


}

FORCE_INLINE static void context_switch(task_switch_type_e type)
{
	if(0 == first_context_switch)
	{
		first_context_switch = 0xF4;	// Arbitrary nonzero value
		register uint32_t SP_reg asm("SP");
		current_task_ptr->sp++;
		*(current_task_ptr->sp) = SP_reg;
	}

	task_list.current_task = task_list.current_task;
	current_task_ptr->state = S_RUNNING;
	PendSV_Handler();
}

static void activate_waiting_tasks()
{
	rtos_tcb_t * task_ptr;
	for(task_ptr = idle_task_addr; &task_list.tasks[RTOS_MAX_NUMBER_OF_TASKS] > task_ptr; task_ptr++)
	{
		if(S_WAITING == task_ptr->state)
		{
			task_ptr->local_tick--;
			if(0 == task_ptr->local_tick) task_ptr->state = S_READY;
		}

	}
}

/**********************************************************************************/
// IDLE TASK
/**********************************************************************************/

static void idle_task(void)
{
	for (;;)
	{

	}
}

/****************************************************/
// ISR implementation
/****************************************************/

void SysTick_Handler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	refresh_is_alive();
#endif
	activate_waiting_tasks();
	reload_systick();
}

void PendSV_Handler(void)
{
	register int32_t r0 asm("r0");
	(void) r0;
	SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
	r0 		   = (int32_t) task_list.tasks[task_list.current_task].sp;
	asm("mov r7, r0");
}

/**********************************************************************************/
// IS ALIVE SIGNAL IMPLEMENTATION
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
static void init_is_alive(void)
{
	gpio_pin_config_t gpio_config =
	{ kGPIO_DigitalOutput, 1, };

	port_pin_config_t port_config =
	{ kPORT_PullDisable, kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
	        kPORT_UnlockRegister, };
	CLOCK_EnableClock(alive_CLOCK(RTOS_IS_ALIVE_PORT));
	PORT_SetPinConfig(alive_PORT(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &port_config);
	GPIO_PinInit(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &gpio_config);
}

static void refresh_is_alive(void)
{
	static uint8_t state = 0;
	static uint32_t count = 0;
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
	if (RTOS_IS_ALIVE_PERIOD_IN_US / RTOS_TIC_PERIOD_IN_US - 1 == count)
	{
		GPIO_PinWrite(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
		        state);
		state = state == 0 ? 1 : 0;
		count = 0;
	} else //
	{
		count++;
	}
}

#endif
///
