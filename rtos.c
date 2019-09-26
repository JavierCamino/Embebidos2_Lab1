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

#define current_task_ptr ( (rtos_tcb_t*) &(task_list.tasks[task_list.current_task]) )
#define newborn_task_ptr ( (rtos_tcb_t*) &(task_list.tasks[task_list.nTasks])       )
#define first_task_addr  ( (rtos_tcb_t*) &(task_list.tasks[0])                      )

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

// Function 1: Verified
void rtos_start_scheduler(void)
{

	/* Enable the "is alive" channel (Green LED). */
#ifdef RTOS_ENABLE_IS_ALIVE
	init_is_alive();
#endif


	/* Enable the SysTick timer. */
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
	/* Reload the SysTick with a 0. */
	reload_systick();
	/* Create idle task */
	task_list.current_task = rtos_create_task(idle_task, 0, kAutoStart);




	/* Infinite loop. */
	for (;;);
}
// Function 2: Verified
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
	newborn_task_ptr->sp		 = &newborn_task_ptr->stack[RTOS_STACK_SIZE - STACK_FRAME_SIZE - 1U];
	newborn_task_ptr->task_body  = task_body;
	newborn_task_ptr->local_tick = 0;

	/* Initialize the newborn task's stack frame */
	newborn_task_ptr->stack[RTOS_STACK_SIZE - STACK_PSR_OFFSET] = STACK_PSR_DEFAULT;
	newborn_task_ptr->stack[RTOS_STACK_SIZE - STACK_LR_OFFSET ] = ((uint32_t) task_body);


	/* Increment the number of tasks on the system. */
	task_list.nTasks++;

	/* Return the newborn task index on the task list. */
	return (task_list.nTasks - 1U);
}
// Function 3: Ready
rtos_tick_t rtos_get_clock(void)
{
	return ( task_list.global_tick );
}
// Function 4: Ready
void rtos_delay(rtos_tick_t ticks)
{
	current_task_ptr->state      = S_WAITING;
	current_task_ptr->local_tick = ticks;
	dispatcher(kFromNormalExec);
}
// Function 5: Ready
void rtos_suspend_task(void)
{
	current_task_ptr->state = S_SUSPENDED;
	dispatcher(kFromNormalExec);
}
// Function 6: Ready
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
// Function 7: Verified
static void dispatcher(task_switch_type_e type)
{
	int8_t next_task_index      = task_list.nTasks - 1U;
	int8_t highest_priority_yet = -1;
	/* Search for the highest priority task among the READY/RUNNING tasks. */
	rtos_tcb_t * task_ptr       = 0U;
	for(task_ptr = first_task_addr; &(task_list.tasks[task_list.nTasks]) > task_ptr; task_ptr++)
	{

		if(
			(highest_priority_yet < task_ptr->priority)                    &&
			(S_READY == task_ptr->state || S_RUNNING == task_ptr->state)
		  )
		{
			next_task_index      = ((uint8_t) (task_ptr - first_task_addr));
			highest_priority_yet = task_ptr->priority;
		}

	}

	/* Update the next task to be executed */
	task_list.next_task = next_task_index;

	/* If necessary, perform a context switch. */
	if( task_list.next_task != task_list.current_task )	context_switch(type);


}
// Function 8: Verified
FORCE_INLINE static void context_switch(task_switch_type_e type)
{
	static uint8_t first_context_switch = 1;
	register uint32_t r0 asm("r0");

	(void) r0;

	if(!first_context_switch)
	{
		asm("mov r0, r7");
		current_task_ptr->sp  = ((uint32_t*) r0);
		current_task_ptr->sp += (kFromNormalExec == type) ? (-(STACK_FRAME_SIZE + 1)) : (STACK_FRAME_SIZE + 1);
	}
	else
	{
		first_context_switch = 0;
	}

	task_list.current_task = task_list.next_task;
	current_task_ptr->state = S_RUNNING;
	SCB->ICSR |= SCB_ICSR_PENDSTSET_Msk;
}
// Function 9: Verified
static void activate_waiting_tasks()
{
	rtos_tcb_t * task_ptr;
	for(task_ptr = first_task_addr; &(task_list.tasks[task_list.nTasks]) > task_ptr; task_ptr++)
	{
		if(S_WAITING == task_ptr->state)
		{

			if(0 == task_ptr->local_tick)
			{
				task_ptr->state = S_READY;
			}
			else
			{
				task_ptr->local_tick--;
			}

		}

	}
}

/**********************************************************************************/
// IDLE TASK
/**********************************************************************************/

static void idle_task(void)
{
	for (;;);
}

/****************************************************/
// ISR implementation
/****************************************************/

// Function 10: Verified
void SysTick_Handler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	refresh_is_alive();
#endif

	register uint32_t* sp asm("sp");
	(void) sp;
	register uint32_t* r0 asm("r0");
	(void) r0;
	task_list.global_tick++;

	activate_waiting_tasks();
	reload_systick();
	dispatcher(kFromISR);
}

// Function 11: Verified
void PendSV_Handler(void)
{
	// PENDSVCLR y PENDSVSET

	register int32_t r0 asm("r0");
	(void) r0;
	SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
	r0 = (int32_t) task_list.tasks[task_list.current_task].sp;
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
