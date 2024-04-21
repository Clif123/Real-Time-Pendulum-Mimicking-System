//Name: Clifton Mak Ren Ming
//ID: 29439701

#include <asf.h>
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include <string.h>

// PID include
#include "FastPID.h"

// FreeRTOS includes. 
#include "FreeRTOS.h"
#include "task.h"

// Photoelectric Sensor Interrupt
#define MY_INTERRUPT_PEND	PIO_PC24_IDX // ARDUINO DUE DIGITAL PIN 6

//Pin allocation variables
#define MOTOR_DIR_PIN PIO_PC26_IDX   //Digital pin 4

// Motor and encoder constants
#define MOTOR_RATED_RPM 430
#define COUNT_PER_REVOLUTION (98*2) // Encoder is specified as 98 pulses per revolution. Hardware QDEC seems to accumulate both A and B pulses

////////////////////////////////////////////////////////////  PWM  //////////////////////////////////////////////////
/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

//PID parameter settings
//float Kp=0.1, Ki=0.5, Kd=0, Hz=10;
float Kp=2.1, Ki=0.001, Kd=30, Hz=10;
//float Kp=2.1, Ki=0.001, Kd=50, Hz=10;

//PID variables
int output_bits = 16;
bool output_signed = true;


/** PWM channel instance for motor */
pwm_channel_t pwm_opts;

////////////////////////////////////////////////////////////  Tasks  //////////////////////////////////////////////////
/* The tasks to be created. */
static void vHandlerTask( void *pvParameters );
static void lookupTask( void *pvParameters );
void photoelectric_isr(const uint32_t id, const uint32_t index);

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
xSemaphoreHandle xBinarySemaphore;

/*-----------------------------------------------------------*/
/**
 *  Configure UART console.
 */
// [main_console_configure]
static void configure_console(void)
{
	const usart_serial_options_t printf = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONSOLE_UART, &printf);
}

//Global Variables
int32_t direction = 0;
int32_t g_SetpointPos = 0;
int32_t g_SetpointSpeed = 0;
int32_t lookup_time = 0;
uint32_t flag_move = 0;
uint32_t flag_start = 0;
static uint32_t exit_time2 = 0;
static uint32_t quarter_period = 0;
static uint32_t state = 0;

int main( void )
{
   /* This function initializes the MCU clock  */
	sysclk_init();
	/* Board initialization */
	board_init();
	
	//Initialize the Fast PID controller based on the given parameters.
	InitializeFastPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
	
	ioport_set_pin_dir(MY_INTERRUPT_PEND, IOPORT_DIR_INPUT);  //set dig pin 6 as input pin
	ioport_set_pin_mode(MY_INTERRUPT_PEND, IOPORT_MODE_PULLUP);  //set dig pin 6 as a pullup
	
	//Direction pin
	ioport_set_pin_dir(MOTOR_DIR_PIN, IOPORT_DIR_OUTPUT);
	
	/* Initialize the serial I/O(console ) */
	configure_console();
	
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM);

	/* Disable PWM channels for motor */
	pwm_channel_disable(PWM, PWM_CHANNEL_6);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};

	//PWM options
	pwm_opts.ul_prescaler = PWM_CMR_CPRE_CLKA;  //set to clock A
	pwm_opts.ul_period = PERIOD_VALUE;
	pwm_opts.ul_duty = INIT_DUTY_VALUE;
	pwm_opts.channel = PWM_CHANNEL_6;   //Digital Pin 7 (PC23)   //refer to datasheet in Table 9.4, page 43
	
	//Initialize PWM
	pwm_init(PWM, &clock_setting);
	
	// Initialize the PWM Channel
	pwm_channel_init(PWM, &pwm_opts);
	
	// Enable the PWM channel
	pwm_channel_enable(PWM, PWM_CHANNEL_6);
	
	/* Before a semaphore is used it must be explicitly created.  In this example
    a binary semaphore is created. */
    vSemaphoreCreateBinary( xBinarySemaphore );
	
	 if(xBinarySemaphore != NULL)
	 {
		// Configure Timer Counter for quadrature decoding
		// Uses TIOA0 and TIOB0 which are on pins 2 and 13
		pmc_enable_periph_clk(ID_TC0);  //
		tc_init(TC0, 0, TC_CMR_ETRGEDG_RISING | TC_CMR_ABETRG | TC_CMR_TCCLKS_XC0);  //rising edge,TIOA or B trigger selection,XC0 clock
		tc_set_block_mode(TC0, TC_BMR_QDEN | TC_BMR_POSEN | TC_BMR_MAXFILT(63));  //Quad decoder enabled,position enabled,maximum filter

		////////////////////////////////////////////////// starting and enabling modules
		//Start timer counter
		tc_start(TC0, 0);
		
		pmc_enable_periph_clk(ID_PIOC); // Sets the peripheral clock
		pio_set_input(PIOC, PIO_PC24, PIO_PULLUP); // maps the SAM3X8E PC24 pin to PIOC port
		pio_handler_set(PIOC, ID_PIOC, PIO_PC24, PIO_IT_EDGE, photoelectric_isr); // Assigns an Interrupt Service Routine (ISR)  //triggered by rising & falling edges
		pio_enable_interrupt(PIOC, PIO_PC24); // Enable the interrupt
		
		/* The interrupt service routine uses an (interrupt safe) FreeRTOS API
		function so the interrupt priority must be at or below the priority defined
		by configSYSCALL_INTERRUPT_PRIORITY. */
		NVIC_SetPriority(PIOC_IRQn, 11);  //priority 11 and shift left logical by 4 - will get (11x2^4 = 176 = oxb0) //config_max_syscall_prio

		/* Enable the interrupt. */
		NVIC_EnableIRQ(PIOC_IRQn);
        
		/* Create the 'handler' task.  This is the task that will be synchronized
		with the interrupt.  The handler task is created with a high priority to
		ensure it runs immediately after the interrupt exits.  In this case a
		priority of 3 is chosen. */
		xTaskCreate( vHandlerTask, "Handler", 512, NULL, 2, NULL );

	   xTaskCreate( lookupTask, "lookup", 512, NULL, 3, NULL );

		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	 }
    
    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void lookupTask( void *pvParameters )
{
	portBASE_TYPE xStatus;
	portTickType numTicks = 0;
	TickType_t last_wake_time = xTaskGetTickCount();
	int32_t position_list [40] = {45,42,38,26,26,26,26,26,23,23,23,23,23,21,21,14,14,14,14,11,11,11,11,11,11,8,8,8,8,8,8,8,7,7,7,7,7,7,7};
	int32_t time_list [38] = {5,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,32,33,34,35,36,39,40,42,44,47,48,52,54};
	int32_t speed_list [38] = {18,18,18,18,17,17,17,15,15,15,11,11,11,11,11,10,10,10,10,9,9,9,9,9,8,8,8,8,7,7,7,7,7,7,7,7,7,7};
		
    /* As per most tasks, this task is implemented within an infinite loop.

    Take the semaphore once to start with so the semaphore is empty before the
    infinite loop is entered.  The semaphore was created before the scheduler
    was started so before this task ran for the first time.*/
    xSemaphoreTake( xBinarySemaphore, 0 );

    for( ;; )
    {
        /* Use the semaphore to wait for the event.  The task blocks
        indefinitely meaning this function call will only return once the
        semaphore has been successfully obtained - so there is no need to check
        the returned value. */
        xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );
		
		//g_SetpointPos = 45 * direction;
		
		//printf("lookup:%d\n",lookup_time);
		//printf("task");
		
		if(quarter_period > 800)   //when the pendulum is about to halt  OR //the initial stage where first quarter period is being collected
		{
			g_SetpointPos = 0;
			exit_time2 = 0;
		}
		
		else
		{
			uint32_t i = 0;
			
			while (i < 38)
			{
				if (lookup_time < time_list[i])
				{
					g_SetpointPos = direction * position_list[i];
					g_SetpointSpeed = speed_list[i];
					break;
				}
				i++;
			}
		}
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

static void vHandlerTask( void *pvParameters )
{
	portBASE_TYPE xStatus;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	portTickType numTicks = 0;
	TickType_t last_wake_time = xTaskGetTickCount();
	int32_t motor_pos;
	
	static uint32_t prev_quarter = 0;
	static uint32_t counter = 0;
	int16_t pid_output = 0;
    
	// As per most tasks, this task is implemented within an infinite loop.
    for( ;; )
    {
		motor_pos = tc_read_cv(TC0, 0);
		motor_pos = -(motor_pos % (COUNT_PER_REVOLUTION + 1));  //maintain the count values within one rev range  //multiplied by "-" to make clockwise counts "+"
		//printf("Motor pos: %d\n", motor_pos);
		
		//printf("pid_output:%d\n",pid_output);
		//printf("g_setpoint:%d\n",g_SetpointPos);
		//printf("direction:%d\n",direction);
		//printf("flag_move:%d\n",flag_move);
		//quarter_compare_current = xTaskGetTickCount();

		
		if((xTaskGetTickCount() - exit_time2) >= quarter_period)
		{
			flag_move = 1;
			g_SetpointPos = 0;
			//exit_time2 = 0;
			//printf("Entered\n");
		}
		
		pid_output = step(g_SetpointPos,motor_pos);
		//printf("current:%d   exit_time:%d  diff:%d  quarter:%d  pid:%d\n", xTaskGetTickCount(),exit_time2,xTaskGetTickCount()-exit_time2,quarter_period,pid_output);
		if (flag_move == 1)
		{
			
			if(pid_output >-11 && pid_output <11)
			{
				pwm_channel_update_duty(PWM, &pwm_opts,0);
				//printf("Case 1\n");
				flag_move = 0;
				
			}
			
			if (pid_output >= 11)
			{
				ioport_set_pin_level(MOTOR_DIR_PIN, 1);   //1 is clockwise
				pwm_channel_update_duty(PWM, &pwm_opts,g_SetpointSpeed);
				//printf("Case 2\n");
				
			}
			else if(pid_output <= -11)
			{
				ioport_set_pin_level(MOTOR_DIR_PIN, 0);   //0 is anti-clockwise
				pwm_channel_update_duty(PWM, &pwm_opts,g_SetpointSpeed);
				//printf("Case 3\n");
			}
		}
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

void photoelectric_isr(const uint32_t id, const uint32_t index)  //ISR
{
	uint32_t pinVal = 0;  //sensor reading
	unsigned int priority = 0;
	
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	portBASE_TYPE xStatus;
	
	static uint32_t enter_time1 = 0;
	static uint32_t enter_time2 = 0;
	static uint32_t enter_duration = 0;
	static uint32_t exit_time1 = 0;
	//static uint32_t exit_time2 = 0;
	static uint32_t exit_duration = 0;
	static uint32_t start_quarter = 0;
	
	pinVal = ioport_get_pin_level(MY_INTERRUPT_PEND); //get reading from sensor (dig pin 6) 1 if detect smtg (NC pin)

	if ((id != ID_PIOC) || (index != PIO_PC24))  //if interrupt is not coming from digital pin 6
	{
		return;
	}
	
	if (pinVal == 1 && state == 0)  //when pendulum first enters stage A
	{

		enter_time1 = xTaskGetTickCountFromISR();
		
	
		quarter_period = (enter_time1 - start_quarter)/2;
		//printf("quarter:%d\n",quarter_period);
		start_quarter = enter_time1;
	
		//printf("enter time1: %d\n",enter_time1);
		state = 1;
		//printf("State 1\n");
	}
	else if (pinVal == 0 && state == 1)  //when pendulum exits stage A
	{
		enter_time2 = xTaskGetTickCountFromISR();
		//printf("enter time2: %d\n",enter_time2);
		state = 2;
		enter_duration = enter_time2 - enter_time1;  //Duration for A
		//printf("State 2\n");

	}
	
	if (pinVal == 1 && state == 2)  //when pendulum enters stage B
	{
		exit_time1 = xTaskGetTickCountFromISR();
		state = 3;
		//printf("State 3\n");
	} 
	
	else if (pinVal == 0 && state == 3)  //when pendulum exits stage B
	{
		exit_time2 = xTaskGetTickCountFromISR();
		
		exit_duration = exit_time2 - exit_time1;  //Remember count when entering (A)
		state = 0;
		//printf("State 4\n");
		
		//printf("enter: %d  exit: %d\n",enter_duration,exit_duration);
		
		if(enter_duration < exit_duration)
		{
			//printf("Right\n");  //Coming from right
			direction = -1;  //anti-clockwise  
			flag_move = 1;
			//g_SetpointPos = -30;
			lookup_time = enter_duration;	
			
		}
		else if(enter_duration > exit_duration)
		{
			//printf("Left\n");  //Coming from left
			direction = 1; //clockwise
			flag_move = 1;
			//g_SetpointPos = 30;
			lookup_time = exit_duration;
		}
		
					
		xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken );
	}
	 /* Giving the semaphore may have unblocked a task - if it did and the
		unblocked task has a priority equal to or above the currently executing
		task then xHigherPriorityTaskWoken will have been set to pdTRUE and
		portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
		higher priority task.

		NOTE: The syntax for forcing a context switch within an ISR varies between
		FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
		the Cortex M3 port layer for this purpose.  taskYIELD() must never be called
		from an ISR! */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	
}

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
}






