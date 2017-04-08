#include "avr/io.h"
#include "../os.h"

#define TASK1_PORT PA0
#define TASK2_PORT PA1

volatile CHAN chan_comm;

// This test ensures that two senders cannot send on the same channel
// Execution order: T1, T2, OS_Abort()

void init_debug_pins()
{
	DDRA |= (1<<TASK1_PORT);
	DDRA |= (1<<TASK2_PORT);
	PORTA = 0;
}

void Task_1()
{
	PORTA |= (1<<TASK1_PORT);
	Send( chan_comm, 1 );
	PORTA &= ~(1<<TASK1_PORT);
}

void Task_2()
{
	PORTA |= (1<<TASK2_PORT);
	Send( chan_comm, 1 );
	PORTA &= ~(1<<TASK2_PORT);
}

void a_main(void)
{
	init_debug_pins();
	chan_comm = Chan_Init();
	Task_Create_RR(Task_1, 0);
	Task_Create_RR(Task_2, 0);
}