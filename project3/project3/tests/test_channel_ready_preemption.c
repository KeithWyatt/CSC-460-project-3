#include "avr/io.h"
#include "../os.h"

#define TASK1_PORT PA0
#define TASK2_PORT PA1

volatile CHAN chan_comm;

// This test ensures that unblocking a higher priority task will cause it to preempt the current task
// The task calling Recv() will block until the other task calls Write() then resume execution in place of that task
// Execution order: [T1, [T2,T1], T2]

void init_debug_pins()
{
	DDRA |= (1<<TASK1_PORT);
	DDRA |= (1<<TASK2_PORT);
	PORTA = 0;
}

// Receiving task
void Task_1()
{
	PORTA |= (1<<TASK1_PORT);
	Recv( chan_comm );
	PORTA &= ~(1<<TASK1_PORT);
}

// Sending task
void Task_2()
{
	PORTA |= (1<<TASK2_PORT);
	Write( chan_comm, 1 );
	PORTA &= ~(1<<TASK2_PORT);
}

void a_main(void)
{
	init_debug_pins();
	chan_comm = Chan_Init();
	Task_Create_System(Task_1, 0);
	Task_Create_RR(Task_2, 0);
}