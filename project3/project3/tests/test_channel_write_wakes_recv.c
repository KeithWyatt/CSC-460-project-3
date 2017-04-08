#include "avr/io.h"
#include "../os.h"

#define TASK1_PORT PA0
#define TASK2_PORT PA1

volatile CHAN chan_comm;

// This test ensures that Recv() is a synchronous operation for one receiver
// The task calling Recv() will block until the other task calls Write()
// Execution order: [T1, [T2,T2], T1]

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
	Task_Create_RR(Task_1, 0);
	Task_Create_RR(Task_2, 0);
}