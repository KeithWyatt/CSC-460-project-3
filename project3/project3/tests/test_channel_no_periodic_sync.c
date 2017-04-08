#include "avr/io.h"
#include "../os.h"

#define TASK1_PORT PA0
#define TASK2_PORT PA2

volatile CHAN chan_comm;

// This test ensures that periodic tasks are not allowed to call synchronous operations
// Execution order: [T#, OS_Abort
// Error code: ERROR_PERIODIC_BLOCK_OP (3)

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
	Send( chan_comm, 1 );
	PORTA &= ~(1<<TASK2_PORT);
}

void a_main(void)
{
	init_debug_pins();
	chan_comm = Chan_Init();

	// Comment out Task_Create for T2 to test blocking Recv()
	//Task_Create_Period(Task_1, 0, 100, 80, 1);

	// Comment out Task_Create for T1 to test blocking Send()
	Task_Create_Period(Task_2, 00, 100, 80, 1);
}