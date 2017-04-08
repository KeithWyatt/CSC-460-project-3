#include "avr/io.h"
#include "../os.h"

#define TASK1_PORT PA0
#define TASK2_PORT PA1
#define TASK3_PORT PA2
#define TASK4_PORT PA3

volatile CHAN chan_comm;

// This test ensures that Recv() is a synchronous operation for multiple receivers
// The tasks calling Recv() will block until the other task calls Send()
// Execution order: [T1, [T2,[T3, [T4, T4], T1], T2], T3]

void init_debug_pins()
{
	DDRA |= (1<<TASK1_PORT);
	DDRA |= (1<<TASK2_PORT);
	DDRA |= (1<<TASK3_PORT);
	DDRA |= (1<<TASK4_PORT);
	PORTA = 0;
}

// Receiving task
void Task_1()
{
	PORTA |= (1<<TASK1_PORT);
	Recv( chan_comm );
	PORTA &= ~(1<<TASK1_PORT);
}

// Receiving task
void Task_2()
{
	PORTA |= (1<<TASK2_PORT);
	Recv( chan_comm );
	PORTA &= ~(1<<TASK2_PORT);
}

// Receiving task
void Task_3()
{
	PORTA |= (1<<TASK3_PORT);
	Recv( chan_comm );
	PORTA &= ~(1<<TASK3_PORT);
}

// Sending task
void Task_4()
{
	PORTA |= (1<<TASK4_PORT);
	Send( chan_comm, 1 );
	PORTA &= ~(1<<TASK4_PORT);
}

void a_main(void)
{
	init_debug_pins();
	chan_comm = Chan_Init();
	Task_Create_RR(Task_1, 0);
	Task_Create_RR(Task_2, 0);
	Task_Create_RR(Task_3, 0);
	Task_Create_RR(Task_4, 0);
}