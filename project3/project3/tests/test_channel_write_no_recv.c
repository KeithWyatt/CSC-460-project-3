#include "avr/io.h"
#define F_CPU 16000000
#include "util/delay.h"
#include "../os.h"

#define TASK1_PORT PA0

volatile CHAN chan_comm;

// This test ensures that Write() is an asynchronous operation
// No receivers are added to the channel, and the task will call Write() and not block
// Execution order: [T1, T1]

void init_debug_pins()
{
	DDRA |= (1<<TASK1_PORT);
	PORTA = 0;
}

// Receiving task
void Task_1()
{
	for (;;) {
		PORTA |= (1<<TASK1_PORT);
		Write( chan_comm, 1 );
		PORTA &= ~(1<<TASK1_PORT);
		_delay_ms(1);
	}
}

void a_main(void)
{
	init_debug_pins();
	chan_comm = Chan_Init();
	Task_Create_RR(Task_1, 0);
}