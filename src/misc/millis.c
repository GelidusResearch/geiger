
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include "../misc/millis.h"
#include "../config.h"

#if F_CPU > 16320000 // 16.32MHz - 65.28MHz
	#define CLOCKSEL (_BV(CS20))
	#define PRESCALER 256
#elif F_CPU > 2040000 // 2.04MHz - 16.32MHz
	#define CLOCKSEL (_BV(CS01)|_BV(CS00))
	#define PRESCALER 64
#elif F_CPU > 255 // 256Hz - 2.04MHz
	#define CLOCKSEL (_BV(CS01))
	#define PRESCALER 8
#endif

#define pwr_enable()	power_timer0_enable()
#define pwr_disable()	power_timer0_disable()
#define SET_TCCRA()	(TCCR0A = _BV(WGM01))
#define SET_TCCRB()	(TCCR0B = CLOCKSEL)


static volatile millis_t milliseconds;

// Initialize
void millis_init()
{
	// Timer settings
	SET_TCCRA();
	SET_TCCRB();
	TIMSK0 = _BV(OCIE0A);
	OCR0A = ((F_CPU / PRESCALER) / 1000);
}

// Get current milliseconds
millis_t millis()
{
	millis_t ms;
	{
		ms = milliseconds;
	}
	return ms;
}

// Turn on timer and resume time keeping
void millis_resume()
{
	pwr_enable();
	TIMSK0 |= _BV(OCIE0A);
}

// Pause time keeping and turn off timer to save power
void millis_pause()
{
	TIMSK0 &= ~_BV(OCIE0A);
	pwr_disable();
}

// Reset milliseconds count to 0
void millis_reset()
{
	{
		milliseconds = 0;
	}
}

// Add time
void millis_add(millis_t ms)
{
	{
		milliseconds += ms;
	}
}

// Subtract time
void millis_subtract(millis_t ms)
{

	{
		milliseconds -= ms;
	}
}

ISR(TIMER0_COMPA_vect)
{
	++milliseconds;
}