
#ifndef MILLIS_H_
#define MILLIS_H_

typedef uint32_t millis_t;

#ifdef __cplusplus
extern "C" {
#endif

//Initialize timer 
void millis_init(void);

// Get millisecond count
millis_t millis(void);

// Turn on timer.
void millis_resume(void);

// Shutdown timer
void millis_pause(void);

// Clear timer count
void millis_reset(void);

// Increment time in ms
void millis_add(millis_t ms);

// Decrement time in ms
void millis_sub(millis_t ms);

#ifdef __cplusplus
}
#endif

#endif /* MILLIS_H_ */