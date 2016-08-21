#if !defined(_OPTIMIZATION_NONE) && !defined(_OPTIMIZATION_1) && !defined(_OPTIMIZATION_2) && !defined(_OPTIMIZATION_3)
	#error DEFINE OPTIMIZATION LEVEL
#else
	#ifdef _OPTIMIZATION_NONE
		#define K_Const			13300
	#endif

	#ifdef _OPTIMIZATION_1
		#define K_Const			3000
	#endif

	#ifdef _OPTIMIZATION_2
		#error DELAY DO NOT WORK
	#endif

	#ifdef _OPTIMIZATION_3
		#error DELAY DO NOT WORK
	#endif
#endif

#define CPU_CLOCK_DELAY	F_CPU


void delay_ms(unsigned long nTime);
void delay_us(unsigned long nTime);
