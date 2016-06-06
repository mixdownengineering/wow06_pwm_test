#ifndef _CRITICAL_H_
#define _CRITICAL_H_

/* from https://mcuoneclipse.com/2014/01/26/entercritical-and-exitcritical-why-things-are-failing-badly */

#define CpuCriticalVar()			unsigned char cpuSR

#define CpuEnterCritical()			\
do {						\
	__asm volatile (			\
	"MRS R0, PRIMASK\n\t"			\
	"CPSID I\n\t"				\
	"STRB R0, %[output]"			\
	: [output] "=m" (cpuSR) :: "r0");	\
} while(0)

#define CpuExitCritical()			\
do {						\
	__asm volatile (			\
	"ldrb r0, %[input]\n\t"			\
	"msr PRIMASK,r0;\n\t"			\
	::[input] "m" (cpuSR) : "r0");		\
} while(0)

#if 0
/* original, buggy version */
#define DEFINE_CRITICAL()	/* nothing */
#define ENTER_CRITICAL()	EnterCritical()
#define EXIT_CRITICAL()		ExitCritical()

#else
/* fixed version using local variable */
#define DEFINE_CRITICAL()	CpuCriticalVar()
#define ENTER_CRITICAL()	CpuEnterCritical()
#define EXIT_CRITICAL()		CpuExitCritical()
#endif

#endif /* ifndef _CRITIAL_H_ */
