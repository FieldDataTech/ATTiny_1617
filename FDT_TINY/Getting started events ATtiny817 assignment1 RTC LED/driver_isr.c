#define NOP()			__asm__ __volatile__ ("nop")
#include <driver_init.h>
#include <compiler.h>
#include "tinyInclude.h"
#include "initTinyBoard.h"
#include <avr/pgmspace.h>
#define DISABLE_INTERRUPTS()        __asm__ __volatile__ ( "cli" ::: "memory")
#define ENABLE_INTERRUPTS()         __asm__ __volatile__ ( "sei" ::: "memory")

