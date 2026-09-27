#ifndef RM2027AT_MDK_FREERTOS_CONFIG_H
#define RM2027AT_MDK_FREERTOS_CONFIG_H

#include "../Config/FreeRTOSConfig.h"

/* ARM Compiler 5 requires this priority as a literal in the RVDS port's
 * inline assembler. Keep the general expression for GCC and ARM Compiler 6. */
#if defined(__CC_ARM) && !defined(__clang__)
    #undef configMAX_SYSCALL_INTERRUPT_PRIORITY
    #define configMAX_SYSCALL_INTERRUPT_PRIORITY 0x50U
#endif

#endif /* RM2027AT_MDK_FREERTOS_CONFIG_H */
