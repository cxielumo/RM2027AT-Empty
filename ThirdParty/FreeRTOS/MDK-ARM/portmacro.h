#ifndef RM2027AT_FREERTOS_PORTMACRO_H
#define RM2027AT_FREERTOS_PORTMACRO_H

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)
    #include "../Source/portable/GCC/ARM_CM4F/portmacro.h"
#elif defined(__CC_ARM)
    #include "../Source/portable/RVDS/ARM_CM4F/portmacro.h"
#else
    #error Unsupported compiler for the MDK FreeRTOS port
#endif

#endif /* RM2027AT_FREERTOS_PORTMACRO_H */
