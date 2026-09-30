cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# AT32 WorkBench generated symbols (macros)
set(WK_Defines_Syms
    USE_STDPERIPH_DRIVER
    AT32F423CCT7
    $<$<CONFIG:Debug>:DEBUG>
)

# AT32 WorkBench generated include paths
set(WK_Include_Dirs
    ${CMAKE_SOURCE_DIR}/project/inc
    ${CMAKE_SOURCE_DIR}/Algorithm/Inc
    ${CMAKE_SOURCE_DIR}/Bsp/Inc
    ${CMAKE_SOURCE_DIR}/Components/Inc
    ${CMAKE_SOURCE_DIR}/Application/Inc
    ${CMAKE_SOURCE_DIR}/libraries/drivers/inc
    ${CMAKE_SOURCE_DIR}/libraries/cmsis/cm4/core_support
    ${CMAKE_SOURCE_DIR}/libraries/cmsis/cm4/device_support
    ${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Config
)

# AT32 WorkBench generated application sources
set(WK_Application_Src
    ${CMAKE_SOURCE_DIR}/project/src/main.c
    ${CMAKE_SOURCE_DIR}/project/src/syscalls.c
    ${CMAKE_SOURCE_DIR}/project/src/sysmem.c
    ${CMAKE_SOURCE_DIR}/project/src/at32f423_wk_config.c
    ${CMAKE_SOURCE_DIR}/project/src/at32f423_int.c
    ${CMAKE_SOURCE_DIR}/project/src/wk_system.c
    ${CMAKE_SOURCE_DIR}/project/src/wk_adc.c
    ${CMAKE_SOURCE_DIR}/project/src/wk_can.c
    ${CMAKE_SOURCE_DIR}/project/src/wk_tmr.c
    ${CMAKE_SOURCE_DIR}/project/src/wk_usart.c
    ${CMAKE_SOURCE_DIR}/project/src/wk_gpio.c
    ${CMAKE_SOURCE_DIR}/project/src/wk_dma.c
    ${CMAKE_SOURCE_DIR}/startup_at32f423.s
)

# User framework sources are maintained outside generated WorkBench files.
# Keep this list explicit so retired modules are not compiled accidentally.
set(RM2027AT_Framework_Src
    ${CMAKE_SOURCE_DIR}/Algorithm/Src/pid.c
    ${CMAKE_SOURCE_DIR}/Algorithm/Src/ramp.c
    ${CMAKE_SOURCE_DIR}/Algorithm/Src/ringBuffer.c
    ${CMAKE_SOURCE_DIR}/Bsp/Src/bsp.c
    ${CMAKE_SOURCE_DIR}/Bsp/Src/adc.c
    ${CMAKE_SOURCE_DIR}/Bsp/Src/can.c
    ${CMAKE_SOURCE_DIR}/Bsp/Src/led.c
    ${CMAKE_SOURCE_DIR}/Bsp/Src/os.c
    ${CMAKE_SOURCE_DIR}/Bsp/Src/pwm.c
    ${CMAKE_SOURCE_DIR}/Bsp/Src/uart.c
    ${CMAKE_SOURCE_DIR}/Components/Src/daemon.c
    ${CMAKE_SOURCE_DIR}/Components/Src/dbus.c
    ${CMAKE_SOURCE_DIR}/Components/Src/motor_tt.c
    ${CMAKE_SOURCE_DIR}/Components/Src/motor_dji.c
    ${CMAKE_SOURCE_DIR}/Components/Src/motor.c
    ${CMAKE_SOURCE_DIR}/Components/Src/servo.c
    ${CMAKE_SOURCE_DIR}/Application/Src/robot_main.c
)

# FreeRTOS Kernel V11.3.1 is shared with the C-board framework in this
# workspace. Override FREERTOS_ROOT when building from another checkout.
set(FREERTOS_ROOT "${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS" CACHE PATH "FreeRTOS root")
if(NOT EXISTS "${FREERTOS_ROOT}/Source/tasks.c")
    message(FATAL_ERROR "FreeRTOS Kernel not found at ${FREERTOS_ROOT}")
endif()
set(FREERTOS_KERNEL_DIR "${FREERTOS_ROOT}/Source")
set(FREERTOS_Src
    ${FREERTOS_KERNEL_DIR}/croutine.c
    ${FREERTOS_KERNEL_DIR}/event_groups.c
    ${FREERTOS_KERNEL_DIR}/list.c
    ${FREERTOS_KERNEL_DIR}/queue.c
    ${FREERTOS_KERNEL_DIR}/stream_buffer.c
    ${FREERTOS_KERNEL_DIR}/tasks.c
    ${FREERTOS_KERNEL_DIR}/timers.c
    ${FREERTOS_KERNEL_DIR}/portable/MemMang/heap_4.c
       ${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM4F/port.c
)

# AT32 Drivers
set(AT32_Drivers_Src
    ${CMAKE_SOURCE_DIR}/libraries/cmsis/cm4/device_support/system_at32f423.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_crm.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_tmr.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_gpio.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_usart.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_pwc.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_adc.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_dma.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_debug.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_flash.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_exint.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_misc.c
    ${CMAKE_SOURCE_DIR}/libraries/drivers/src/at32f423_can.c
)

# Drivers Middlewares

# Link directories setup
set(WK_LINK_DIRS

)

# Project static libraries
set(WK_LINK_LIBS
    AT32_Drivers
)

# Interface library for includes and symbols
add_library(at32_workbench INTERFACE)
target_include_directories(at32_workbench INTERFACE ${WK_Include_Dirs})
target_compile_definitions(at32_workbench INTERFACE ${WK_Defines_Syms})

# Create AT32_Drivers static library
add_library(AT32_Drivers OBJECT)
target_sources(AT32_Drivers PRIVATE ${AT32_Drivers_Src})
target_link_libraries(AT32_Drivers PUBLIC at32_workbench)

# Add AT32 WorkBench generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${WK_Application_Src} ${RM2027AT_Framework_Src})
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${FREERTOS_Src})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${FREERTOS_KERNEL_DIR}/include
    ${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM4F
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${WK_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${WK_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

