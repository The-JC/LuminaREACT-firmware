add_library(freertos
	FreeRTOS/Source/croutine.c
	FreeRTOS/Source/event_groups.c
	FreeRTOS/Source/list.c
	FreeRTOS/Source/queue.c
	FreeRTOS/Source/stream_buffer.c
	FreeRTOS/Source/tasks.c
	FreeRTOS/Source/timers.c
	FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
	FreeRTOS/Source/portable/MemMang/heap_4.c
	FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
)

target_include_directories(freertos PUBLIC FreeRTOS/Source/include)
target_include_directories(freertos PUBLIC FreeRTOS/Source/CMSIS_RTOS_V2)
target_include_directories(freertos PUBLIC FreeRTOS/Source/portable/GCC/ARM_CM4F)
# Include FreeRTOSConfig.h
target_include_directories(freertos PUBLIC ${PROJECT_SOURCE_DIR}/Core/Inc)
target_include_directories(freertos PUBLIC ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/Include)
