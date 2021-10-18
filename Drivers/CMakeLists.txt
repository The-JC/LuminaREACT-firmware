add_library(stm32f4xx "")

add_subdirectory(STM32F4xx_HAL_Driver)
add_subdirectory(CMSIS)

target_include_directories(stm32f4xx PUBLIC ${PROJECT_SOURCE_DIR}/Core/Inc)

# # set up some defines for the StdPeriph library
# target_compile_definitions(stm32f4xx PUBLIC USE_STDPERIPH_DRIVER)
target_compile_definitions(stm32f4xx PUBLIC ${DEVICE_FAMILY})

target_compile_options(stm32f4xx PRIVATE ${additional_flags})
