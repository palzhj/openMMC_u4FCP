set(STM32F1XX_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(STM32F1XX_HAL_SRCPATH ${STM32F1XX_PATH}/STM32F1xx_HAL_Driver/Src)
set(STM32F1XX_HAL_INCPATH ${STM32F1XX_PATH}/STM32F1xx_HAL_Driver/Inc)
set(STM32F1XX_LEGACY_INCPATH ${STM32F1XX_PATH}/STM32F1xx_HAL_Driver/Inc/Legacy)
set(CMSIS_INCPATH ${STM32F1XX_PATH}/CMSIS/Include)
set(CMSIS_ST_INCPATH ${STM32F1XX_PATH}/CMSIS/Device/ST/STM32F1xx/Include)

#Silence GCC Warnings
# add_compile_options( -Wno-switch -Wno-unused-but-set-variable )
add_compile_options( -Wall )

set(UCONTROLLER_SRCS
  # ${STM32F1XX_PATH}/cr_startup_lpc175x_6x.c
  # ${STM32F1XX_PATH}/sysinit.c
  # ${STM32F1XX_PATH}/faults.c
  )

set(UCONTROLLER_HDRS
  ${STM32F1XX_PATH}
  ${STM32F1XX_HAL_INCPATH}
  ${STM32F1XX_LEGACY_INCPATH}
  ${CMSIS_INCPATH}
  ${CMSIS_ST_INCPATH}
  )

set(LIBSTM32F1XX_SRCS
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_cortex.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_cortex.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_dma.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_exti.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_flash.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_flash_ex.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_gpio.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_gpio_ex.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_i2c.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_pwr.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_rcc.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_rcc_ex.c
  ${STM32F1XX_HAL_SRCPATH}/stm32f1xx_hal_uart.c

  # ${STM32F1XX_PATH}/lpc17_i2c.c
  # ${STM32F1XX_PATH}/lpc17_spi.c
  # ${STM32F1XX_PATH}/lpc17_ssp.c
  # ${STM32F1XX_PATH}/lpc17_hpm.c
  # ${STM32F1XX_PATH}/lpc17_pincfg.c
  )

if( UART_RINGBUFFER )
  set(LIBSTM32F1XX_SRCS 
    ${LIBSTM32F1XX_SRCS} 
    # ${STM32F1XX_HAL_SRCPATH}/ring_buffer.c 
    # ${STM32F1XX_PATH}/lpc17_uartrb.c
  )
  set(MODULES_FLAGS ${MODULES_FLAGS} -DUART_RINGBUFFER PARENT_SCOPE)
else()
  set(LIBSTM32F1XX_SRCS 
    ${LIBSTM32F1XX_SRCS} 
    # ${STM32F1XX_PATH}/lpc17_uart.c
  )
endif()

# set(TARGET_ARCH     "armv7-m" )
set(TARGET_CPU      "cortex-m3")

# set(STM32_FLAGS ${STM32_FLAGS} -march=${TARGET_ARCH})
set(STM32_FLAGS ${STM32_FLAGS} -mcpu=${TARGET_CPU})
set(STM32_FLAGS ${STM32_FLAGS} -DSTM32F103xB)
set(STM32_FLAGS ${STM32_FLAGS} -DUSE_HAL_DRIVER)
set(STM32_FLAGS ${STM32_FLAGS} -mthumb -mthumb-interwork -mfloat-abi=soft)
set(STM32_FLAGS ${STM32_FLAGS} -ffunction-sections -fdata-sections)

string(REGEX REPLACE ";" " " STM32_FLAGS "${STM32_FLAGS}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${STM32_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" PARENT_SCOPE)

set(UCONTROLLER_SRCS ${UCONTROLLER_SRCS} PARENT_SCOPE)
set(UCONTROLLER_HDRS ${UCONTROLLER_HDRS} PARENT_SCOPE)
set(UCONTROLLER_APP_LD_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/linker/${TARGET_CONTROLLER}_app.ld" PARENT_SCOPE)

add_library(STM32F1XX STATIC ${LIBSTM32F1XX_SRCS})

# target_link_libraries(STM32F1XX PUBLIC FreeRTOS)
target_link_libraries(STM32F1XX PUBLIC)
target_include_directories(STM32F1XX PUBLIC ${STM32F1XX_HAL_INCPATH})
target_include_directories(STM32F1XX PUBLIC ${STM32F1XX_LEGACY_INCPATH})
target_include_directories(STM32F1XX PUBLIC ${CMSIS_INCPATH})
target_include_directories(STM32F1XX PUBLIC ${CMSIS_ST_INCPATH})
target_include_directories(STM32F1XX PUBLIC ${STM32F1XX_PATH})
# target_include_directories(STM32F1XX PUBLIC ${PROJ_HDRS})

add_subdirectory(bootloader)
