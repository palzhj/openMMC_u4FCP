set(BOOTLOADER_INC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Inc)

set(BOOTLOADER_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_msp.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_it.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/syscalls.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/sysmem.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/system_stm32f1xx.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Startup/startup_stm32f103c8tx.s
  )

# set(BOOTLOADER_SRCS ${BOOTLOADER_SRCS} PARENT_SCOPE)
# set(BOOTLOADER_INC_PATH ${BOOTLOADER_INC_PATH} PARENT_SCOPE)

include_directories(${STM32F1XX_HAL_INCPATH})
include_directories(${STM32F1XX_LEGACY_INCPATH})
include_directories(${CMSIS_INCPATH})
include_directories(${CMSIS_ST_INCPATH})
include_directories(${BOOTLOADER_INC_PATH})

set(COMMON_FLAGS "")
set(CMAKE_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_EXE_LINKER_FLAGS "")
set(CMAKE_C_FLAGS_DEBUG "-Og -g3 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-Os")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Os -g3")

set(CMAKE_ASM_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})
set(CMAKE_ASM_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})
set(CMAKE_ASM_FLAGS_RELWITHDEBINFO ${CMAKE_C_FLAGS_RELWITHDEBINFO})

set(STM32_BOOT_FLAGS "${STM32_FLAGS}")
set(STM32_BOOT_LINK_FLAGS "${STM32_BOOT_FLAGS} -Wl,--gc-sections -T \"${CMAKE_CURRENT_SOURCE_DIR}/../linker/${TARGET_CONTROLLER}_boot.ld\"")
message(STATUS "STM32_BOOT_FLAGS: ${STM32_BOOT_FLAGS}")
message(STATUS "STM32_BOOT_LINK_FLAGS: ${STM32_BOOT_LINK_FLAGS}")
# message(STATUS "STM32_FLAGS: ${STM32_FLAGS}")

add_executable(newboot ${BOOTLOADER_SRCS})
target_link_libraries(newboot STM32F1XX)
set_target_properties(newboot PROPERTIES
  COMPILE_FLAGS ${STM32_BOOT_FLAGS}
  SUFFIX ".elf"
  LINK_FLAGS ${STM32_BOOT_LINK_FLAGS}
  )
add_custom_command(TARGET newboot POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary newboot.elf newboot.bin
  WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
  COMMENT "Generate a raw binary image of the bootloader"
  )

##Generate hpm files if bin2hpm is installed
find_program(BIN2HPM NAMES "bin2hpm")
if(BIN2HPM)
  add_custom_command(TARGET newboot POST_BUILD
    COMMAND bin2hpm -c 0 -n -m 0x315A -p 0x00 newboot.bin -o newboot.hpm
    WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    COMMENT "Creating HPM file from binary"
  )
endif()