#Select which microcontroller and board are being used
if (NOT TARGET_CONTROLLER)
  set(TARGET_CONTROLLER "STM32F103C8T6" CACHE STRING "Target MMC Controller")
endif()

if (NOT TARGET_BOARD_NAME)
  set(TARGET_BOARD_NAME "uFCP v2.0" CACHE STRING "Board Name")
endif()

#List all modules used by this board
set(TARGET_MODULES
  # "HOTSWAP_SENSOR"
  "BOARD_CONFIG"
  "UART_DEBUG"
  # "SDR"
  # "PAYLOAD"
  # "DAC_AD84XX"
  # "FRU"
  # "CLOCK_CONFIG"
  # "EEPROM_24XX02"
  # "EEPROM_AT24MAC"
  # "LM75"
  # "MAX6642"
  # "INA3221_VOLTAGE"
  # "INA3221_CURRENT"
  # "MCP23016"
  # "ADC"
  # "IDT_8V54816"
  # "HPM"
  # "SYSUTILS"
  # "RTM"
  )

if (NOT DISABLE_WATCHDOG)
  list(APPEND TARGET_MODULES "WATCHDOG")
endif()

set(BOARD_PATH ${CMAKE_CURRENT_SOURCE_DIR})

#Include the modules sources
##Sensors SDR list
# set( PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/sdr_list.c )
set( PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/i2c_mapping.c )
set( PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/pin_mapping.c )

if (";${TARGET_MODULES};" MATCHES ";PAYLOAD;")
  set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/payload.c)
  set(MODULES_FLAGS "${MODULES_FLAGS} -DMODULE_PAYLOAD")
endif()

if (";${TARGET_MODULES};" MATCHES ";RTM;")
  set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/rtm.c)
  set(MODULES_FLAGS "${MODULES_FLAGS} -DMODULE_RTM")
endif()

if (";${TARGET_MODULES};" MATCHES ";ADC;")
  set(MODULES_FLAGS "${MODULES_FLAGS} -DMODULE_ADC")
endif()

if (";${TARGET_MODULES};" MATCHES ";BOARD_CONFIG;")
  set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/board_config.c)
  set(MODULES_FLAGS "${MODULES_FLAGS} -DMODULE_BOARD_CONFIG")
endif()

#Extra definitions
if (FRU_WRITE_EEPROM)
  message(STATUS "FRU EEPROM will be written if no valid data is found!")
  set(MODULES_FLAGS "${MODULES_FLAGS} -DFRU_WRITE_EEPROM")
  set(FRU_WRITE_EEPROM false CACHE BOOL "Set this flag to enable writing the board ID on the EEPROM" FORCE)
endif()

#Set the variables in the main scope
set(TARGET_MODULES ${TARGET_MODULES} PARENT_SCOPE)
set(MODULES_FLAGS ${MODULES_FLAGS} PARENT_SCOPE)
set(PROJ_HDRS ${PROJ_HDRS} ${BOARD_PATH})
set(PROJ_SRCS ${PROJ_SRCS} PARENT_SCOPE)
set(PROJ_HDRS ${PROJ_HDRS} PARENT_SCOPE)
