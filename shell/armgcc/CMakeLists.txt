# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 3.1.1)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})
SET(LIBRARY_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})


project(shell_bm)

set(MCUX_SDK_PROJECT_NAME shell_bm.elf)

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME} 
"${ProjDirPath}/../app_common.h"
"${ProjDirPath}/../blinky.c"
"${ProjDirPath}/../blinky.h"
"${ProjDirPath}/../circ_buffer.c" 
"${ProjDirPath}/../circ_buffer.h" 
"${ProjDirPath}/../event_dispatcher.c"
"${ProjDirPath}/../event_dispatcher.h"
"${ProjDirPath}/../event_list.h"
"${ProjDirPath}/../list.h"
"${ProjDirPath}/../main.c"
"${ProjDirPath}/../mainloop_timer.c"
"${ProjDirPath}/../mainloop_timer.h"
"${ProjDirPath}/../shell.c"
"${ProjDirPath}/../shell.h"
"${ProjDirPath}/../shell_if_uart.c"
"${ProjDirPath}/../shell_if_uart.h"
"${ProjDirPath}/../soft_timer.c"
"${ProjDirPath}/../soft_timer.h"
"${ProjDirPath}/../sys_tick.c"
"${ProjDirPath}/../sys_tick.h"
"${ProjDirPath}/../pin_mux.c"
"${ProjDirPath}/../pin_mux.h"
"${ProjDirPath}/../board.c"
"${ProjDirPath}/../board.h"
"${ProjDirPath}/../clock_config.c"
"${ProjDirPath}/../clock_config.h"
"${ProjDirPath}/../usb_dev/usb_device_descriptor.c"
"${ProjDirPath}/../usb_dev/usb_device_descriptor.h"
"${ProjDirPath}/../shell_if_usb.c"
"${ProjDirPath}/../usb_dev/virtual_com.h"
"${ProjDirPath}/../usb_dev/usb_device_config.h"
"${ProjDirPath}/../usb_dev/usb_device_ch9.c"
"${ProjDirPath}/../usb_dev/usb_device_ch9.h"
"${ProjDirPath}/../usb_dev/usb_device_class.c"
"${ProjDirPath}/../usb_dev/usb_device_class.h"
"${ProjDirPath}/../usb_dev/usb_device_cdc_acm.c"
"${ProjDirPath}/../usb_dev/usb_device_cdc_acm.h"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${ProjDirPath}/..
    ${ProjDirPath}/../usb_dev
)

set(CMAKE_MODULE_PATH
    ${ProjDirPath}/../../../frdm-k64f-sdk_2.9.0/middleware/usb
    ${ProjDirPath}/../../../frdm-k64f-sdk_2.9.0/middleware
    ${ProjDirPath}/../../../frdm-k64f-sdk_2.9.0/devices/MK64F12/drivers
    ${ProjDirPath}/../../../frdm-k64f-sdk_2.9.0/devices/MK64F12/utilities
    ${ProjDirPath}/../../../frdm-k64f-sdk_2.9.0/components/lists
    ${ProjDirPath}/../../../frdm-k64f-sdk_2.9.0/devices/MK64F12
    ${ProjDirPath}/../../../frdm-k64f-sdk_2.9.0/components/uart
    ${ProjDirPath}/../../../frdm-k64f-sdk_2.9.0/components/osa
    ${ProjDirPath}/../../../frdm-k64f-sdk_2.9.0/CMSIS/Include
)

# include modules
include(middleware_usb_device_khci_MK64F12)

include(middleware_baremetal_MK64F12)

include(middleware_usb_device_controller_driver_MK64F12)

include(middleware_usb_device_common_header_MK64F12)

include(middleware_usb_common_header_MK64F12)

include(component_osa_MK64F12)

include(driver_gpio_MK64F12)

include(component_lists_MK64F12)

include(driver_common_MK64F12)

include(driver_flash_MK64F12)

include(driver_clock_MK64F12)

include(driver_port_MK64F12)

include(driver_uart_MK64F12)

include(driver_smc_MK64F12)

include(device_MK64F12_CMSIS_MK64F12)

include(component_uart_adapter_MK64F12)

include(device_MK64F12_startup_MK64F12)

include(CMSIS_Include_core_cm4_MK64F12)

include(CMSIS_Include_common_MK64F12)

include(CMSIS_Include_dsp_MK64F12)

include(utilities_misc_utilities_MK64F12)

include(device_MK64F12_system_MK64F12)


TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE m)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE c)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE gcc)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE nosys)

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

add_custom_target(shell_bm.bin ALL DEPENDS ${CMAKE_BUILD_TYPE}/shell_bm.elf
  COMMAND ${CMAKE_OBJCOPY} -O binary -S ${CMAKE_BUILD_TYPE}/shell_bm.elf ${CMAKE_BUILD_TYPE}/shell_bm.bin
)

add_custom_target(shell_bm.hex ALL DEPENDS ${CMAKE_BUILD_TYPE}/shell_bm.elf
  COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_BUILD_TYPE}/shell_bm.elf ${CMAKE_BUILD_TYPE}/shell_bm.hex
)

set_property( DIRECTORY PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
  "${CMAKE_BUILD_TYPE}/shell_bm.bin"
  "${CMAKE_BUILD_TYPE}/shell_bm.hex"
)
