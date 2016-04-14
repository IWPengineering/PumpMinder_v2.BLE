#This file is generated by VisualGDB.
#It contains GCC settings automatically derived from the board support package (BSP).
#DO NOT EDIT MANUALLY. THE FILE WILL BE OVERWRITTEN. 
#Use VisualGDB Project Properties dialog or modify Makefile or per-configuration .mak files instead.

#VisualGDB provides BSP_ROOT and TOOLCHAIN_ROOT via environment when running Make. The line below will only be active if GNU Make is started manually.
BSP_ROOT ?= BSP
EFP_BASE ?= $(LOCALAPPDATA)/VisualGDB/EmbeddedEFPs
TOOLCHAIN_ROOT ?= C:/SysGCC/arm-eabi

#Embedded toolchain
CC := $(TOOLCHAIN_ROOT)/bin/arm-eabi-gcc.exe
CXX := $(TOOLCHAIN_ROOT)/bin/arm-eabi-g++.exe
LD := $(CXX)
AR := $(TOOLCHAIN_ROOT)/bin/arm-eabi-ar.exe
OBJCOPY := $(TOOLCHAIN_ROOT)/bin/arm-eabi-objcopy.exe

#Additional flags
#PREPROCESSOR_MACROS += ARM_MATH_CM4 NRF52 S132 nRF52832_XXAA BOARD_PCA10040 BSP_SIMPLE  BLE_STACK_SUPPORT_REQD
PREPROCESSOR_MACROS += ARM_MATH_CM4 NRF52 S132 nRF52832_XXAA BOARD_PCA10040 BSP_SIMPLE BLE_STACK_SUPPORT_REQD SOFTDEVICE_PRESENT SWI_DISABLE0 SOFTDEVICE_PRESENT SWI_DISABLE0
INCLUDE_DIRS += . $(BSP_ROOT)/nRF52/components/softdevice/S132/headers $(BSP_ROOT)/nRF52/components/softdevice/S132/headers/nrf52 $(BSP_ROOT)/nRF52/components/toolchain $(BSP_ROOT)/nRF52/components/toolchain/gcc $(BSP_ROOT)/nRF52/components/device $(BSP_ROOT)/nRF52/components/softdevice/common/softdevice_handler $(BSP_ROOT)/nRF52/components/drivers_nrf/ble_flash $(BSP_ROOT)/nRF52/components/drivers_nrf/clock $(BSP_ROOT)/nRF52/components/drivers_nrf/common $(BSP_ROOT)/nRF52/components/drivers_nrf/config $(BSP_ROOT)/nRF52/components/drivers_nrf/delay $(BSP_ROOT)/nRF52/components/drivers_nrf/gpiote $(BSP_ROOT)/nRF52/components/drivers_nrf/hal $(BSP_ROOT)/nRF52/components/drivers_nrf/lpcomp $(BSP_ROOT)/nRF52/components/drivers_nrf/nrf_soc_nosd $(BSP_ROOT)/nRF52/components/drivers_nrf/ppi $(BSP_ROOT)/nRF52/components/drivers_nrf/pstorage $(BSP_ROOT)/nRF52/components/drivers_nrf/pstorage/config $(BSP_ROOT)/nRF52/components/drivers_nrf/qdec $(BSP_ROOT)/nRF52/components/drivers_nrf/radio_config $(BSP_ROOT)/nRF52/components/drivers_nrf/rng $(BSP_ROOT)/nRF52/components/drivers_nrf/rtc $(BSP_ROOT)/nRF52/components/drivers_nrf/saadc $(BSP_ROOT)/nRF52/components/drivers_nrf/sdio $(BSP_ROOT)/nRF52/components/drivers_nrf/sdio/config $(BSP_ROOT)/nRF52/components/drivers_nrf/spi_master $(BSP_ROOT)/nRF52/components/drivers_nrf/spi_slave $(BSP_ROOT)/nRF52/components/drivers_nrf/swi $(BSP_ROOT)/nRF52/components/drivers_nrf/timer $(BSP_ROOT)/nRF52/components/drivers_nrf/twis_slave $(BSP_ROOT)/nRF52/components/drivers_nrf/twi_master $(BSP_ROOT)/nRF52/components/drivers_nrf/twi_master/incubated $(BSP_ROOT)/nRF52/components/drivers_nrf/twi_master/incubated/config $(BSP_ROOT)/nRF52/components/drivers_nrf/uart $(BSP_ROOT)/nRF52/components/drivers_nrf/wdt $(BSP_ROOT)/nRF52/components/libraries/util $(BSP_ROOT)/nRF52/examples/bsp $(BSP_ROOT)/nRF52/components/libraries/bootloader_dfu $(BSP_ROOT)/nRF52/components/libraries/bootloader_dfu/ble_transport $(BSP_ROOT)/nRF52/components/libraries/bootloader_dfu/hci_transport $(BSP_ROOT)/nRF52/components/libraries/button $(BSP_ROOT)/nRF52/components/libraries/crc16 $(BSP_ROOT)/nRF52/components/libraries/experimental_nfc/connection_handover $(BSP_ROOT)/nRF52/components/libraries/experimental_nfc/nfc_lib $(BSP_ROOT)/nRF52/components/libraries/experimental_nfc/uri $(BSP_ROOT)/nRF52/components/libraries/fifo $(BSP_ROOT)/nRF52/components/libraries/gpiote $(BSP_ROOT)/nRF52/components/libraries/hci $(BSP_ROOT)/nRF52/components/libraries/hci/config $(BSP_ROOT)/nRF52/components/libraries/ic_info $(BSP_ROOT)/nRF52/components/libraries/pwm $(BSP_ROOT)/nRF52/components/libraries/scheduler $(BSP_ROOT)/nRF52/components/libraries/sensorsim $(BSP_ROOT)/nRF52/components/libraries/sha256 $(BSP_ROOT)/nRF52/components/libraries/simple_timer $(BSP_ROOT)/nRF52/components/libraries/timer $(BSP_ROOT)/nRF52/components/libraries/trace $(BSP_ROOT)/nRF52/components/libraries/uart $(BSP_ROOT)/nRF52/components/libraries/util $(BSP_ROOT)/nRF52/components/softdevice/common/softdevice_handler $(BSP_ROOT)/nRF52/components/ble/common $(BSP_ROOT)/nRF52/components/ble/device_manager $(BSP_ROOT)/nRF52/ble_advertising $(BSP_ROOT)/nRF52/ble_services/ble_bas $(BSP_ROOT)/nRF52/ble_services/ble_dis $(BSP_ROOT)/nRF52/ble_services/ble_hrs
LIBRARY_DIRS += $(BSP_ROOT)/nRF52/SoftdeviceLibraries
LIBRARY_NAMES += compactcpp
ADDITIONAL_LINKER_INPUTS += 
MACOS_FRAMEWORKS += 
LINUX_PACKAGES += 

CFLAGS += -std=gnu99
CXXFLAGS += 
ASFLAGS += -mfpu=fpv4-sp-d16
LDFLAGS +=  
COMMONFLAGS += -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard
LINKER_SCRIPT := $(BSP_ROOT)/nRF52832_XXAA_S132_64k.lds

