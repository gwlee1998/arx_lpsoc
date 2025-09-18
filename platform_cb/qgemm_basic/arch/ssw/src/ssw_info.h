#ifndef __SSW_INFO_H__
#define __SSW_INFO_H__


//#define USE_SMALL_RAM_MANUALLY
#define SMALL_RAM_BASEADDR 0xe0000000
#define SMALL_RAM_SIZE 0x10000
#define SMALL_RAM_LASTADDR 0xe000ffff
//#define USE_LARGE_RAM_MANUALLY
#define LARGE_RAM_BASEADDR 0x0
#define LARGE_RAM_SIZE 0x40000000
#define LARGE_RAM_LASTADDR 0x3fffffff
#define STACK_SIZE 0x10000
#define DATA_ALIGN_SIZE 16
#define PRINTF_USING_UART
//#define PRINTF_USING_OLED
#define TEMPORARY_CACHING_HEAP_START 0
#define TEMPORARY_CACHING_HEAP_SIZE 0
#define TEMPORARY_CACHING_HEAP_LAST 0
#define EXT_MRAM_CYCLE 0
//#define USE_PROFILING
//#define SILENT_FOR_PROFILING
//#define USE_TCACHING
//#define USE_HW_BARRIER
//#define USE_SW_BARRIER
//#define USE_SDRAM
//#define USE_VIM
//#define USE_VOM
//#define USE_VDMA
//#define USE_FLORIAN_SP
//#define USE_FLORIAN_DP
//#define USE_REUSE_MEMORY_ALLOCATOR
//#define CACHING_NONE
#define CACHING_SAFE
//#define CACHING_MOST
//#define CACHING_ALL
#define USE_IROM
#define USE_SLOW_DRAM
//#define USE_FAST_DRAM
//#define USE_EXT_MRAM
#define USE_DDR
#define USE_SMALL_RAM
#define USE_LARGE_RAM
//#define USE_BOOT_MODE
#define USE_CACHE
#define USE_TIMER
//#define USE_PLIC
//#define USE_CORE_JTAG
//#define USE_OLED
//#define USE_OLED_BW
//#define USE_OLED_RGB
//#define USE_SPI_FLASH
#define USE_UART_PRINTF
//#define USE_OLEDBW_PRINTF
//#define USE_FUSE_BOX
//#define USE_ADC
//#define USE_USER_INTERRUPTS
//#define USE_ARDUCAM
//#define USE_BLUETOOTH
//#define USE_WIFI
//#define USE_JPEG_ENCODER
//#define USE_JPEGLS_ENCODER
//#define USE_HBC1_TX
//#define USE_HBC1_RX
//#define USE_HBC1
#define USE_CORE_PERI_GROUP
//#define USE_NOC_DEBUG
//#define USE_PACT
//#define USE_STARC
#define USE_DCA
//#define USE_I2S
//#define USE_DMA
//#define USE_FLORIAN
//#define USE_TRAFFIC_GENERATOR
//#define USE_VTA
//#define USE_LCD_SHIELD
//#define USE_GPIO_PINMUX
//#define USE_USER_DDR4
//#define USE_USER_DDR3
//#define USE_C2C
//#define USE_EXTINPUT_BACKEND
//#define USE_EDGE_VIDEO_SYSTEM
//#define USE_HDMI
//#define USE_CIS
//#define USE_SPI_LCD
//#define USE_TFT_LCD
#define USE_RVC_ORCA_PLUS
//#define USE_MULTICORE
#define USE_CORE_USER
#define USE_CORE_TEAM
//#define USE_LOCK_USER
//#define USE_LOCK_SYSTEM
//#define USE_BARRIER_USER
//#define USE_BARRIER_SYSTEM
//#define USE_AUTO_ID
//#define USE_GLOBAL_TAG
//#define USE_UART_READYMADE
//#define USE_SPI_READYMADE
//#define USE_I2C_READYMADE
//#define USE_UART_USER
//#define USE_SPI_USER
//#define USE_I2C_USER
#define USE_LED_CHECKER
//#define USE_AIOIF
//#define USE_SWITCH_SLIDE_READYMADE
//#define USE_C2C_MASTER
//#define USE_C2C_SLAVE
#define USE_CORE
#define USE_REAL_CORE
//#define USE_BARRIER
//#define USE_LOCK
//#define USE_SPI_SYSTEM
#define USE_UART_SYSTEM
//#define USE_I2C_SYSTEM
//#define USE_GPIO
#define USE_UART
//#define USE_SPI
//#define USE_I2C
#define USE_SRAM
#define USE_DRAM
//#define CACHING_MOST_OR_ALL
#define NUM_THREAD_PER_TEAM 1
#define TEXT_MEMORY_REGION dram
#define SINGLE_WRITE_MEMORY_REGION dram
#define MULTI_WRITE_MEMORY_REGION dram
#define UNCACHEABLE_MEMORY_REGION sram
#define APP_ADDR 0x0
#define SRAM_ADDR 0xe0000000
#define SRAM_SIZE 0x10000
#define SRAM_ACTUAL_SIZE 0x10000
#define DRAM_ADDR 0x0
#define DRAM_SIZE 0x40000000
#define DRAM_ACTUAL_SIZE 0x40000000

#endif