
TOP = .

# General project files
VPATH					:=	$(TOP)/src						\
							$(TOP)/src/system				\
                             
PROJECT_INC_PATHS		:=	-I$(TOP)/inc					\

PROJECT_SOURCE			:=	startup_stm32f446xx.s			\
							system_stm32f4xx.c				\
							stm32f4xx_it.c					\
							main.c							\
							syscall.c						\
							debug.c							\
							blink.c							\
							command.c						\

# FreeRTOS
VPATH					+=  $(TOP)/lib/FreeRTOS

PROJECT_INC_PATHS		+=	-I$(TOP)/lib/FreeRTOS/include

PROJECT_SOURCE			+=	croutine.c						\
							event_groups.c					\
							heap_2.c						\
							list.c							\
							port.c							\
							queue.c							\
							tasks.c							\
							timers.c

# FreeRTOS-Plus-CLI
VPATH					+=  $(TOP)/lib/FreeRTOS-Plus-CLI

PROJECT_INC_PATHS		+=	-I$(TOP)/lib/FreeRTOS-Plus-CLI

PROJECT_SOURCE			+=	FreeRTOS_CLI.c						

# STM32 Library
VPATH					+=  $(TOP)/lib/STM32F4xx_StdPeriph_Driver/src

PROJECT_INC_PATHS		+=	-I$(TOP)/lib/STM32F4xx_StdPeriph_Driver/inc

PROJECT_SOURCE			+=	misc.c                       \
							stm32f4xx_gpio.c             \
							stm32f4xx_rcc.c              \
							stm32f4xx_usart.c            \
#							stm32f4xx_rtc.c              \
#							stm32f4xx_tim.c              \
#							stm32f4xx_adc.c              \
#							stm32f4xx_can.c              \
#							stm32f4xx_cec.c              \
#							stm32f4xx_crc.c              \
#							stm32f4xx_cryp.c             \
#							stm32f4xx_cryp_aes.c         \
#							stm32f4xx_cryp_des.c         \
#							stm32f4xx_cryp_tdes.c        \
#							stm32f4xx_dac.c              \
#							stm32f4xx_dbgmcu.c           \
#							stm32f4xx_dcmi.c             \
#							stm32f4xx_dma.c              \
#							stm32f4xx_dma2d.c            \
#							stm32f4xx_exti.c             \
#							stm32f4xx_flash.c            \
#							stm32f4xx_flash_ramfunc.c    \
#							stm32f4xx_fmpi2c.c           \
#							stm32f4xx_fsmc.c             \
#							stm32f4xx_hash.c             \
#							stm32f4xx_hash_md5.c         \
#							stm32f4xx_hash_sha1.c        \
#							stm32f4xx_iwdg.c             \
#							stm32f4xx_ltdc.c             \
#							stm32f4xx_pwr.c              \
#							stm32f4xx_qspi.c             \
#							stm32f4xx_rng.c              \
#							stm32f4xx_sai.c              \
#							stm32f4xx_sdio.c             \
#							stm32f4xx_spdifrx.c          \
#							stm32f4xx_spi.c              \
#							stm32f4xx_syscfg.c           \
#							stm32f4xx_wwdg.c             \

MCU_CC_FLAGS = $(CORTEX_M4_HWFP_CC_FLAGS)
MCU_LIB_PATH = $(CORTEX_M4_HWFP_LIB_PATH)
                   
PROJECT_NAME = STM32F446

PROJECT_SYMBOLS	=	-DUSE_STDPERIPH_DRIVER                         \
					-DSTM32F446xx                                  \
					-D__FPU_PRESENT=1                              \
                             
PROJECT_LINKER_SCRIPT = $(TOP)/STM32F446ZETx_FLASH.ld

PROJECT_DOXYGEN_CONFIG = config.doxyfile

PROJECT_OPENOCD_CONFIG = board/st_nucleo_f4.cfg

sinclude $(TOP)/common.mk

# Force to update the builded time in main.c
obj/main.o : force 