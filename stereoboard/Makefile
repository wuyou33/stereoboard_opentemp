TARGET        = StereoBoard
BOARD        ?= default
PROJECT      ?= example

EXECUTABLE    = $(TARGET).elf
BIN_IMAGE     = $(TARGET).bin

CROSS        ?= @arm-none-eabi-

CC            = $(CROSS)gcc
GDB           = $(CROSS)gdb
OBJCOPY       = $(CROSS)objcopy
RANLIB        = $(CROSS)ranlib
AR            = $(CROSS)ar
GDB           = $(CROSS)gdb


COMMONFLAGS = -mcpu=cortex-m4 -mthumb

CFLAGS        = -g -Og -mlittle-endian -DHSE_VALUE=12000000
CFLAGS       += $(COMMONFLAGS) -DARM_MATH_CM4 -fsingle-precision-constant
CFLAGS       += -ffreestanding -nostdlib

ifeq ($(findstring .h,$(BOARD)), .h)
	BOARD_PATH   = $(BOARD)
else
	BOARD_PATH   = boards/board_$(BOARD).h
endif
ifeq ($(findstring .h,$(PROJECT)), .h)
	PROJECT_PATH = $(PROJECT)
else
	PROJECT_PATH = projects/$(PROJECT).h
endif
CFLAGS   += -DBOARD_FILE=\"./$(BOARD_PATH)\" -DPRoJECT_FILE=\"./$(PROJECT_PATH)\"

# https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
CFLAGS       += -Wunused-variable -Wuninitialized -Wreturn-type -Wimplicit-function-declaration -Wimplicit-int -Wenum-compare -Waddress -Wstrict-prototypes #-Wextra

LFLAGS        =  -lSTM32F4xx_StdPeriph_Driver -lgcc -lm -lc
# to run from FLASH
LFLAGS        += -Wl,-T,stm32_flash.ld

STMLIB_PATH   = ../libs/STM32F4xx_DSP_StdPeriph_Lib_V1.7.1
STDPH_PATH    = ${STMLIB_PATH}/Libraries/STM32F4xx_StdPeriph_Driver
CMSIS_PATH    = ${STMLIB_PATH}/Libraries/CMSIS
CORESUP_PATH  = ${CMSIS_PATH}/Include
DEVSUP_PATH   = ${CMSIS_PATH}/Device/ST/STM32F4xx/Include
DSP_LIB_PATH  = ${CMSIS_PATH}/DSP_Lib/Source
USB_PATH      = ../libs/STM32_USB_Device_Library/
USB_OTG_PATH  = ../libs/STM32_USB_OTG_Driver/

ifeq ($(OS),Windows_NT)
    #Windows stuff
	CFLAGS+=-I${CURDIR}
else
    #Linux stuff
    CFLAGS+=-I./
endif

CFLAGS+=-Idrivers/inc
CFLAGS+=-I${CORESUP_PATH}
CFLAGS+=-I${DEVSUP_PATH}
CFLAGS+=-I${STDPH_PATH}/inc

CFLAGS+=-I${USB_PATH}
CFLAGS+=-I${USB_PATH}/Core/inc
CFLAGS+=-I${USB_PATH}/Class/cdc/inc

CFLAGS+=-I${USB_OTG_PATH}/inc
CFLAGS+=-DUSE_USB_OTG_FS=1
CFLAGS+=-DSTM32F40_41xxx -DUSE_STDPERIPH_DRIVER=1

CFLAGS+=-I../common

# Standard Peripheral Library
STDP_LIB = libSTM32F4xx_StdPeriph_Driver.a

STDP_SRCS  = $(wildcard ${STDPH_PATH}/src/*.c)
STDP_SRCS := $(filter-out ${STDPH_PATH}/src/stm32f4xx_fmc.c, $(STDP_SRCS))

STDP_SRCS += $(wildcard ${DSP_LIB_PATH}/**/*.c)

STDP_OBJS = $(STDP_SRCS:.c=.o)

USB_SRCS = \
../libs/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c \
../libs/STM32_USB_Device_Library/Core/src/usbd_core.c  \
../libs/STM32_USB_Device_Library/Core/src/usbd_ioreq.c \
../libs/STM32_USB_Device_Library/Core/src/usbd_req.c \
../libs/STM32_USB_OTG_Driver/src/usb_dcd.c      \
../libs/STM32_USB_OTG_Driver/src/usb_dcd_int.c

all: check_user_inputs submodule print $(BIN_IMAGE)
	$(info Build Successful!)

check_user_inputs:
ifeq (,$(wildcard $(BOARD_PATH)))
	$(error Board header $(BOARD_PATH) doesn't exist)
endif
ifeq (,$(wildcard $(PROJECT_PATH)))
	$(error Project header $(PROJECT_PATH) doesn't exist)
endif

print:
	$(info Including boards/board_$(BOARD).h)
	$(info Including projects/$(PROJECT).h)
	$(info )

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

MAIN_SRCS  = $(wildcard drivers/src/*.c)
MAIN_SRCS += $(wildcard ../common/*.c)
MAIN_SRCS += $(wildcard *.c)

MAIN_SRCS := $(filter-out droplet.c, $(MAIN_SRCS))

MAIN_OBJS = $(MAIN_SRCS:.c=.o)
MAIN_LST = $(MAIN_SRCS:.c=.lst)

$(EXECUTABLE): 	\
    startup_stm32f4xx.s \
    ${STDP_LIB} \
    ${USB_SRCS} \
		$(MAIN_OBJS)

	$(CC) $(CFLAGS) $^ -o $@  -L./ $(LFLAGS)

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
	rm -rf *.lst *.o

mrproper:
	make clean -C ./
	make stdplib_clean -C ./

submodule:
	cd ../ && git submodule init && git submodule sync && git submodule update

com:
	minicom -D /dev/ttyACM2 -o -w -t ansi

#ifeq ($(BMP_PORT),)
#upload:
#	../stlink/st-flash write ${BIN_IMAGE} 0x08000000
#else

ifeq ($(OS),Windows_NT)
    #Windows stuff
    BMP_PORT ?= COM2
else
    #Linux stuff
    BMP_PORT ?= /dev/ttyACM0
endif


upload:
	@echo "Using Black Magic Probe"
	@echo "Using GDB = $(GDB)"
	$(GDB) --batch \
                -ex 'target extended-remote $(BMP_PORT)' \
                -x black_magic_probe_flash.scr ${EXECUTABLE}

erase_flash:
	@echo "Using Black Magic Probe"
	@echo "Using GDB = $(GDB)"
	$(GDB) --batch \
                -ex 'target extended-remote $(BMP_PORT)' \
                -x black_magic_probe_erase_flash.scr ${EXECUTABLE}
#endif

cleanspaces:
	make -C style

quick_build: print $(BIN_IMAGE)

BOARD_FILES = $(wildcard boards/*.h)
testallboards:
	$(foreach var,$(BOARD_FILES), make clean && make quick_build BOARD=$(var);)

PROJECT_FILES = $(wildcard projects/*.h)
testallprojects: 
	$(foreach var,$(PROJECT_FILES), make clean && make quick_build PROJECT=$(var);)

style:
	find . -path docs -prune -o -name '*.[ch]' -exec sed -i {} -e 's/\r$$//' \;
	find . -path docs -prune -o -name 'Makefile*' -exec sed -i {} -e 's/\r$$//' \;
	find . -path docs -prune -o -name '*.[ch]' -exec sed -i {} -e 's/[[:space:]]*$$//' \;
	find . -path docs -prune -o -name 'Makefile*' -exec sed -i {} -e 's/[[:space:]]*$$//' \;
	./fix_code_style.sh *.c
	./fix_code_style.sh *.h

stdplib: $(STDP_LIB)

$(STDP_LIB): $(STDP_OBJS)
	$(AR) -r $(STDP_LIB) $(STDP_OBJS)
	$(RANLIB) $(STDP_LIB)

stdplib_clean:
	-rm -f $(STDP_OBJS)
	-rm -f $(STDP_LIB)

lst: $(MAIN_LST)

%.lst: %.c
	$(CC) -c -g -Wa,-a,-ad $(CFLAGS) $< > $@

roland: PROJECT_FILE=roland
roland: all

kimberly: PROJECT=kimberly
kimberly: all

stereoavoid: PROJECT_FILE=projects/stereo_avoid_demo.h BOARD_FILE="boards/board_6.h"
stereoavoid: all


explorer: PROJECT_FILE="projects/explorer.h" BOARD_FILE="boards/board_6.h"
explorer: all



.PHONY: all clean style cleanspaces
