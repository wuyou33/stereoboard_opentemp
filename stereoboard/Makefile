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

CFLAGS        = -g -Og -mlittle-endian -mthumb -DHSE_VALUE=12000000
CFLAGS       += -mcpu=cortex-m4	-DARM_MATH_CM4
CFLAGS       += -ffreestanding -nostdlib
CFLAGS       += -DBOARD_FILE=\"./boards/board_$(BOARD).h\"
CFLAGS       += -DPROJECT_FILE=\"./projects/$(PROJECT).h\"
# https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
CFLAGS       += -Wunused-variable -Wuninitialized -Wreturn-type -Wimplicit-function-declaration -Wimplicit-int -Wenum-compare -Waddress

# to run from FLASH
CFLAGS       += -Wl,-T,stm32_flash.ld

SUBMODULE     = ../libs/README.md

STMLIB_PATH   = ../libs/STM32F4xx_DSP_StdPeriph_Lib_V1.1.0
STDPH_PATH    = ${STMLIB_PATH}/Libraries/STM32F4xx_StdPeriph_Driver
CMSIS_PATH    = ${STMLIB_PATH}/Libraries/CMSIS
DEVSUP_PATH   = ${CMSIS_PATH}/Device/ST/STM32F4xx/Include
CORESUP_PATH  = ${CMSIS_PATH}/Include
DSP_LIB_PATH  = ${CMSIS_PATH}/DSP_Lib/Source

ifeq ($(OS),Windows_NT)
    #Windows stuff
	CFLAGS+=-I${CURDIR}
else
    #Linux stuff
    CFLAGS+=-I./
endif

CFLAGS+=-I${STDPH_PATH}/inc
CFLAGS+=-I${DEVSUP_PATH}
CFLAGS+=-I${CORESUP_PATH}
CFLAGS+=-I../libs/STM32_USB_Device_Library/Core/inc
CFLAGS+=-I../libs/STM32_USB_Device_Library/Class/cdc/inc
CFLAGS+=-I../libs/STM32_USB_OTG_Driver/inc
CFLAGS+=-DUSE_USB_OTG_FS=1
CFLAGS+=-DSTM32F40XX -DUSE_STDPERIPH_DRIVER=1

# Standard Peripheral Library
STDP_LIB = libSTM32F4xx_StdPeriph_Driver.a

STDP_SRCS = \
${STDPH_PATH}/src/misc.c \
${STDPH_PATH}/src/stm32f4xx_adc.c \
${STDPH_PATH}/src/stm32f4xx_can.c \
${STDPH_PATH}/src/stm32f4xx_crc.c \
${STDPH_PATH}/src/stm32f4xx_cryp_aes.c \
${STDPH_PATH}/src/stm32f4xx_cryp_des.c \
${STDPH_PATH}/src/stm32f4xx_cryp_tdes.c \
${STDPH_PATH}/src/stm32f4xx_cryp_des.c \
${STDPH_PATH}/src/stm32f4xx_cryp.c \
${STDPH_PATH}/src/stm32f4xx_dac.c \
${STDPH_PATH}/src/stm32f4xx_dbgmcu.c \
${STDPH_PATH}/src/stm32f4xx_dcmi.c \
${STDPH_PATH}/src/stm32f4xx_dma.c \
${STDPH_PATH}/src/stm32f4xx_exti.c \
${STDPH_PATH}/src/stm32f4xx_flash.c \
${STDPH_PATH}/src/stm32f4xx_fsmc.c \
${STDPH_PATH}/src/stm32f4xx_gpio.c \
${STDPH_PATH}/src/stm32f4xx_hash_md5.c \
${STDPH_PATH}/src/stm32f4xx_hash_sha1.c \
${STDPH_PATH}/src/stm32f4xx_hash.c \
${STDPH_PATH}/src/stm32f4xx_i2c.c \
${STDPH_PATH}/src/stm32f4xx_iwdg.c \
${STDPH_PATH}/src/stm32f4xx_pwr.c \
${STDPH_PATH}/src/stm32f4xx_rcc.c \
${STDPH_PATH}/src/stm32f4xx_rng.c \
${STDPH_PATH}/src/stm32f4xx_rtc.c \
${STDPH_PATH}/src/stm32f4xx_sdio.c \
${STDPH_PATH}/src/stm32f4xx_spi.c \
${STDPH_PATH}/src/stm32f4xx_syscfg.c \
${STDPH_PATH}/src/stm32f4xx_tim.c \
${STDPH_PATH}/src/stm32f4xx_usart.c \
${STDPH_PATH}/src/stm32f4xx_wwdg.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_sub_q15.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_sub_q7.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_abs_q7.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_shift_q15.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_negate_q15.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_offset_q7.c \
${DSP_LIB_PATH}/SupportFunctions/arm_fill_q7.c \
${DSP_LIB_PATH}/SupportFunctions/arm_fill_q15.c \
${DSP_LIB_PATH}/SupportFunctions/arm_copy_q15.c \
${DSP_LIB_PATH}/SupportFunctions/arm_q15_to_q7.c \
${DSP_LIB_PATH}/SupportFunctions/arm_q7_to_q15.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_sub_q15.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_abs_q15.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_offset_q15.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_add_q15.c \
${DSP_LIB_PATH}/StatisticsFunctions/arm_min_q15.c \
${DSP_LIB_PATH}/SupportFunctions/arm_copy_q31.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_add_q31.c \
${DSP_LIB_PATH}/StatisticsFunctions/arm_max_q15.c \
${DSP_LIB_PATH}/FastMathFunctions/arm_sin_f32.c \
${DSP_LIB_PATH}/FastMathFunctions/arm_cos_f32.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_scale_f32.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_add_f32.c \
${DSP_LIB_PATH}/SupportFunctions/arm_float_to_q7.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_mult_f32.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_offset_f32.c



STDP_OBJS = $(STDP_SRCS:.c=.o)

USB_SRCS = \
../libs/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c \
../libs/STM32_USB_Device_Library/Core/src/usbd_core.c  \
../libs/STM32_USB_Device_Library/Core/src/usbd_ioreq.c \
../libs/STM32_USB_Device_Library/Core/src/usbd_req.c \
../libs/STM32_USB_OTG_Driver/src/usb_dcd.c      \
../libs/STM32_USB_OTG_Driver/src/usb_dcd_int.c

all: check_user_inputs $(SUBMODULE) print $(BIN_IMAGE)
	$(info Build Successful!)

check_user_inputs:
ifeq (,$(wildcard boards/board_$(BOARD).h))
	$(error Board header boards/board_$(BOARD).h doesn't exist)
endif
ifeq (,$(wildcard projects/$(PROJECT).h))
	$(error Project header projects/$(PROJECT).h doesn't exist)
endif

print:
	$(info Including boards/board_$(BOARD).h)
	$(info Including projects/$(PROJECT).h)
	$(info )
	
$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

MAIN_SRCS = \
main.c \
main_parameters.h \
stm32f4xx_it.c \
system_stm32f4xx.c \
../common/led.c \
../common/led.h \
../common/stereoprotocol.c\
../common/stereoprotocol.h\
forward_velocity_estimator.c \
forward_velocity_estimator.h \
disparity_map_functions.c \
disparity_map_functions.h \
forward_velocity_estimator.h \
tmg3993.c \
tmg3993.h \
image.c \
image.h \
i2c.c \
i2c.h \
usart.c \
usart.h \
dcmi.c \
dcmi.h \
sccb.h \
sccb.c \
tcm8230.c \
tcm8230.h \
ov7670.h \
ov7670.c \
ov2640.c \
ov7670_drv.c \
ov9712_drv.c \
jpeg.c \
jpeg.h \
cpld.c \
cpld.h \
stereo_vision.c \
stereo_vision.h \
optic_flow.c \
optic_flow.h \
window_detection.c \
window_detection.h \
filter_color.c \
filter_color.h \
color.c \
color.h \
entropy_patches.c \
entropy_patches.h \
usbd_cdc_vcp.c \
usbd_cdc_vcp.h \
usb.h \
usb.c \
usb_bsp.c \
usbd_desc.c \
usbd_usr.c \
usb_core.c \
eeprom.c \
hmc5883.h \
hmc5883.c \
../common/utils.h \
../common/utils.c \
divergence.h \
divergence.c \
xprintf.c \
xprintf.h \
droplet_algorithm.c \
distance_matrix.c \
odometry.c \
textons.h \
textons.c \
learning.h \
learning.c


MAIN_OBJS = $(MAIN_SRCS:.c=.o)
MAIN_LST = $(MAIN_SRCS:.c=.lst)

$(EXECUTABLE): 	\
		startup_stm32f4xx.s \
		$(MAIN_OBJS) \
		${USB_SRCS} \
		${STDP_LIB}
	$(CC) $(CFLAGS) $^ -o $@  -L./ -lSTM32F4xx_StdPeriph_Driver -lgcc -lc

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
	rm -rf *.lst *.o

mrproper:
	make clean -C ./
	make stdplib_clean -C ./

$(SUBMODULE):
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
    BMP_PORT ?= /dev/cu.usbmodemD5DA96C1
endif


upload:
	@echo "Using Black Magic Probe"
	@echo "Using GDB = $(GDB)"
	$(GDB) --batch \
                -ex 'target extended-remote $(BMP_PORT)' \
                -x black_magic_probe_flash.scr ${EXECUTABLE}
#endif

cleanspaces:
	make -C style

testall:
	make clean && make BOARD=ov7670 && make clean && make BOARD=ov2640 && make clean && make BOARD=ov9712 && make clean && make BOARD=festo

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

stereoavoid: PROJECT_FILE=projects/stereo_avoid_demo.h BOARD_FILE=boards/board_6.h
stereoavoid: all


explorer: PROJECT_FILE="projects/explorer.h" BOARD_FILE="boards/board_6.h"
explorer: all



.PHONY: all clean style cleanspaces
