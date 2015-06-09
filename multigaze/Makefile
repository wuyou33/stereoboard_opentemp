TARGET        = MultiGaze
CAMERA	      = StereoBoard

EXECUTABLE    = $(TARGET).elf
BIN_IMAGE     = $(TARGET).bin

CROSS        ?= arm-none-eabi-

# Tunnel from 1 to 6
TUNNEL       ?= _NONE

# When uploading a single camera, defaultly upload camera 1
CAMNUMBER    ?=1

CC            = $(CROSS)gcc
GDB           = $(CROSS)gdb
OBJCOPY       = $(CROSS)objcopy
RANLIB        = $(CROSS)ranlib
AR            = $(CROSS)ar
GDB           = $(CROSS)gdb

CFLAGS        = -g -O3 -mlittle-endian -mthumb -DHSE_VALUE=12000000
CFLAGS       += -mcpu=cortex-m4	-DARM_MATH_CM4
CFLAGS       += -ffreestanding -nostdlib
CFLAGS       += -DTUNNEL$(TUNNEL)

# to run from FLASH
CFLAGS       += -Wl,-T,stm32_flash.ld

SUBMODULE     = ../libs/README.md

STMLIB_PATH   = ../libs/STM32F4xx_DSP_StdPeriph_Lib_V1.1.0/
STDPH_PATH    = ${STMLIB_PATH}/Libraries/STM32F4xx_StdPeriph_Driver/
CMSIS_PATH    = ${STMLIB_PATH}/Libraries/CMSIS/
DEVSUP_PATH   = ${CMSIS_PATH}/Device/ST/STM32F4xx/Include/
CORESUP_PATH  = ${CMSIS_PATH}/Include/
DSP_LIB_PATH  = ${CMSIS_PATH}/DSP_Lib/Source/

CFLAGS+=-I./
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
${DSP_LIB_PATH}/BasicMathFunctions/arm_sub_q7.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_abs_q7.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_offset_q7.c \
${DSP_LIB_PATH}/SupportFunctions/arm_fill_q7.c \
${DSP_LIB_PATH}/SupportFunctions/arm_fill_q15.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_sub_q15.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_abs_q15.c \
${DSP_LIB_PATH}/BasicMathFunctions/arm_offset_q15.c \
${DSP_LIB_PATH}/StatisticsFunctions/arm_min_q15.c \
${DSP_LIB_PATH}/StatisticsFunctions/arm_max_q15.c

STDP_OBJS = $(STDP_SRCS:.c=.o)

all: $(SUBMODULE) $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

MAIN_SRCS = \
main.c \
stm32f4xx_it.c \
system_stm32f4xx.c \
../common/led.c \
tunnel.c \
usart.c \
../common/utils.c


MAIN_OBJS = $(MAIN_SRCS:.c=.o)
MAIN_LST = $(MAIN_SRCS:.c=.lst)

$(EXECUTABLE): 	\
		startup_stm32f4xx.s \
		$(MAIN_OBJS) \
		${STDP_LIB}
	@echo "========= TUNNEL = $(TUNNEL) ========="
	$(CC) $(CFLAGS) $^ -o $@  -L./ -lSTM32F4xx_StdPeriph_Driver -lgcc -lc

clean:
	rm -rf $(EXECUTABLE)
	rm -rf *.elf
	rm -rf $(BIN_IMAGE)
	rm -rf *.lst *.o

mrproper:
	make clean -C ./
	make stdplib_clean -C ./

$(SUBMODULE):
	cd ../ && git submodule init && git submodule sync && git submodule update

com:
	minicom -D /dev/ttyACM2 -o -w -t ansi

BMP_PORT ?= /dev/ttyACM0

singlecamera:
	make -C ../stereoboard clean && make -C ../stereoboard BOARD=mg${CAMNUMBER} #SOMETHING=-DSEND_COMMANDS=1 
	cp ../stereoboard/${CAMERA}.elf ./${CAMERA}${CAMNUMBER}.elf

uploadsinglecamera: ${CAMERA}${CAMNUMBER}.elf
	$(GDB) --batch \
		-ex 'target extended-remote $(BMP_PORT)' \
		-x black_magic_probe_flash_camera${CAMNUMBER}.scr ${CAMERA}${CAMNUMBER}.elf


${CAMERA}1.elf:
	make -C ../stereoboard clean && make -C ../stereoboard BOARD=mg1
	cp ../stereoboard/${CAMERA}.elf ./${CAMERA}1.elf

${CAMERA}2.elf:
	make -C ../stereoboard clean && make -C ../stereoboard BOARD=mg2
	cp ../stereoboard/${CAMERA}.elf ./${CAMERA}2.elf

${CAMERA}3.elf:
	make -C ../stereoboard clean && make -C ../stereoboard BOARD=mg3
	cp ../stereoboard/${CAMERA}.elf ./${CAMERA}3.elf

${CAMERA}4.elf:
	make -C ../stereoboard clean && make -C ../stereoboard BOARD=mg4
	cp ../stereoboard/${CAMERA}.elf ./${CAMERA}4.elf

${CAMERA}5.elf:
	make -C ../stereoboard clean && make -C ../stereoboard BOARD=mg5
	cp ../stereoboard/${CAMERA}.elf ./${CAMERA}5.elf

${CAMERA}6.elf:
	make -C ../stereoboard clean && make -C ../stereoboard BOARD=mg6
	cp ../stereoboard/${CAMERA}.elf ./${CAMERA}6.elf

upload: ${EXECUTABLE} ${CAMERA}1.elf ${CAMERA}2.elf ${CAMERA}3.elf ${CAMERA}4.elf ${CAMERA}5.elf ${CAMERA}6.elf
	@echo "Using Black Magic Probe"
	@echo "Using GDB = $(GDB)"
	$(GDB) --batch \
                -ex 'target extended-remote $(BMP_PORT)' \
                -x black_magic_probe_flash.scr ${EXECUTABLE}

upload2: ${EXECUTABLE}
	$(GDB) --batch \
		-ex 'target extended-remote $(BMP_PORT)' \
		-x black_magic_probe_flash_multigazeonly.scr ${EXECUTABLE}

cleanspaces:
	make -C style

style:
	find . -path docs -prune -o -name '*.[ch]' -exec sed -i {} -e 's/\r$$//' \;
	find . -path docs -prune -o -name 'Makefile*' -exec sed -i {} -e 's/\r$$//' \;
	find . -path docs -prune -o -name '*.[ch]' -exec sed -i {} -e 's/[[:space:]]*$$//' \;
	find . -path docs -prune -o -name 'Makefile*' -exec sed -i {} -e 's/[[:space:]]*$$//' \;
	../stereoboard/fix_code_style.sh *.c
	../stereoboard/fix_code_style.sh *.h

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

.PHONY: all clean style cleanspaces
