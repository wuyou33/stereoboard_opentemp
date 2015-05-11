cmake_minimum_required (VERSION 2.6)


SET(CMAKE_C_COMPILER   arm-none-eabi-gcc)
SET(CMAKE_C_FLAGS "-g -O3 -mlittle-endian -mthumb -DHSE_VALUE=12000000 -mcpu=cortex-m4    -DARM_MATH_CM4 -ffreestanding -nostdlib  -Wl,-T,stm32_flash.ld")
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)


SET(STMLIB_PATH "../libs/STM32F4xx_DSP_StdPeriph_Lib_V1.1.0/")
SET(STDPH_PATH    "${STMLIB_PATH}/Libraries/STM32F4xx_StdPeriph_Driver/")
#CMSIS_PATH    = ${STMLIB_PATH}/Libraries/CMSIS/
#DEVSUP_PATH   = ${CMSIS_PATH}/Device/ST/STM32F4xx/Include/
#CORESUP_PATH  = ${CMSIS_PATH}/Include/
#DSP_LIB_PATH  = ${CMSIS_PATH}/DSP_Lib/Source/

project (MultiGaze)

include_directories(${STMLIB_PATH} ${STDPH_PATH})
add_executable(MultiGaze main.c)
