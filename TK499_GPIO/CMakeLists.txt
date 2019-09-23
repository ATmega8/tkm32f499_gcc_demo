#CMake 最低版本号要求
cmake_minimum_required ( VERSION 3.1 )

#项目信息
project ( tkm32f499)
set(CMAKE_SYSTEM_NAME Generic)
enable_language(ASM)

set (CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)
#包含的头文件
include_directories ( project/ )
include_directories ( Hal_lib/inc/ )
include_directories ( CMSIS_and_startup/ )
include_directories ( CMSIS_and_startup/CMSIS/ )

#需要编译的文件
aux_source_directory ( project/ SRCS ) 
aux_source_directory ( Hal_lib/src/ SRCS )
aux_source_directory ( CMSIS_and_startup/ SRCS )
	
#CC AR LD AS
set ( CMAKE_C_COMPILER "arm-none-eabi-gcc" )
set ( CMAKE_C_AR "arm-none-eabi-ar" )
set ( CMAKE_C_LINKER "arm-none-eabi-ld" )
set ( CMAKE_ASM_COMPILER "arm-none-eabi-gcc" )

#CFLAGS
set ( CMAKE_C_FLAGS "-g3 -O0 -Wall -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fdata-sections -ffunction-sections")

#LDFLAGS
set ( CMAKE_EXE_LINKER_FLAGS "-specs=nano.specs -Wl,--gc-sections" )
set ( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${CMAKE_CURRENT_LIST_DIR}/CMSIS_and_startup/TKM32F499_FLASH.ld" )

#ASMFLAGS
set ( CMAKE_ASM_FLAGS "-x assembler-with-cpp ${CMAKE_C_FLAGS}")

#指定生成目标
add_executable ( main.elf ${SRCS} CMSIS_and_startup/startup_tkm32f499.s)
target_link_libraries( main.elf nosys )

#链接静态库
#target_link_libraries ( main.elf STM32F4StdPeripherals )


#生成二进制文件
add_custom_target ( main.bin DEPENDS main.elf COMMAND arm-none-eabi-objcopy -Obinary main.elf main.bin )
add_custom_target ( size DEPENDS main.elf COMMAND arm-none-eabi-size main.elf )
