################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
..\SUGENTEM_N_52.cpp 

C_SRCS += \
..\Algorithm.c \
..\ad_converter.c \
..\dbsct.c \
..\hardware_infomation_for_sugentem_n_52.c \
..\intprg.c \
..\loging.c \
..\mode.c \
..\motor_control.c \
..\periodic_interrupt_function.c \
..\resetprg.c \
..\sbrk.c \
..\sci.c \
..\spi.c \
..\timer_for_sugentem_n_52.c \
..\vecttbl.c 

C_DEPS += \
./Algorithm.d \
./ad_converter.d \
./dbsct.d \
./hardware_infomation_for_sugentem_n_52.d \
./intprg.d \
./loging.d \
./mode.d \
./motor_control.d \
./periodic_interrupt_function.d \
./resetprg.d \
./sbrk.d \
./sci.d \
./spi.d \
./timer_for_sugentem_n_52.d \
./vecttbl.d 

OBJS += \
./Algorithm.obj \
./SUGENTEM_N_52.obj \
./ad_converter.obj \
./dbsct.obj \
./hardware_infomation_for_sugentem_n_52.obj \
./intprg.obj \
./loging.obj \
./mode.obj \
./motor_control.obj \
./periodic_interrupt_function.obj \
./resetprg.obj \
./sbrk.obj \
./sci.obj \
./spi.obj \
./timer_for_sugentem_n_52.obj \
./vecttbl.obj 

CPP_DEPS += \
./SUGENTEM_N_52.d 


# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c
	@echo 'Scanning and building file: $<'
	@echo 'Invoking: Scanner and Compiler'
	ccrx  -MM -MP -output=dep="$(@:%.obj=%.d)" -MT="$(@:%.obj=%.obj)" -MT="$(@:%.obj=%.d)" -lang=c99   -include="C:\PROGRA~2\RENESA~1\CS_~1\CC\CC-RX\V206~1.00/include"  -debug -isa=rxv1 -speed -fpu -alias=noansi -nologo -nomessage  -define=__RX   "$<"
	ccrx -lang=c99 -output=obj="$(@:%.d=%.obj)"  -include="C:\PROGRA~2\RENESA~1\CS_~1\CC\CC-RX\V206~1.00/include"  -debug -isa=rxv1 -speed -fpu -alias=noansi -nologo -nomessage  -define=__RX   "$<"
	@echo 'Finished scanning and building: $<'
	@echo.

%.obj: ../%.cpp
	@echo 'Scanning and building file: $<'
	@echo 'Invoking: Scanner and Compiler'
	ccrx  -MM -MP -output=dep="$(@:%.obj=%.d)" -MT="$(@:%.obj=%.obj)" -MT="$(@:%.obj=%.d)" -lang=cpp   -include="C:\PROGRA~2\RENESA~1\CS_~1\CC\CC-RX\V206~1.00/include"  -debug -isa=rxv1 -speed -fpu -alias=noansi -nologo -nomessage  -define=__RX   "$<"
	ccrx -lang=cpp -output=obj="$(@:%.d=%.obj)"  -include="C:\PROGRA~2\RENESA~1\CS_~1\CC\CC-RX\V206~1.00/include"  -debug -isa=rxv1 -speed -fpu -alias=noansi -nologo -nomessage  -define=__RX   "$<"
	@echo 'Finished scanning and building: $<'
	@echo.

