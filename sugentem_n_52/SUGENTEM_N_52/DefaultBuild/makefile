################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf *.sni *.dbg *.lst *.lis *.lpp *.map *.x *.lbp src/*.src src/*.p src/*.pp src/*.mrc *.bls *.libelf *.lbk *.abs *.fsy

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include .metadata/.plugins/org.eclipse.cdt.make.core/subdir.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_UPPER_DEPS)),)
-include $(CC_UPPER_DEPS)
endif
ifneq ($(strip $(C++_UPPER_DEPS)),)
-include $(C++_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_UPPER_DEPS)),)
-include $(CXX_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_UPPER_DEPS)),)
-include $(CPP_UPPER_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CP_DEPS)),)
-include $(CP_DEPS)
endif
ifneq ($(strip $(SRC_UPPER_DEPS)),)
-include $(SRC_UPPER_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(SRC_DEPS)),)
-include $(SRC_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(CP_UPPER_DEPS)),)
-include $(CP_UPPER_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(OBJ_DEPS)),)
-include $(OBJ_DEPS)
endif
ifneq ($(strip $(ABS_DEPS)),)
-include $(ABS_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(O_DEPS)),)
-include $(O_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

ifneq ($(strip $(SRC_UPPER_SRCS)),)
ALL_ASMS += $(SRC_UPPER_SRCS) 
endif
ifneq ($(strip $(SRC_SRCS)),)
ALL_ASMS += $(SRC_SRCS) 
endif
ifneq ($(strip $(S_UPPER_SRCS)),)
ALL_ASMS += $(S_UPPER_SRCS) 
endif
ifneq ($(strip $(S_SRCS)),)
ALL_ASMS += $(S_SRCS) 
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
CONVERTOR_OUTPUT_OUTPUTS += \
SUGENTEM_N_52.mot \

LINKER_OUTPUT_OUTPUTS += \
SUGENTEM_N_52.abs \

LIBRARY_GENERATOR_OUTPUTTYPE_OUTPUTS += \
SUGENTEM_N_52.lib \

# All Target
all: SUGENTEM_N_52.mot
	@echo 'Build complete.'

# Tool invocations
SUGENTEM_N_52.mot: $(LINKER_OUTPUT_OUTPUTS)
	@echo 'Invoking: Converter'
	rlink   "C:\git\Duo_monitos\sugentem_n_52\SUGENTEM_N_52\DefaultBuild\SUGENTEM_N_52.abs" -form=stype  -output="C:\git\Duo_monitos\sugentem_n_52\SUGENTEM_N_52\DefaultBuild\SUGENTEM_N_52.mot"
	@echo 'Finished building target:'
	@echo.
	$(MAKE) --no-print-directory post-build

SUGENTEM_N_52.abs: $(OBJS) $(LIBRARY_GENERATOR_OUTPUTTYPE_OUTPUTS) $(ALL_ASMS) 
	@echo 'Invoking: Linker'
	@echo 'Building target:'
	rlink  -subcommand="LinkerSubCommand.tmp"  -list="SUGENTEM_N_52.map" -show -nooptimize -debug -output="C:\git\Duo_monitos\sugentem_n_52\SUGENTEM_N_52\DefaultBuild\SUGENTEM_N_52.abs" -rom=D=R,D_1=R_1,D_2=R_2 -nocompress -nologo -nomessage
	@echo 'Finished building:'
	@echo.

SUGENTEM_N_52.lib:  
	@echo 'Invoking: Standard Library'
	lbgrx -lang=c99 -head=runtime,math,stdlib,string,new,cppstring -isa=rxv1 -speed -fpu -nologo -output="SUGENTEM_N_52.lib"
	@echo 'Finished building:'
	@echo.

# Other Targets
clean:
	-$(RM) $(LIBRARY_GENERATOR_OUTPUTTYPE_OUTPUTS) $(CONVERTOR_OUTPUT_OUTPUTS) 
	-rm -rf ""*.obj ""*.d ""*.src ""*.p ""*.pp ""*.mrc ""*.prn
	-rm -rf ".metadata/.plugins/org.eclipse.cdt.make.core"/*.obj ".metadata/.plugins/org.eclipse.cdt.make.core"/*.d ".metadata/.plugins/org.eclipse.cdt.make.core"/*.src ".metadata/.plugins/org.eclipse.cdt.make.core"/*.p ".metadata/.plugins/org.eclipse.cdt.make.core"/*.pp ".metadata/.plugins/org.eclipse.cdt.make.core"/*.mrc ".metadata/.plugins/org.eclipse.cdt.make.core"/*.prn
	@echo.
	@echo 'Clean complete.'

post-build:
	@echo.
	-C:\Renesas\E2_STU~1\DEBUGC~1\RX\RX_CON~1.EXE SUGENTEM_N_52.abs SUGENTEM_N_52.x
	@echo.

.PHONY: all clean dependents $(LINKER_OUTPUT_OUTPUTS)
.SECONDARY: post-build

-include ../makefile.targets
