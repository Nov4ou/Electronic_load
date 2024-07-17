################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
user/main.obj: ../user/main.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccstheia140/ccs/tools/compiler/ti-cgt-c2000_22.6.1A23259/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="/Users/nov4ou/workspace_v12/Problems/Electronic_load" --include_path="/Users/nov4ou/workspace_v12/f2806x/headers/include" --include_path="/Users/nov4ou/workspace_v12/f2806x/common/include" --include_path="/Applications/ti/ccstheia140/ccs/tools/compiler/ti-cgt-c2000_22.6.1A23259/include" --include_path="/Users/nov4ou/workspace_v12/Problems/Electronic_load/app/timer" --include_path="/Users/nov4ou/workspace_v12/Problems/Electronic_load/app/epwm" --include_path="/Users/nov4ou/workspace_v12/Problems/Electronic_load/app/adc" --include_path="/Users/nov4ou/workspace_v12/Problems/Electronic_load/app/key" --include_path="/Applications/ti/solar/v1.2/float/include" --include_path="/Users/nov4ou/workspace_v12/Problems/Electronic_load/app/oled" --include_path="/Users/nov4ou/workspace_v12/Problems/Electronic_load/app/spi" -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="user/$(basename $(<F)).d_raw" --obj_directory="user" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


