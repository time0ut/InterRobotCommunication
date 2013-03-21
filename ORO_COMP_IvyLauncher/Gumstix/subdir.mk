################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../IvyLauncher.cpp 

OBJS += \
./IvyLauncher.o 

CPP_DEPS += \
./IvyLauncher.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-g++ -DOROCOS_TARGET=xenomai -DOCL_DLL_EXPORT -I/home/dmia/InterRobotCommunication/gumstix/include -I/opt/arm-eabi/boost-arm-eabi/include -I/opt/arm-eabi/xeno-arm-eabi/usr/xenomai/include -I/opt/arm-eabi/oro-arm-corba-eabi_v2.4/include -I/opt/arm-eabi/oro-arm-corba-eabi_v2.4/include/orocos -O3 -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


