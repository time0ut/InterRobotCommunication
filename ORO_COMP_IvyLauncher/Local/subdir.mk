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
	g++ -DOROCOS_TARGET=gnulinux -DOCL_DLL_EXPORT -I"/home/dmia/workspace/workspaceEclipse/ORO_TYPE_base-types" -I/home/dmia/InterRobotCommunication/local/include -I/opt/orocos-toolchain/install/include/orocos -I/opt/orocos-toolchain/install/include -O3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


