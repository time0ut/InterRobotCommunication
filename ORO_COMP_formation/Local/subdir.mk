################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../formation.cpp 

OBJS += \
./formation.o 

CPP_DEPS += \
./formation.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -DOROCOS_TARGET=gnulinux -DOCL_DLL_EXPORT -I"/home/dmia/workspace/ORO_TYPE_base-types" -I/home/dmia/InterRobotCommunication/local/include -I/opt/orocos-toolchain/install/include/orocos -I/opt/orocos-toolchain/install/include -I"/home/dmia/workspace/ORO_component_tools" -O3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


