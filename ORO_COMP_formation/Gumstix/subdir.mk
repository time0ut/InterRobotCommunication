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
	arm-linux-g++ -DOROCOS_TARGET=xenomai -DOCL_DLL_EXPORT -I"/home/dmia/workspace/workspaceEclipse/ORO_TYPE_base-types" -I/home/dmia/InterRobotCommunication/gumstix/include -I/opt/arm-eabi/mrpt-arm-eabi/libs/base/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/detectors/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/hmtslam/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/maps/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/mrpt-config -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/obs/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/reactivenav/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/topography/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/vision/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/scanmatching/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/gui/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/base/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/bayes/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/hwdrivers/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/maps/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/slam/include -I/opt/arm-eabi/mrpt-arm-eabi/include/mrpt/opengl/include -I/opt/arm-eabi/boost-arm-eabi/include -I/opt/arm-eabi/xeno-arm-eabi/usr/xenomai/include -I/opt/arm-eabi/oro-arm-corba-eabi_v2.4/include -I/opt/arm-eabi/oro-arm-corba-eabi_v2.4/include/orocos -O3 -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


