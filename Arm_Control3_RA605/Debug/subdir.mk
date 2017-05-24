################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Arm_Control3_7axis.cpp 

OBJS += \
./Arm_Control3_7axis.o 

CPP_DEPS += \
./Arm_Control3_7axis.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -fPIC -std=c++0x -std=c++11 -I/usr/local/gazebo-7/include/gazebo-7 -I/home/kevin/research/gazebo/msgs/include -I/usr/local/sdformat/include/sdformat-4.2 -I/usr/local/ignition/include/ignition/math2 -I/usr/local/pcl-1-8/include/pcl-1.8 -I/usr/local/ogre-1-9/include/OGRE -I/usr/local/kdl/include -I/usr/include/eigen3 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


