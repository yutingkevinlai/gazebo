################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../depth_sensor_plugin.cpp 

OBJS += \
./depth_sensor_plugin.o 

CPP_DEPS += \
./depth_sensor_plugin.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -fPIC -std=c++11 -I/usr/local/include -I/usr/include/eigen3 -I/usr/local/gazebo-7/include/gazebo-7 -I/usr/include/c++/4.9 -I/home/kevin/research/gazebo/msgs/include -I/usr/local/ignition/include/ignition/math2 -I/usr/local/pcl-1-8/include/pcl-1.8 -I/usr/local/ogre-1-9/include/OGRE/Paging -I/usr/local/sdformat/include/sdformat-4.2 -I/usr/local/ogre-1-9/include -I/usr/local/ogre-1-9/include/OGRE -I/usr/local/opencv-2-4-10/include -I/usr/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


