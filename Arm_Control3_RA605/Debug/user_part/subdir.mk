################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../user_part/robot_position_controll.cpp \
../user_part/socketmain.cpp 

OBJS += \
./user_part/robot_position_controll.o \
./user_part/socketmain.o 

CPP_DEPS += \
./user_part/robot_position_controll.d \
./user_part/socketmain.d 


# Each subdirectory must supply rules for building sources it contributes
user_part/%.o: ../user_part/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -fPIC -std=c++0x -std=c++11 -I/usr/local/gazebo-7/include/gazebo-7 -I/home/kevin/research/gazebo/msgs/include -I/usr/local/sdformat/include/sdformat-4.2 -I/usr/local/ignition/include/ignition/math2 -I/usr/local/pcl-1-8/include/pcl-1.8 -I/usr/local/ogre-1-9/include/OGRE -I/usr/local/kdl/include -I/usr/include/eigen3 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


