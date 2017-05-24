################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../cloud_viewer.cpp 

OBJS += \
./cloud_viewer.o 

CPP_DEPS += \
./cloud_viewer.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -fPIC -std=c++11 -I/usr/local/pcl-1-8/include/pcl-1.8 -I/usr/include/eigen3 -I/usr/local/vtk-6-3/include/vtk-6.3 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


