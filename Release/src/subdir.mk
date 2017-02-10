################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Agent.cpp \
../src/Pathfinder.cpp \
../src/Tree.cpp \
../src/World.cpp \
../src/WorldObject.cpp 

OBJS += \
./src/Agent.o \
./src/Pathfinder.o \
./src/Tree.o \
./src/World.o \
./src/WorldObject.o 

CPP_DEPS += \
./src/Agent.d \
./src/Pathfinder.d \
./src/Tree.d \
./src/World.d \
./src/WorldObject.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++1y -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


