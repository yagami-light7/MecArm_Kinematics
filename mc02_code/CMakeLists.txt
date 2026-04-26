cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME mec_arm_impedance_control)

# Use the bundled ARM bare-metal toolchain by default when the caller did
# not provide one explicitly. This must be set before the first project().
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake" CACHE FILEPATH "ARM GCC toolchain")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# 查找 Eigen3 包
find_package(Eigen3 REQUIRED)

# 添加 Eigen 头文件路径
include_directories(${EIGEN3_INCLUDE_DIRS})

## 使用 GLOB_RECURSE 收集所有 C 文件
#file(GLOB_RECURSE SOURCES
#        "${SOURCE_DIR}/*.c"  # 收集 src 目录下的所有 C 文件
#        "${SOURCE_DIR}/*.h"  # 收集 src 目录下的所有 C 文件
#)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here


        # 在此添加c文件
        #        Users/Bsp/Bsp_Board/Board_ADC/Board_ADC.c
#        Users/Bsp/Bsp_Board/Board_Buzzer/Board_Buzzer.c
#        Users/Bsp/Bsp_Board/Board_DWT/Board_DWT.c
#        Users/Bsp/Bsp_Board/Board_CRC/Board_CRC32.c
        Users/1-MWL/MWL_CRC/MWL_CRC.c
#        Users/Bsp/Bsp_Board/Board_Delay/Board_Delay.c
#        #        Users/Bsp/Bsp_Board/Board_Flash/Board_Flash.c
#        #        Users/Bsp/Bsp_Board/Board_I2C/Board_I2C.c
#        Users/Bsp/Bsp_Board/Board_IMU_PWM/Board_IMU_PWM.c
#        #        Users/Bsp/Bsp_Board/Board_Laser/Board_Laser.c
#        Users/Bsp/Bsp_Board/Board_LED/Board_LED.c
#        Users/Bsp/Bsp_Board/Board_Remote_Control/Board_RC.c
#        #        Users/Bsp/Bsp_Board/Board_RNG/Board_RNG.c
#        #        Users/Bsp/Bsp_Board/Board_Servo_PWM/Board_Servo_PWM.c
#        #        Users/Bsp/Bsp_Board/Board_SPI/Board_SPI.c
#        Users/Bsp/Bsp_Board/Board_USART/Board_USART.c
#
#        Users/Bsp/Bsp_Device/Dev_BMI088/Dev_BMI088_Reg.h
#        Users/Bsp/Bsp_Device/Dev_BMI088/Dev_BMI088_Driver.c
#        Users/Bsp/Bsp_Device/Dev_BMI088/Dev_BMI088_Driver.h
#        Users/Bsp/Bsp_Device/Dev_BMI088/Dev_BMI088_Middleware.h
#        Users/Bsp/Bsp_Device/Dev_BMI088/Dev_BMI088_Middleware.c
#        #        Users/Bsp/Bsp_Device/Dev_CV/Dev_CV.c
#        Users/Bsp/Bsp_Device/Dev_FDCAN/Dev_FDCAN.c
#        Users/Bsp/Bsp_Device/Dev_Remote_Control/Dev_Remote_Control.cpp
#        Users/Bsp/Bsp_Device/Dev_Referee/Dev_Referee.c
#        Users/Bsp/Bsp_Device/Dev_Custom/Dev_Custom.c
#
#
#        Users/Bsp/Bsp_Support/Sp_FIFO/Sp_FIFO.c
#        Users/Bsp/Bsp_Support/Sp_MEM_MANG/Sp_MEM_MANG.c
#
#        Users/Algorithm/Alg_ToolBox/Alg_DH_Kinematics/Alg_DH_Kinematics.cpp
#        Users/Algorithm/Alg_Support/Alg_UserLib.c
#        Users/Algorithm/Alg_PID/Alg_PID.c
#        Users/Algorithm/Alg_Filter/Alg_Filter.c
#        Users/Algorithm/Alg_EKF/EKF/QuaternionEKF.c
#        Users/Algorithm/Alg_EKF/kalman/kalman_filter.c
#        Users/Algorithm/Alg_ToolBox/Alg_Utilities/Alg_Utilities.cpp
#        Users/Algorithm/Alg_ToolBox/Alg_Matrix/Alg_Matrix.cpp
#
#
#        Users/Application/App_Buzzer/App_Buzzer_Task.c
#        Users/Application/App_Calibrate/App_Calibrate_Task.c
#        Users/Application/App_Custom/App_Custom_Task.c
#        Users/Application/App_Chassis/App_Chassis_Task.cpp
#        Users/Application/App_Chassis/App_Chassis_Behaviour.cpp
#        Users/Application/App_CV_Control/App_CV_Control.c
#        Users/0-APL/APL_RC_Hub/APL_RC_Hub.cpp
#        Users/Application/App_Gimbal/App_Gimbal_Task.cpp
#        Users/Application/App_Gimbal/App_Gimbal_Behaviour.cpp
#        Users/Application/App_Detect/App_Detect_Task.c
#        #        Users/Application/App_IMU/AHRS.lib
#        Users/Application/App_IMU/App_INS_Task.c
#        #        Users/Application/App_IMU/App_AHRS_Middleware.c
#        Users/Application/App_Mechanical_Arm/App_Mechanical_Arm_Task.cpp
#        Users/Application/App_Mechanical_Arm/App_Mechanical_Arm_Behaviour.cpp
#        #        Users/Application/App_CV_Control/App_CV_Control.c
#        #        Users/Application/App_Battery/App_Voltage_Task.c
#        Users/Application/App_IMU/App_INS_Task.c
#        Users/Application/App_Led/App_LED_RGB_Flow_Task.c
#        Users/Application/App_Referee/App_Referee_Task.c
#        Users/Application/App_UI/App_UI_Task.c
#        Users/Application/App_UI/ui_g_Dynamic_0.c
#        Users/Application/App_UI/ui_g_Dynamic_1.c
#        Users/Application/App_UI/ui_g_Dynamic_2.c
#        Users/Application/App_UI/ui_g_Dynamic_3.c
#        Users/Application/App_UI/ui_g_Dynamic_4.c
#        Users/Application/App_UI/ui_g_Static_0.c
#        Users/Application/App_UI/ui_g_Static_1.c
#        Users/Application/App_UI/ui_g_Static_2.c
#        Users/Application/App_UI/ui_interface.c
#
#
#
#        Users/Upper_Communication/UC_Referee/UC_Referee.c
#        Users/Upper_Communication/UC_Referee/UC_Protocol.c

        Users/0-APL/CallBack_USART/CallBack_USART.cpp
        Users/0-APL/CallBack_FDCAN/CallBack_FDCAN.cpp
        Users/0-APL/APL_MecArm/APL_MecArm.cpp
        Users/0-APL/APL_RC_Hub/APL_RC_Hub.cpp
        Users/0-APL/APL_MecArm/gravity_trig_model.h

        Users/1-MWL/MWL_Kinematics/MWL_Kinematics.cpp
        Users/1-MWL/MWL_Data_Utils/MWL_Data_Utils.cpp
        Users/1-MWL/MWL_Motor/MWL_Motor.cpp
        Users/1-MWL/MWL_PID/MWL_PID.cpp
        Users/1-MWL/MWL_Matrix/MWL_Matrix.h
        Users/1-MWL/MWL_SO3/MWL_SO3.cpp
        Users/1-MWL/MWL_Trajectory/MWL_Trajectory.cpp

        Users/2-HDL/HDL_DR16/HDL_DR16.cpp
        Users/2-HDL/HDL_Video_Transmission/HDL_VT.cpp
        Users/2-HDL/HDL_Motor/HDL_MIT_Motor_Base/HDL_MIT_Motor_Base.cpp
        Users/2-HDL/HDL_Motor/HDL_DJI_Motor/HDL_DJI_Motor.cpp
        Users/2-HDL/HDL_Motor/HDL_Damiao_Motor/HDL_Damiao_Motor.cpp
        Users/2-HDL/HDL_Motor/HDL_RS_Motor/HDL_RS_Motor.cpp
        Users/2-HDL/HDL_Motor/HDL_Motor_Bus/HDL_Motor_Bus.cpp
        Users/2-HDL/HDL_Motor/HDL_Motor_Rx/HDL_Motor_Rx.cpp

        Users/3-HAL/HAL_USART/HAL_USART.cpp
        Users/3-HAL/HAL_FDCAN/HAL_FDCAN.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        User/application
        User/application/protocol
        User/bsp/boards
        User/controller
        User/devices
        User/support
        User/algorithm

        # 在此添加文件路径
        Users/Bsp/Bsp_Board/Board_ADC
        Users/Bsp/Bsp_Board/Board_Buzzer
        Users/Bsp/Bsp_Board/Board_CRC
        Users/Bsp/Bsp_Board/Board_DWT
        Users/Bsp/Bsp_Board/Board_Delay
        Users/Bsp/Bsp_Board/Board_Flash
        Users/Bsp/Bsp_Board/Board_I2C
        Users/Bsp/Bsp_Board/Board_IMU_PWM
        Users/Bsp/Bsp_Board/Board_LED
        Users/Bsp/Bsp_Board/Board_Remote_Control
        Users/Bsp/Bsp_Board/Board_RNG
        Users/Bsp/Bsp_Board/Board_Servo_PWM
        Users/Bsp/Bsp_Board/Board_SPI
        Users/Bsp/Bsp_Board/Board_USART


        Users/Bsp/Bsp_Device/Dev_BMI088
        Users/Bsp/Bsp_Device/Dev_CV
        Users/Bsp/Bsp_Device/Dev_Custom
        Users/Bsp/Bsp_Device/Dev_FDCAN
        Users/Bsp/Bsp_Device/Dev_IST8310
        Users/Bsp/Bsp_Device/Dev_Oled
        Users/Bsp/Bsp_Device/Dev_Remote_Control
        Users/Bsp/Bsp_Device/Dev_Referee
        Users/Bsp/Bsp_Device/Dev_Video_Transmission

        Users/Bsp/Bsp_Support/Sp_FIFO/
        Users/Bsp/Bsp_Support/Sp_MEM_MANG/

        Users/Algorithm/Alg_Support
        Users/Algorithm/Alg_PID
        Users/Algorithm/Alg_Filter
        Users/Algorithm/Alg_EKF/EKF
        Users/Algorithm/Alg_EKF/kalman
        Users/Algorithm/Alg_ToolBox/Alg_Utilities
        Users/Algorithm/Alg_ToolBox/Alg_Matrix

        Users/Application/App_Buzzer
        Users/Application/App_Calibrate
        Users/Application/App_Chassis
        Users/Application/App_Custom
        Users/0-APL/APL_RC_Hub
        Users/Application/App_Gimbal
        Users/Application/App_Detect
        Users/Application/App_CV_Control
        Users/Application/App_Battery
        Users/Application/App_IMU
        Users/Application/App_Mechanical_Arm
        Users/Application/App_Led
        Users/Application/App_Operator
        Users/Application/App_Referee
        Users/Application/App_UI

        Users/Upper_Communication/UC_Referee

        Users/0-APL/CallBack_USART/
        Users/0-APL/CallBack_FDCAN/
        Users/0-APL/APL_MecArm/
        Users/0-APL/APL_USART/
        Users/0-APL/APL_RC_Hub/

        Users/1-MWL/MWL_Kinematics/
        Users/1-MWL/MWL_Data_Utils/
        Users/1-MWL/MWL_CRC/
        Users/1-MWL/MWL_Motor/
        Users/1-MWL/MWL_PID/
        Users/1-MWL/MWL_Matrix
        Users/1-MWL/MWL_SO3
        Users/1-MWL/MWL_Trajectory

        Users/2-HDL/HDL_DR16
        Users/2-HDL/HDL_Video_Transmission
        Users/2-HDL/HDL_Motor/HDL_MIT_Motor_Base
        Users/2-HDL/HDL_Motor/HDL_DJI_Motor
        Users/2-HDL/HDL_Motor/HDL_Damiao_Motor
        Users/2-HDL/HDL_Motor/HDL_RS_Motor
        Users/2-HDL/HDL_Motor/HDL_Motor_Device_Base
        Users/2-HDL/HDL_Motor/HDL_Motor_Bus
        Users/2-HDL/HDL_Motor/HDL_Motor_Rx

        Users/3-HAL/HAL_USART
        Users/3-HAL/HAL_FDCAN

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
        #添加全局宏定义 比如打开FPU浮点运算单元等
        ARM_MATH_CM7,__FPU_USED=1U,__FPU_PRESENT=1U,__CC_ARM,ARM_MATH_MATR*IX_CHECK,ARM_MATH_ROUNDING
)

# 若要添加静态链接库 需要以下下四行代码
add_library(AHRS INTERFACE)

target_link_directories(AHRS INTERFACE
        Users/Application/App_IMU/
)

target_link_libraries(AHRS INTERFACE
        :AHRS.lib
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        # Add user defined libraries
        AHRS
)
