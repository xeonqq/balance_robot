

set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake) # Arduino Toolchain


cmake_minimum_required(VERSION 2.8)
#====================================================================#
#  Setup Project                                                     #
#====================================================================#
project(BalanceRobot C CXX)

print_board_list()
print_programmer_list()

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/i2cdevlib/Arduino/I2Cdev)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/i2cdevlib/Arduino/MPU6050)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/PinChangeInt)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/EnableInterrupt)

add_subdirectory(src)   #add the example directory into build
