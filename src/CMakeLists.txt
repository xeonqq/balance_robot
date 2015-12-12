
set(ARDUINO_DEFAULT_BOARD leonardo) # Default Board ID, when not specified
set(ARDUINO_DEFAULT_PORT /dev/ttyACM0) # Default Port, when not specified

generate_arduino_firmware(balance_robot
    SKETCH balance_robot.ino
    PORT  /dev/ttyACM0
    SERIAL picocom @SERIAL_PORT@
    BOARD ${ARDUINO_DEFAULT_BOARD})
