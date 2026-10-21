cmake_minimum_required(VERSION 3.16)
project(DroneFlightSimulator LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Embedded Drone System")
add_library(drone_core
  "${SOURCE_DIR}/DynamicPathPlanning.cpp" "${SOURCE_DIR}/Physics.cpp"
  "${SOURCE_DIR}/Sensor.cpp" "${SOURCE_DIR}/PIDController.cpp"
  "${SOURCE_DIR}/PredictiveMaintenance.cpp" "${SOURCE_DIR}/RTOS.cpp"
  "${SOURCE_DIR}/Drone.cpp" "${SOURCE_DIR}/Statistics.cpp"
  "${SOURCE_DIR}/EnhancedCommunication.cpp")
target_include_directories(drone_core PUBLIC "${SOURCE_DIR}")
if(MSVC)
  target_compile_options(drone_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(drone_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_executable(drone_sim "${SOURCE_DIR}/Main.cpp")
target_link_libraries(drone_sim PRIVATE drone_core)
add_executable(sim_tests tests/test_main.cpp)
target_link_libraries(sim_tests PRIVATE drone_core)
enable_testing()
add_test(NAME simulation_tests COMMAND sim_tests)
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  add_test(NAME cli_tests COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli.py" "$<TARGET_FILE:drone_sim>")
endif()
