cmake_minimum_required(VERSION 3.16)
project(quadcopter_pid VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(Eigen)

add_executable(quadcopter_pid
    src/main.cpp
    src/quadcopter.cpp
    src/pid_controller.cpp
    src/cascade_controller.cpp
    src/scenario.cpp
    src/csv_writer.cpp
)

target_include_directories(quadcopter_pid PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(quadcopter_pid PRIVATE Eigen3::Eigen)

if(MSVC)
    target_compile_options(quadcopter_pid PRIVATE /W4 /O2)
else()
    target_compile_options(quadcopter_pid PRIVATE -Wall -Wextra -O2)
endif()
