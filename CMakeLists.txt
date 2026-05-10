cmake_minimum_required(VERSION 3.15)
project(ModularVIO VERSION 1.0 LANGUAGES CXX)

# Enforce C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enforce strict warnings to catch sloppy C++ early
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror=return-type")

# ---------------------------------------------------------
# TODO 1: Find OpenCV and Eigen3 packages here.
# Hint: Use the `find_package(...)` command. 
# Eigen might require the `REQUIRED` keyword.
# ---------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Define our main executable target
add_executable(vio_app 
    src/main.cpp
    src/dataset_reader.cpp
    src/frotend.cpp
)

# Tell the target where to look for our local header files
target_include_directories(vio_app PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# ---------------------------------------------------------
# TODO 2: Link OpenCV and Eigen to our target `vio_app`
# Hint: Use `target_link_libraries(...)`. 
# Modern Eigen is usually linked as `Eigen3::Eigen`.
# ---------------------------------------------------------
target_include_directories(vio_app PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(vio_app PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(vio_app PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
