cmake_minimum_required(VERSION 3.0)
project(rrt)

find_package(OpenCV REQUIRED)

set( my_includes "./headers/preprocess.h" "./headers/rrt.h")
set( my_cpps "./src/main.cpp" "./src/preprocess.cpp" "./src/rrt.cpp")

include_directories(${OpenCV_INCLUDE_DIRS}
                    "./headers")

add_executable(rrt ${my_includes} ${my_cpps})

# target_include_directories(rrt PRIVATE ./utils/headers ./utils)

target_link_libraries(rrt ${OpenCV_LIBS})
