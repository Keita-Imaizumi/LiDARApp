cmake_minimum_required(VERSION 3.10)
project(Lidar_app)

set(CMAKE_CXX_STANDARD 11)

add_executable(Lidar_app main.cpp)

add_subdirectory(include/Livox-SDK2)
add_subdirectory(include/Payload-SDK-3.8.1)

# includeディレクトリの指定
target_include_directories(Lidar_app PRIVATE include/Livox-SDK2 include/Payload-SDK-3.8.1)

# libAとlibBをリンク
target_link_libraries(Lidar_app PRIVATE Livox-SDK2 Payload-SDK-3.8.1)
