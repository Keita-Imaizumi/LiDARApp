cmake_minimum_required(VERSION 3.10)
project(LidarApp)

# C++のバージョンを指定
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Livox-SDK2のインクルードディレクトリを指定
include_directories(/usr/local/include)

# Livox-SDK2のライブラリディレクトリを指定
link_directories(/usr/local/lib)

# 実行可能ファイルのターゲットを作成
add_executable(LidarApp main.cpp)



execute_process(COMMAND uname -m
        OUTPUT_VARIABLE DEVICE_SYSTEM_ID)

if (DEVICE_SYSTEM_ID MATCHES x86_64)
    set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_x86_64=1)
elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
    set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_aarch64=1)
else ()
    message(FATAL_ERROR "FATAL: Please confirm your platform.")
endif ()

# Livox-SDK2ライブラリをリンク
target_link_libraries(LidarApp livox_lidar_sdk_shared)

# include ディレクトリをインクルードパスに追加
target_include_directories(LidarApp  PRIVATE ${PROJECT_SOURCE_DIR}/include)
link_libraries(${CMAKE_CURRENT_LIST_DIR}/../include/Payload-SDK-3.8.1/psdk_lib/lib/${TOOLCHAIN_NAME}/libpayloadsdk.a -lstdc++)

