cmake_minimum_required(VERSION 3.10)
project(LidarApp)

# C++のバージョンを指定
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Livox-SDK2のインクルードディレクトリを指定
include_directories(/usr/local/include)

# Livox-SDK2のライブラリディレクトリを指定
link_directories(/usr/local/lib)

# プラットフォームに応じたツールチェーンの設定
execute_process(COMMAND uname -m
        OUTPUT_VARIABLE DEVICE_SYSTEM_ID
        OUTPUT_STRIP_TRAILING_WHITESPACE)

if (DEVICE_SYSTEM_ID MATCHES x86_64)
    set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_x86_64=1)
elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
    set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_aarch64=1)
else ()
    message(FATAL_ERROR "FATAL: Please confirm your platform.")
endif ()

# 実行可能ファイルのターゲットを作成
add_executable(LidarApp main.cpp)

# include ディレクトリをインクルードパスに追加
target_include_directories(LidarApp PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_include_directories(LidarApp PRIVATE ../include/Payload-SDK-3.8.1/samples/sample_c++/platform/linux/manifold2/application)

# ソースファイルを収集
file(GLOB MODULE_APP_SRC application/*.cpp)

# PSDKライブラリをリンク
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/Payload-SDK-3.8.1/psdk_lib/lib/${TOOLCHAIN_NAME})
target_link_libraries(LidarApp 
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/Payload-SDK-3.8.1/psdk_lib/lib/${TOOLCHAIN_NAME}/libpayloadsdk.a
    stdc++
)

# Livox-SDK2ライブラリをリンク
target_link_libraries(LidarApp livox_lidar_sdk_shared)

# プラットフォームに応じたライブラリリンク（例: OpenCV, FFMPEG など）
# ここでは必要に応じて追加のライブラリをリンクすることができます
