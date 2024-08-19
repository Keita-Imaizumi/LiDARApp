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

file(GLOB_RECURSE MODULE_SAMPLE_SRC 
        ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/module_sample/liveview/*.c*
        ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/module_sample/camera_manager/*.c*
        ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/module_sample/mobile/*.c*
        ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/module_sample/mop_channel/*.c*
        ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/module_sample/perception/*.c*
        ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/module_sample/gimbal/*.c*
        ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/module_sample/flight_controller/*.c*
        ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/module_sample/hms_manager/*.c*
        ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c/module_sample/*.c
        )
file(GLOB_RECURSE MODULE_COMMON_SRC ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/platform/linux/common/*.c*)
file(GLOB_RECURSE MODULE_HAL_SRC ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/platform/linux/manifold2/hal/*.c)
file(GLOB_RECURSE MODULE_APP_SRC ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/platform/linux/manifold2/application/*.c)

# 実行可能ファイルのターゲットを作成
add_executable(LidarApp 
    main.cpp
    ${MODULE_APP_SRC}
    ${MODULE_SAMPLE_SRC}
    ${MODULE_COMMON_SRC}
    ${MODULE_HAL_SRC}
)

# include ディレクトリをインクルードパスに追加
target_include_directories(LidarApp PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/platform/linux/manifold2/application
    ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/platform/linux/common/osal
    ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/platform/linux/manifold2/hal
    ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/platform/linux/manifold2
    ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c/module_sample
    ${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/samples/sample_c++/module_sample # 追加
)

# PSDKライブラリのパスを変数に保存
set(PSDK_LIB_PATH "${CMAKE_SOURCE_DIR}/include/Payload-SDK-3.8.1/psdk_lib/lib/${TOOLCHAIN_NAME}/libpayloadsdk.a")

# PSDKライブラリが存在するか確認
if(NOT EXISTS ${PSDK_LIB_PATH})
    message(FATAL_ERROR "PSDK library file not found: ${PSDK_LIB_PATH}")
endif()

# PSDKライブラリをリンク
target_link_libraries(LidarApp 
    ${PSDK_LIB_PATH}
    -lstdc++
    livox_lidar_sdk_shared
    pthread
)
