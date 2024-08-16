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

# Livox-SDK2ライブラリをリンク
target_link_libraries(LidarApp livox_lidar_sdk_shared)
