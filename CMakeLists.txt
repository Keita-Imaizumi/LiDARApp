cmake_minimum_required(VERSION 3.10)
project(Lidar_app)

set(CMAKE_CXX_STANDARD 11)

include_directories(/usr/local/include/Livox-SDK/include)

# Livox-SDK2のライブラリディレクトリを指定
link_directories(/usr/local/lib)

# 実行可能ファイルのターゲットを作成
add_executable(Lidar_app main.cpp)

# Livox-SDK2ライブラリをリンク
target_link_libraries(Lidar_app livox_sdk2)
