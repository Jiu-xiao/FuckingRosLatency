cmake_minimum_required(VERSION 3.10)
project(libxr_tp_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 强制启用警告
add_definitions(-Wall -Werror)

add_compile_options(-g -O3)

## 导出编译命令以便使用 clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# libxr
set(LIBXR_LOG_OUTPUT_LEVEL 2)
add_subdirectory(libxr)

add_executable(${PROJECT_NAME} libxr_tp_test.cpp)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC xr
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)