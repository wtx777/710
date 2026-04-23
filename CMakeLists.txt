cmake_minimum_required(VERSION 3.14)
project(SensorModels LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(sensor_models SHARED
    RadarModel.cpp
    EsmModel.cpp
    EoModel.cpp
)

target_include_directories(sensor_models
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

if(WIN32)
    target_compile_definitions(sensor_models PRIVATE BUILDING_SENSOR_MODELS_DLL)
else()
    set_target_properties(sensor_models PROPERTIES CXX_VISIBILITY_PRESET hidden)
    set_target_properties(sensor_models PROPERTIES VISIBILITY_INLINES_HIDDEN ON)
endif()

# Windows：与 MinGW/MSVC 均可生成 DLL；Linux/macOS：生成 .so/.dylib
if(MSVC)
    target_compile_options(sensor_models PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(sensor_models PRIVATE -Wall -Wextra -Wpedantic)
endif()

option(SENSOR_MODELS_INSTALL "Install headers and library" OFF)
if(SENSOR_MODELS_INSTALL)
    install(TARGETS sensor_models
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
    install(FILES SensorModels.h DESTINATION include)
endif()

# 可执行试用程序：与 DLL 同目录运行（MSVC 多配置下在 build/<Config>/）
add_executable(demo_sensor_models demo_sensor_models.cpp)
target_link_libraries(demo_sensor_models PRIVATE sensor_models)
if(MSVC)
    target_compile_options(demo_sensor_models PRIVATE /utf-8)
endif()
