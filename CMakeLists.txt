cmake_minimum_required(VERSION 3.16)

project(Try3DLibs VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

find_package(PCL 1.15 REQUIRED COMPONENTS common io filters visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

file(GLOB_RECURSE PROJECT_SOURCES 
    src/*.cpp
    src/*.h
)

qt_add_executable(${PROJECT_NAME} MACOSX_BUNDLE ${PROJECT_SOURCES})

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Widgets
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${VTK_LIBRARIES}
)