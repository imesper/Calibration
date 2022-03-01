cmake_minimum_required(VERSION 3.16)

project(Calibration LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

link_directories(/Users/ianesper/Development/librealsense/build/Debug)
include_directories(/Users/ianesper/Development/librealsense/include)


find_package(Qt6 COMPONENTS Widgets REQUIRED)

find_package(VTK REQUIRED)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

find_package(OpenMP)

if(ANDROID)
  add_library(Calibration SHARED
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
  )
else()
  add_executable(Calibration
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    helper.h
    helper.cpp
  )
endif()
#if(OpenMP_CXX_FOUND)
#    target_link_libraries(Calibration  OpenMP::OpenMP_CXX  Qt6::Widgets ${OpenCV_LIBS} ${PCL_LIBRARIES}
#        ${VTK_LIBRARIES} realsense2 )
#else()
target_link_libraries(Calibration PRIVATE Qt6::Widgets ${OpenCV_LIBS} ${PCL_LIBRARIES}
    ${VTK_LIBRARIES} realsense2)
#endif()
