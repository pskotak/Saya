cmake_minimum_required(VERSION 3.11)

project(saya)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

find_package( OpenCV REQUIRED )
set(DEPENDENCIES ${DEPENDENCIES} realsense2)

set(SOURCE_FILES
    vision/vision.cpp
    main.cpp    
)

add_executable(saya ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(saya ${DEPENDENCIES} ${OpenCV_LIBS})

install(TARGETS saya RUNTIME DESTINATION bin)
