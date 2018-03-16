cmake_minimum_required(VERSION 3.3)
project(PIRVS-SDK-SAMPLE)

set(CMAKE_C_FLAGS "-std=c99")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")
set(CMAKE_BUILD_TYPE Release)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

### 3rd party ###
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBS})

# LIBUSB
find_package(PkgConfig)
pkg_check_modules(LIBUSB libusb-1.0)
include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
include_directories(${GTK2_INCLUDE_DIRS})
link_directories(${GTK2_INCLUDE_DIRS})
# Add other flags to the compiler
add_definitions(${GTK2_CFLAGS_OTHER})

set(EXT_LIBS
    ${LIBUSB_LIBRARIES}
    ${GTK2_LIBRARIES}
    -lpthread
    -lpng
    -lz
    -ldl
    )

### Sample apps ###
# Set library and headers.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Build the sample apps.
set(APPS
    "apps/viewer.cpp"
    "apps/features.cpp"
    "apps/slam.cpp"
)

foreach(app ${APPS})
    get_filename_component(EXE_NAME ${app} NAME_WE)
    add_executable(${EXE_NAME} ${app})
    target_link_libraries(${EXE_NAME} -lPerceptInPIRVS ${EXT_LIBS})
endforeach(app)

### Sample shared library ###
# An example to show how to build a shared library with the PerceptInPIRVS SDK
add_library(shared_lib SHARED "apps/shared_lib.cpp")
target_link_libraries(shared_lib
    -lPerceptInPIRVS
    ${EXT_LIBS})

add_executable(viewer_lib apps/viewer_lib.cpp)
target_link_libraries(viewer_lib shared_lib ${EXT_LIBS})
