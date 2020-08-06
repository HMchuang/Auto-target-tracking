project(HS_tracking)
 
cmake_minimum_required(VERSION 2.6)

# Specify C++ version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-pthread")

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

# Search libray with pkg-config and name them "GSTREAMER","TCAMLIB"
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
pkg_check_modules(TCAMLIB tcam)
pkg_check_modules(EIGEN eigen3)
pkg_check_modules(CUDA cuda-9.0)
pkg_check_modules(VW visionworks)

# Search all headfiles. (1)CMAKE_CURRENT_BINARY_DIR: the path of current CMakeLists.txt (2)*[packgename]_INCLUDE_DIRS
include_directories( ${CMAKE_CURRENT_BINARY_DIR} ./include  ${CMAKE_CURRENT_BINARY_DIR} ./mavlink_c_library_v1/common  ${GSTREAMER_INCLUDE_DIRS}  ${TCAMLIB_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS}/Eigen ${CUDA_INCLUDE_DIRS}  ${VW_INCLUDE_DIRS})

# Define logic switch
add_definitions(${GSTREAMER_CFLAGS_OTHER})  

add_executable(HS_tracking main.cpp tcamimage.cpp tcamcamera.cpp multithreaded_interface.cpp position_controller.cpp serial_port.cpp)
target_link_libraries(HS_tracking ${GSTREAMER_LIBRARIES} ${TCAMLIB_LIBRARIES} ${OpenCV_LIBS} ${VW_LIBRARIES})

# Install configuration for insatlling target:HS_tracking in /usr/local/bin 
install(TARGETS HS_tracking RUNTIME DESTINATION bin)
