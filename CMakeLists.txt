cmake_minimum_required(VERSION 2.8)
project(calibCamLiDAR)

add_definitions(-w)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Eigen
include_directories("/usr/include/eigen3")
# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
# OpenCV
find_package(OpenCV 3 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
# pcl
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(readBin ./test/readPointCloud.cpp)
target_link_libraries(readBin ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable(readImg ./test/readImg.cpp)
target_link_libraries(readImg ${OpenCV_LIBS} )

add_executable(writeYAML ./test/writeYAML.cpp)
target_link_libraries(writeYAML ${OpenCV_LIBS} )

add_executable(fusion ./test/fusion.cpp)
target_link_libraries(fusion ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(calib
	./src/onlineCalib.cpp
	./src/imgHandler.cpp
	./src/pcHandler.cpp
	./src/calibHandler.cpp
)

target_link_libraries(calib ${OpenCV_LIBS} ${PCL_LIBRARIES}) 