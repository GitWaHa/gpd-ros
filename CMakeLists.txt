cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(gpd-ros)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)

#set(CMAKE_CXX_FLAGS "-fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++17")
set(CMAKE_CXX_FLAGS "-O3 -march=native -mtune=intel -msse4.2 -mavx2 -mfma -flto -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++17")

find_package(
  catkin REQUIRED
  COMPONENTS
  roscpp
  sensor_msgs
  tf
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  grasp_detector # grasp_detect
)

# Eigen library
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# PCL
find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV REQUIRED)

message("start Caffe_FOUND")
option(USE_OPENVINO "use classifier based on OpenVINO toolkit" OFF)
option(USE_CAFFE "use classifier based on Caffe framework" ON)
option(USE_OPENCV "use classifier based on OpenCV framework" OFF)
# if(USE_CAFFE)
message("USE_CAFFE")
find_package(Caffe REQUIRED)
if (NOT Caffe_FOUND)
  message(FATAL_ERROR "Please install Caffe https://caffe.berkeleyvision.org/installation.html")
endif()
include_directories(
  /home/waha/Workspace/Git/caffe/include
  /home/waha/Workspace/Git/caffe/build/include
)
add_definitions(${Caffe_DEFINITIONS})
add_definitions(-DUSE_CAFFE)
# set(classifier_src src/${PROJECT_NAME}/net/classifier.cpp src/${PROJECT_NAME}/net/caffe_classifier.cpp)
# set(classifier_dep ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES})
# endif()
message("end Caffe_FOUND")

catkin_package(
  # INCLUDE_DIRS include
#  LIBRARIES grasp_detect
 CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


aux_source_directory(./src/gpd/candidate candidate_src)
aux_source_directory(./src/gpd/util util_src)
aux_source_directory(./src/gpd/net net_src)
aux_source_directory(./src/gpd/descriptor descriptor_src)


## Declare C++ executables
add_executable(gpd_detect_grasps src/detect_grasps.cpp

src/gpd/grasp_detector.cpp
src/gpd/clustering.cpp

${candidate_src}
${util_src}
${net_src}
${descriptor_src}
)

target_link_libraries(gpd_detect_grasps
${PCL_LIBRARIES}
${EIGEN_LIBRARIES}
${OpenCV_LIBS}
${Caffe_LIBRARIES}
)

add_executable(gpd_detect_grasps_server src/detect_grasps_server.cpp

src/gpd/grasp_detector.cpp
src/gpd/clustering.cpp

${candidate_src}
${util_src}
${net_src}
${descriptor_src}
)

target_link_libraries(gpd_detect_grasps_server
${PCL_LIBRARIES}
${EIGEN_LIBRARIES}
${OpenCV_LIBS}
${catkin_LIBRARIES}
${Caffe_LIBRARIES}
)

