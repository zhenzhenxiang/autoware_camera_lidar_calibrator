cmake_minimum_required(VERSION 2.8.12)
project(autoware_camera_lidar_calibrator)

include(FindPkgConfig)

find_package(catkin REQUIRED COMPONENTS
    autoware_build_flags
    cv_bridge
    image_transport
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    autoware_msgs
    geometry_msgs
    tf
)
find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


catkin_package(CATKIN_DEPENDS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    pcl_conversions
    pcl_ros
    autoware_msgs
    geometry_msgs
    tf
)

set(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(lib)

#camera lidar node
add_executable(autoware_camera_lidar_calibration_node
        src/autoware_camera_lidar_calibration/camera_lidar_calibration_node.cpp
)
target_link_libraries(autoware_camera_lidar_calibration_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(autoware_camera_lidar_calibration_node ${catkin_EXPORTED_TARGETS})

# calibration_publisher
add_executable(calibration_publisher
        src/calibration_publisher/calibration_publisher.cpp
        )

add_dependencies(calibration_publisher
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(calibration_publisher
        ${catkin_LIBRARIES}
        )

# bag_creator
add_executable(bag_creator_from_image_and_ply
    src/bag_creator/bag_creator_from_image_and_ply.cpp
    )

add_dependencies(bag_creator_from_image_and_ply
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(bag_creator_from_image_and_ply
    ${catkin_LIBRARIES}
    )

add_executable(convert_from_txt_to_ply
    src/bag_creator/convert_from_txt_to_ply.cpp
    )

add_dependencies(convert_from_txt_to_ply
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(convert_from_txt_to_ply
    ${catkin_LIBRARIES}
    )

add_executable(query_local_clouds
    src/bag_creator/query_local_clouds.cpp
    )

add_dependencies(query_local_clouds
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(query_local_clouds
    ${catkin_LIBRARIES}
    ndt_omp
    )

add_executable(query_cloud_poses
    src/bag_creator/query_cloud_poses.cpp
    )

add_dependencies(query_cloud_poses
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(query_cloud_poses
    ${catkin_LIBRARIES}
    )

#camera intrinsics node
catkin_python_setup()

catkin_install_python(PROGRAMS
    nodes/cameracalibrator.py
    nodes/cameracheck.py
    scripts/tarfile_calibration.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

