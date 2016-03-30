cmake_minimum_required(VERSION 2.8.3)
project(vr_track_alvar)

set(MSG_DEPS
    std_msgs
    sensor_msgs
    geometry_msgs
    gazebo_msgs
    visualization_msgs)

find_package(catkin COMPONENTS
        genmsg
        roscpp
        gazebo_ros
        tf
        tf2
        image_transport
        resource_retriever
        cv_bridge
        pcl_ros
        pcl_conversions
        message_generation
        ${MSG_DEPS}
        dynamic_reconfigure
        cmake_modules
        REQUIRED)
find_package(ar_track_alvar REQUIRED)
find_package(TinyXML REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vr_track_alvar
  DEPENDS
  CATKIN_DEPENDS
        std_msgs
        roscpp
        tf
        tf2
        gazebo_ros
        ar_track_alvar
        message_runtime
        image_transport
        sensor_msgs
        geometry_msgs
        visualization_msgs
        gazebo_msgs
        resource_retriever
        cv_bridge
        pcl_ros
        pcl_conversions
)

include_directories(include 
        ${catkin_INCLUDE_DIRS}
        ${ar_track_alvar_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${TinyXML_INCLUDE_DIRS}

)

set(GENCPP_DEPS gazebo_msgs_gencpp std_msgs_gencpp sensor_msgs_gencpp geometry_msgs_gencpp visualization_msgs_gencpp)

add_executable(individual nodes/vrnode.cpp)
target_link_libraries(individual ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(individual ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})

install(TARGETS ${VR_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
