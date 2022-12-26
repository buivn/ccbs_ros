cmake_minimum_required(VERSION 3.0.2)
project(cs685)

find_package(catkin REQUIRED
    COMPONENTS
        # actionlib
        actionlib_msgs
        base_local_planner
        clear_costmap_recovery
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        message_generation
        move_base_msgs
        nav_core
        nav_msgs
        # cs685_navfn
        pluginlib
        roscpp
        rospy
        rotate_recovery
        std_srvs
        tf2_geometry_msgs
        tf2_ros
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/cs685.cfg
)

add_service_files(
  FILES
  GetMultiPaths.srv
)

add_action_files(
  DIRECTORY action
  FILES move_control.action
)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs nav_msgs)

catkin_package(

    INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES local_controller
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        move_base_msgs
        nav_msgs
        roscpp
        message_runtime
        actionlib_msgs
)



include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)


# cs685
# add_library(cs685 src/cs685.cpp )
# target_link_libraries(cs685 ${Boost_LIBRARIES} ${catkin_LIBRARIES} )
# add_dependencies(cs685 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(cs685_node src/cs685_node.cpp )
# add_dependencies(cs685_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(cs685_node cs685)
# set_target_properties(cs685_node PROPERTIES OUTPUT_NAME cs685)


add_library(local_controller src/local_controller.cpp )
target_link_libraries(local_controller ${Boost_LIBRARIES} ${catkin_LIBRARIES} )
add_dependencies(local_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(tinyxml2_2 src/tinyxml2.cpp )
target_link_libraries(tinyxml2_2 ${Boost_LIBRARIES} ${catkin_LIBRARIES} )
add_dependencies(tinyxml2_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(local_controller_node src/local_controller_node.cpp )
add_dependencies(local_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_controller_node local_controller)
set_target_properties(local_controller_node PROPERTIES OUTPUT_NAME local_controller)


add_executable(move_control_client src/move_control_Client.cpp )
add_dependencies(move_control_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_control_client ${catkin_LIBRARIES})
# set_target_properties(move_control_client PROPERTIES OUTPUT_NAME move_control_client)

add_executable(reading_paths src/reading_paths.cpp )
add_dependencies(reading_paths ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reading_paths ${catkin_LIBRARIES} tinyxml2_2)


install(
    TARGETS
        local_controller
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        local_controller_node
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
