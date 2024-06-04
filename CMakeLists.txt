cmake_minimum_required(VERSION 2.8.3)

project(gluon)

add_compile_options(-std=c++14)

if( "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
	set(BUILD_PLATFORM
		linux_x86_64)
else()
	set(BUILD_PLATFORM
		windows_x64)
endif()


set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  message_generation
  roslib
  tf_conversions
  PCL
  pcl_ros
  tf2
  tf2_ros
)


include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/include
	${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}
  /home/s/Desktop/libfreenect2/include
  /usr/include/opencv4
)

generate_messages(
		DEPENDENCIES
		std_msgs
		geometry_msgs
    
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs actionlib moveit_msgs
)

find_package(roslaunch)
find_package(orocos_kdl)
find_package(kdl_parser REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(actionlib REQUIRED)
find_package(moveit_msgs REQUIRED)


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/s
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_library(rate
  include/rate.h
  src/rate.cpp
  )
add_dependencies(rate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

            ## Specify libraries to link a library or executable target against
target_link_libraries(rate 
  ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/lib/${BUILD_PLATFORM}/libActuatorController.so
  
  )

add_library(gravity
  include/kinematics_dynamics.h
  src/kinematics_dynamics.cpp
  )
add_dependencies(gravity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

            ## Specify libraries to link a library or executable target against
target_link_libraries(gravity 
  ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/lib/${BUILD_PLATFORM}/libActuatorController.so
  rate
  orocos-kdl
  kdl_parser
  )

add_library(can_interface
  include/can_interface.h
  src/can_interface.cpp
  )
add_dependencies(can_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

            ## Specify libraries to link a library or executable target against
target_link_libraries(can_interface
  ${catkin_LIBRARIES}
  rate
  )

add_library(gluon_interface
  include/gluon_interface.h
  src/gluon_interface.cpp
  )
add_dependencies(gluon_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

            ## Specify libraries to link a library or executable target against
target_link_libraries(gluon_interface
  ${catkin_LIBRARIES}
  rate
  ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/lib/${BUILD_PLATFORM}/libActuatorController.so
  )

  add_library(teaching
  include/teaching.h
  src/teaching.cpp
  )
add_dependencies(teaching ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

            ## Specify libraries to link a library or executable target against
target_link_libraries(teaching 
  ${catkin_LIBRARIES}
  
  )


add_executable(${PROJECT_NAME}_keynode src/gluonControl_withkey.cpp) 

add_dependencies(${PROJECT_NAME}_keynode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_keynode 
   ${catkin_LIBRARIES}
   
   can_interface
   gluon_interface
)


add_executable(${PROJECT_NAME}_gravityoff src/gravityoff.cpp) 

add_dependencies(${PROJECT_NAME}_gravityoff ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_gravityoff 
   ${catkin_LIBRARIES}
   ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/sdk/lib/${BUILD_PLATFORM}/libActuatorController.so
   gravity
   orocos-kdl
   kdl_parser
   teaching
)







add_executable(dual_bringup src/dual_bringup.cpp) 

add_dependencies(dual_bringup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(dual_bringup
   ${catkin_LIBRARIES}
  
   gluon_interface
)


add_executable(right_moveit_server src/right_moveit_server.cpp) 

add_dependencies(right_moveit_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(right_moveit_server 
  ${catkin_LIBRARIES}
  
  ${CMAKE_THREAD_LIBS_INIT}
  
  
)

add_executable(left_moveit_server src/left_moveit_server.cpp) 

add_dependencies(left_moveit_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(left_moveit_server 
  ${catkin_LIBRARIES}
  
  ${CMAKE_THREAD_LIBS_INIT}
  
  
)


add_executable(can_bringup src/can_bringup.cpp) 

add_dependencies(can_bringup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(can_bringup 
  ${catkin_LIBRARIES}
  
  ${CMAKE_THREAD_LIBS_INIT}
  can_interface
  
)


add_executable(dual_replay src/dual_replay.cpp) 

add_dependencies(dual_replay ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(dual_replay
   ${catkin_LIBRARIES}
   teaching
   gluon_interface
)



# catkin_install_python(PROGRAMS
#   scripts/take_photo.py
#   scripts/image_process_py.py
#   scripts/moveitCommander.py
#   scripts/dual_moveitCommander.py
#   scripts/Camera_Calibration_Distort.py
#   scripts/calibrate.py
#   scripts/movearound.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )