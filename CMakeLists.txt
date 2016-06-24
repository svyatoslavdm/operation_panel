cmake_minimum_required(VERSION 2.8.3)
project(operation_panel)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++0x") # Need C++11 


find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  moveit_core
  interactive_markers
  visualization_msgs
  tf
  object_recognition_msgs
  object_recognition_ros
  pluginlib
  rviz
  wsg_50_common
  wsg_50_driver
  schunk_egn160_driver
  genmsg
  actionlib_msgs
  actionlib
  geometry_msgs
  master_slave_msgs
  unscrew_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES operation_panel
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
find_package(catkin REQUIRED)
find_package(cmake_modules REQUIRED)

find_package(PkgConfig)
pkg_check_modules(PC_FLANN flann)
set(FLANN_DEFINITIONS ${PC_FLANN_CFLAGS_OTHER})

find_path(FLANN_INCLUDE_DIR flann/flann.hpp
    HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS})

find_library(FLANN_LIBRARY flann
    HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS})

set(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})
set(FLANN_LIBRARIES ${FLANN_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Flann DEFAULT_MSG
    FLANN_LIBRARY FLANN_INCLUDE_DIR)

mark_as_advanced(FLANN_LIBRARY FLANN_INCLUDE_DIR)



find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)


message("PCL_INCLUDE_DIRS:" "${PCL_INCLUDE_DIRS}")


find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

# set(SOURCE_FILES 
#    src/Servers/TFServer.cpp
# )

qt4_wrap_cpp(MOC_FILES
    include/operation_panel/operation_panel.h
    include/operation_panel/scan_panel.h
    include/operation_panel/open_bag_panel.h
    include/operation_panel/master_slave_panel.h
    include/operation_panel/unscrew_panel.h
    
    include/operation_panel/Servers/IServer.h
    include/operation_panel/Servers/TFServer.h
    include/operation_panel/Servers/MarkerServer.h
    include/operation_panel/Servers/CoordsServer.h
    
    include/operation_panel/GraspsFilters/CollisionFilter.h
    include/operation_panel/GraspsFilters/IGraspsFilter.h
    include/operation_panel/GraspsFilters/ReachAbilityFilter.h
    include/operation_panel/GraspsFilters/ValidStateFilter.h
    
    include/operation_panel/Readers/IGraspReader.h
    include/operation_panel/Readers/FileReader.h 
    include/operation_panel/Readers/DBReader.h 

    include/operation_panel/GripperControllers/IGripperController.h
    include/operation_panel/GripperControllers/WSG50_Controller.h
    include/operation_panel/GripperControllers/EGN160_Controller.h

    include/operation_panel/GripperCommands/IGripperCommand.h
    include/operation_panel/GripperCommands/OpenGripperCommand.h
    include/operation_panel/GripperCommands/CloseGripperCommand.h
    include/operation_panel/GripperCommands/EmptyGripperCommand.h

    include/operation_panel/Executers/IExecuter.h
    include/operation_panel/Executers/MoveGroupExecuter.h
    include/operation_panel/Executers/ExecutionWaiter.h

    include/operation_panel/Plans/MoveGroupMotionPlan.h

    include/operation_panel/CollisionObjects/CollisionObjectUtilities.h
    include/operation_panel/CollisionObjects/ICollisionObjectCommand.h
    include/operation_panel/CollisionObjects/ICollisionObjectController.h
    include/operation_panel/CollisionObjects/AddObjectCommand.h
    include/operation_panel/CollisionObjects/AttachObjectCommand.h
    include/operation_panel/CollisionObjects/DetachObjectCommand.h
    include/operation_panel/CollisionObjects/RemoveObjectCommand.h
    include/operation_panel/CollisionObjects/EmptyObjectCommand.h 
  
    include/operation_panel/MsgsParsers/IMsgsParser.h
    include/operation_panel/MsgsParsers/ObjectMsgsParser.h
    include/operation_panel/MsgsParsers/ORKMsgsParser.h
    include/operation_panel/MsgsParsers/PoseMsgsParser.h

    include/operation_panel/PathPlanners/PickAndPlacePlanner.h
    
    include/operation_panel/MotionPlanners/MoveGroupMotionPlanner.h
    include/operation_panel/MotionPlanners/OpenBagMotionPlanner.h
    include/operation_panel/MotionPlanners/ObjectScanerMotionPlanner.h
    
    include/operation_panel/Utilities/EigenUtils.h
    
    include/operation_panel/GUI/PickAndPlaceGUI.h    
    include/operation_panel/GUI/OpenBagGUI.h
    include/operation_panel/GUI/PretreatmentGUI.h
    include/operation_panel/GUI/UnscrewGUI.h
    include/operation_panel/GUI/MasterSlaveGUI.h
    
    include/operation_panel/ActionClients/ActionUnscrewClient.h
    include/operation_panel/ActionClients/ActionMasterSlaveClient.h
    
    include/operation_panel/Facades/PickAndPlaceFacade.h
    include/operation_panel/Facades/OpenBagFacade.h
    include/operation_panel/Facades/PretreatmentFacade.h
    include/operation_panel/Facades/UnscrewFacade.h
    
    include/operation_panel/BoundingBox/BoundingBox.h   
    include/operation_panel/Scene/Scene.h   
    
    include/operation_panel/InteractiveMarkers/InteractiveMarkerBase.h
    include/operation_panel/InteractiveMarkers/InteractiveMarkerArrow.h
    include/operation_panel/InteractiveMarkers/InteractiveMarkerPoint.h
    include/operation_panel/InteractiveMarkers/InteractiveMarkerObject.h
    include/operation_panel/InteractiveMarkers/InteractiveMarkerGraspChooser.h  
    include/operation_panel/InteractiveMarkers/InteractiveMarkerUnscrewChooser.h  
     
    include/operation_panel/ServersManagers/IServerManager.h
    include/operation_panel/ServersManagers/MarkerServerManager.h
    include/operation_panel/ServersManagers/TFServerManager.h
    include/operation_panel/ServersManagers/InteractiveMarkerServerManager.h
    
    include/operation_panel/PointCloudProcessor/PointCloudProcessor.h
)

set(MOC_SOURCE_FILES
    src/operation_panel.cpp
    src/scan_panel.cpp
    src/open_bag_panel.cpp
    src/master_slave_panel.cpp
    src/unscrew_panel.cpp

    src/Servers/IServer.cpp
    src/Servers/TFServer.cpp
    src/Servers/MarkerServer.cpp
    src/Servers/CoordsServer.cpp

    src/GraspsFilters/IGraspsFilter.cpp
    src/GraspsFilters/CollisionFilter.cpp
    src/GraspsFilters/ReachAbilityFilter.cpp
    src/GraspsFilters/ValidStateFilter.cpp
    
    src/Readers/IGraspReader.cpp 
    src/Readers/FileReader.cpp 
    src/Readers/DBReader.cpp

    src/GripperControllers/IGripperController.cpp
    src/GripperControllers/WSG50_Controller.cpp
    src/GripperControllers/EGN160_Controller.cpp

    src/GripperCommands/IGripperCommand.cpp
    src/GripperCommands/OpenGripperCommand.cpp
    src/GripperCommands/CloseGripperCommand.cpp
    src/GripperCommands/EmptyGripperCommand.cpp

    src/Executers/IExecuter.cpp
    src/Executers/MoveGroupExecuter.cpp
    src/Executers/ExecutionWaiter.cpp

    src/Plans/MoveGroupMotionPlan.cpp

    src/CollisionObjects/CollisionObjectUtilities.cpp
    src/CollisionObjects/ICollisionObjectCommand.cpp
    src/CollisionObjects/ICollisionObjectController.cpp
    
    src/CollisionObjects/AddObjectCommand.cpp
    src/CollisionObjects/AttachObjectCommand.cpp
    src/CollisionObjects/DetachObjectCommand.cpp
    src/CollisionObjects/RemoveObjectCommand.cpp
    src/CollisionObjects/EmptyObjectCommand.cpp
    
    src/MsgsParsers/IMsgsParser.cpp
    src/MsgsParsers/ObjectMsgsParser.cpp
    src/MsgsParsers/ORKMsgsParser.cpp
    src/MsgsParsers/PoseMsgsParser.cpp
    
    src/PathPlanners/PickAndPlacePlanner.cpp

    src/MotionPlanners/MoveGroupMotionPlanner.cpp
    src/MotionPlanners/OpenBagMotionPlanner.cpp
    src/MotionPlanners/ObjectScanerMotionPlanner.cpp

    src/Utilities/EigenUtils.cpp

    src/GUI/PickAndPlaceGUI.cpp
    src/GUI/OpenBagGUI.cpp 
    src/GUI/PretreatmentGUI.cpp
    src/GUI/UnscrewGUI.cpp 
    src/GUI/MasterSlaveGUI.cpp
    
    src/ActionClients/ActionUnscrewClient.cpp
    src/ActionClients/ActionMasterSlaveClient.cpp
    
    src/Facades/PickAndPlaceFacade.cpp
    src/Facades/OpenBagFacade.cpp
    src/Facades/PretreatmentFacade.cpp
    src/Facades/UnscrewFacade.cpp
    
    src/BoundingBox/BoundingBox.cpp    
    src/Scene/Scene.cpp
    
    src/InteractiveMarkers/InteractiveMarkerBase.cpp
    src/InteractiveMarkers/InteractiveMarkerArrow.cpp
    src/InteractiveMarkers/InteractiveMarkerPoint.cpp
    src/InteractiveMarkers/InteractiveMarkerObject.cpp
    src/InteractiveMarkers/InteractiveMarkerGraspChooser.cpp  
    src/InteractiveMarkers/InteractiveMarkerUnscrewChooser.cpp  
    
    src/ServersManagers/IServerManager.cpp
    src/ServersManagers/MarkerServerManager.cpp
    src/ServersManagers/TFServerManager.cpp
    src/ServersManagers/InteractiveMarkerServerManager.cpp   
    
    src/PointCloudProcessor/PointCloudProcessor.cpp  
   
  ${MOC_FILES}
)

add_library(OperationPanel 
	    ${MOC_SOURCE_FILES} 
# 	    ${SOURCE_FILES}
	    )
target_link_libraries(OperationPanel 
	    ${catkin_LIBRARIES}
	    ${QT_LIBRARIES}
	    ${PCL_LIBRARIES}
            )

install(TARGETS OperationPanel
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# install
install(FILES plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

