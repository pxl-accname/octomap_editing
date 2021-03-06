cmake_minimum_required(VERSION 2.8.3)
project(octomap_editing)

# enable c+11 suport
# set(CMAKE_CXX_FLAGS "-std=c++0x $(CMAKE_CXX_FLAGS)")
add_compile_options(-std=c++11)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  interactive_markers
  std_msgs
  sensor_msgs
  octomap_msgs
  pcl_ros
  octomap_server
  geometry_msgs
  rviz
)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
catkin_package()

find_package(octomap REQUIRED)

# set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/Qt5Widget")
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
# include(${QT_USE_FILE})


## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)
## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt5_wrap_cpp(MOC_FILES
  include/OEPanel.hpp
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES leuze_sensor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  include
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS OCTOMAP
)

# set(CMAKE_SOURCE_DIR /home/accname/programming/ROS/OctoMap/catkin_ws/src)
# link_directories(${CMAKE_SOURCE_DIR}/utillib/lib)

## Declare a cpp library
# add_library(leuze_sensor
#   src/${PROJECT_NAME}/leuze_sensor.cpp
# )


set(SOURCE_FILES
  src/octomap_editing.cpp
  src/OEServer.cpp
  src/OEMarkerFactory.cpp
  src/OECube.cpp
  src/OECubeMarker.cpp
  src/OECubeLine.cpp
  src/OEPlane.cpp
  src/OEPanel.cpp
  ${MOC_FILES}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
qt5_use_modules(octomap_editing Core Gui Widgets)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(leuze_sensor leuze_sensor_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  # default_plugin
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  # Qt5::Widgets

)


