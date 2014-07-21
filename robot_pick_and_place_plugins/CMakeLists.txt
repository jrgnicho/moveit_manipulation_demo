cmake_minimum_required(VERSION 2.8.3)
project(robot_pick_and_place_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rviz
	robot_pick_and_place)


# c++11
ADD_DEFINITIONS("-std=c++0x")

# QT
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 	roscpp rviz robot_pick_and_place
)

###########
## Build ##
###########

qt4_wrap_cpp(robot_pick_and_place_plugins_MOCS 
	include/robot_pick_and_place_plugins/rviz_panels/pick_and_place_panel.h)

qt4_wrap_ui(robot_pick_and_place_plugins_UIS_H 
	ui/pick_and_place_widget.ui)

include_directories(include
  ${catkin_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}
)

add_library(${PROJECT_NAME}
	src/rviz_panels/pick_and_place_panel.cpp
	${robot_pick_and_place_plugins_MOCS}
	${robot_pick_and_place_plugins_UIS_H})

target_link_libraries(${PROJECT_NAME} 
	${catkin_LIBRARIES} 
	${QT_LIBRARIES} 
	${QT_QTCORE_LIBRARY} 
	${QT_QTGUI_LIBRARY})

add_dependencies(${PROJECT_NAME} robot_pick_and_place_generate_messages_cpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})



