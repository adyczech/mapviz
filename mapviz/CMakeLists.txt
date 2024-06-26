cmake_minimum_required(VERSION 2.8.3)
project(mapviz)

set(COMMON_DEPS
  cv_bridge
  image_transport
  marti_common_msgs
  pluginlib
  rosapi
  roscpp
  rqt_gui
  rqt_gui_cpp
  std_srvs
  swri_transform_util
  swri_yaml_util
  tf
)
set(BUILD_DEPS
  ${COMMON_DEPS}
  message_generation
)
set(RUNTIME_DEPS
  ${COMMON_DEPS}
  message_runtime
)

find_package(catkin REQUIRED COMPONENTS
  ${BUILD_DEPS}
)

### Boost ###
find_package(Boost REQUIRED system filesystem)

### OpenCV
find_package(OpenCV COMPONENTS core imgproc imgcodecs videoio REQUIRED)

### Qt
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
# Setting these variable so catkin_package can export Qt as a dependency
set(Qt_FOUND TRUE)
set(Qt_INCLUDE_DIRS "${Qt5Concurrent_INCLUDE_DIRS};${Qt5Core_INCLUDE_DIRS};${Qt5Gui_INCLUDE_DIRS};${Qt5OpenGL_INCLUDE_DIRS};${Qt5Widgets_INCLUDE_DIRS}")
set(Qt_LIBRARIES
  Qt5::Concurrent
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Widgets
)
add_definitions(-DWFlags=WindowFlags)
set(QT_USE_QTOPENGL TRUE)

### OpenGL ###
if(${CMAKE_VERSION} VERSION_EQUAL "3.11.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.11.0")
  cmake_policy(SET CMP0072 NEW)
endif()
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

### GLEW ###
find_package(GLEW REQUIRED)

add_service_files(FILES
  AddMapvizDisplay.srv
)

generate_messages(DEPENDENCIES
  marti_common_msgs
)

catkin_package(
  LIBRARIES rqt_${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS Qt
  CATKIN_DEPENDS ${RUNTIME_DEPS}
)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

# Need to include the current dir to include the results of building Qt UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIR}
  ${GLEW_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Qt UI files
set(UI_FILES
  src/configitem.ui
  src/${PROJECT_NAME}.ui
  src/pluginselect.ui
)
# Headers containing QObject definitions
set(QT_HEADERS
  include/${PROJECT_NAME}/color_button.h
  include/${PROJECT_NAME}/config_item.h
  include/${PROJECT_NAME}/map_canvas.h
  include/${PROJECT_NAME}/${PROJECT_NAME}.h
  include/${PROJECT_NAME}/${PROJECT_NAME}_plugin.h
  include/${PROJECT_NAME}/rqt_${PROJECT_NAME}.h
  include/${PROJECT_NAME}/select_frame_dialog.h
  include/${PROJECT_NAME}/select_service_dialog.h
  include/${PROJECT_NAME}/select_topic_dialog.h
  include/${PROJECT_NAME}/video_writer.h
  include/${PROJECT_NAME}/widgets.h
)
# Source files for mapviz
set(SRC_FILES
  src/${PROJECT_NAME}.cpp
  src/color_button.cpp
  src/config_item.cpp
  src/${PROJECT_NAME}_application.cpp
  src/map_canvas.cpp
  src/rqt_${PROJECT_NAME}.cpp
  src/select_frame_dialog.cpp
  src/select_service_dialog.cpp
  src/select_topic_dialog.cpp
  src/video_writer.cpp
)

qt5_add_resources(RCC_SRCS src/resources/icons.qrc)
qt5_wrap_ui(SRC_FILES ${UI_FILES})
qt5_wrap_cpp(SRC_FILES ${QT_HEADERS})

### Build mapviz as an rqt plugin ###
add_library(rqt_${PROJECT_NAME}
  ${SRC_FILES}
  ${RCC_SRCS}
)
target_link_libraries(rqt_${PROJECT_NAME}
  ${Qt_LIBRARIES}
  ${OpenGL_LIBRARY}
  ${GLUT_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
add_dependencies(rqt_${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
if ($ENV{ROS_DISTRO} STREQUAL "kinetic" OR $ENV{ROS_DISTRO} STRLESS "kinetic")
  set_target_properties(rqt_${PROJECT_NAME} PROPERTIES 
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
  )
endif()
set_target_properties(rqt_${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS "-D__STDC_FORMAT_MACROS"
)
add_dependencies(rqt_${PROJECT_NAME}
  ${PROJECT_NAME}_gencpp
)

### Build mapviz as a standalone executable ###
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_main.cpp
)
target_link_libraries(${PROJECT_NAME}
  rqt_${PROJECT_NAME}
  ${Qt_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenGL_LIBRARY}
  ${GLUT_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLU_LIBRARY}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
if ($ENV{ROS_DISTRO} STREQUAL "kinetic" OR $ENV{ROS_DISTRO} STRLESS "kinetic")
  set_target_properties(${PROJECT_NAME} PROPERTIES 
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
  )
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS "-D__STDC_FORMAT_MACROS"
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencpp
)

### Install mapviz ###
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} rqt_${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
