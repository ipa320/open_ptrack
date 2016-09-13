cmake_minimum_required(VERSION 2.8.3)
project(opt_msgs)

find_package(
  catkin REQUIRED COMPONENTS 
  geometry_msgs
  message_generation 
  roscpp 
  sensor_msgs
)

add_message_files(
  FILES 
  BoundingBox2D.msg 
  BoundingBox3D.msg 
  CalibrationStatus.msg
  Detection.msg 
  DetectionArray.msg  
  HumanEntry.msg 
  HumanEntries.msg
  IDArray.msg
  Rois.msg 
  RoiRect.msg 
  Track.msg 
  TrackArray.msg 
)

add_service_files(
  FILES 
  OPTSensor.srv
  OPTTransform.srv
)

generate_messages(
  DEPENDENCIES 
  geometry_msgs 
  sensor_msgs
  std_msgs 
)


catkin_package(
   INCLUDE_DIRS 
   LIBRARIES 
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)

include_directories(include include/open_ptrack/${PROJECT_NAME}/)

add_library(roi_msgs src/overlap.cpp )

target_link_libraries(roi_msgs ${catkin_LIBRARIES})
