cmake_minimum_required(VERSION 3.5)

project(figaro_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(figaro_msgs
  "msg/StringArray.msg"
  "msg/AgentPosition.msg"
  "msg/Entity.msg"
  "msg/Intent.msg"
  "msg/AudioContent.msg"
  "msg/FigureAudioContent.msg"
  "msg/IntentArray.msg"
  "msg/Speech.msg"
  "msg/SpeechArray.msg"
  "msg/Query.msg"
  "msg/DataPass.msg"
  "msg/ItemLocations.msg"
  "msg/RegionLocations.msg"
  "msg/PhysicalLayout.msg"
  "msg/QueryArray.msg"
  "msg/DataPassArray.msg"
  "msg/PathComponent.msg"
  "msg/PathComponentArray.msg"
  "msg/SceneSummary.msg"
  "msg/SceneSummaryArray.msg"
  "msg/DisplayContent.msg"
  "msg/Mouse.msg"
  "msg/PointArray.msg"
  "msg/DataPassUndefinedIntent.msg"
  "msg/DataPassAllIntentCategories.msg"
  "msg/QueryUndefinedIntent.msg"
  "msg/QueryConflict.msg"
  "msg/DataPassConflictResolution.msg"
  "msg/Figurines.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
