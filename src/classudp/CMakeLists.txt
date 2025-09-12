cmake_minimum_required(VERSION 3.8)
project(classudp)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


#add_executable(catch_publisher src/catch_publisher.cpp)
#target_link_libraries(catch_publisher PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

#add_executable(catch_subscriber src/catch_subscriber.cpp)
#target_link_libraries(catch_subscriber PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install (TARGETS
  catch_publisher
  catch_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()


