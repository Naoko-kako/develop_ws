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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



add_executable(catch_publisher src/catch_publisher.cpp)
target_link_libraries(catch_publisher PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(catch_subscriber src/catch_subscriber.cpp)
target_link_libraries(catch_subscriber PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})


add_executable(my_turtle_controller src/my_turtle_controll.cpp)
target_link_libraries(my_turtle_controller PUBLIC rclcpp::rclcpp ${geometry_msgs_TARGETS})
#ament_target_dependencies(my_turtle_controller rclcpp std_msgs geometry_msgs

add_executable(ritsmaycan src/catch_main.cpp src/PS4.cpp src/catch.cpp src/include/UDP.cpp src/include/IP.cpp)
ament_target_dependencies(ritsmaycan rclcpp std_msgs sensor_msgs geometry_msgs)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# figletがインストールされていない場合、警告を表示
execute_process(
  COMMAND dpkg -s figlet
  RESULT_VARIABLE figlet_installed
  OUTPUT_QUIET
  ERROR_QUIET
)

if(NOT figlet_installed EQUAL 0)
  message(WARNING "Please install 'figlet' with the following command. \nsudo apt install figlet")
endif()

install (TARGETS
  catch_publisher
  catch_subscriber
  my_turtle_controller
  ritsmaycan
  DESTINATION lib/${PROJECT_NAME})

ament_package()


