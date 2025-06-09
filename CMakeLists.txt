cmake_minimum_required(VERSION 3.8)
project(agribot)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_localization REQUIRED)
find_package(rosbridge_server REQUIRED)



# Nếu có code Python trong thư mục agribot/ (ví dụ imu_node.py, gps_node.py,...)
install(
  DIRECTORY agribot
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Nếu có folder launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

# Cài đặt folder web để HTTP server có thể serve index.html
install(DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
)