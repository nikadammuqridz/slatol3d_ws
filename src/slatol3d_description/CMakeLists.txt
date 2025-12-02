cmake_minimum_required(VERSION 3.8)
project(slatol3d_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz_sim REQUIRED)

# Install files and configurations
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()