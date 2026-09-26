cmake_minimum_required(VERSION 3.8)
project(lastmile_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY maps params launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
