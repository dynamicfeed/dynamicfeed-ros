cmake_minimum_required(VERSION 3.8)
project(dynamicfeed_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files, config, and example scripts into the package share directory.
install(DIRECTORY launch config examples DESTINATION share/${PROJECT_NAME})

# Install the humanoid example as a runnable script under lib/${PROJECT_NAME} so a developer
# can call:  ros2 run dynamicfeed_bringup humanoid_example_node
# The RENAME strips the .py extension, which is the ament convention for Python executables.
install(PROGRAMS
  examples/humanoid_example_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME humanoid_example_node
)

ament_package()
