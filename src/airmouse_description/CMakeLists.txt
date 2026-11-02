cmake_minimum_required(VERSION 3.8)
project(airmouse_description)

find_package(ament_cmake REQUIRED)

# Models and worlds are looked up at runtime via the package share directory.
install(DIRECTORY models worlds
  DESTINATION share/${PROJECT_NAME}
)

# Installed as an executable so it can be run with `ros2 run airmouse_description generate_maze.py`.
install(PROGRAMS scripts/generate_maze.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
