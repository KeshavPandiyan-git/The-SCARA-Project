cmake_minimum_required(VERSION 3.8)
project(scara_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install meshes if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install rviz config if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install Python scripts
install(PROGRAMS
  scripts/joint_state_publisher_simple.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
