cmake_minimum_required(VERSION 3.8)
project(Test_5)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()
