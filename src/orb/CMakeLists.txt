cmake_minimum_required(VERSION 3.5)
project(orb)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/camera_intrinsics.py
    scripts/keypoint_viewport.py
    scripts/mono_odom_copy.py
    scripts/create_dataset.py
    scripts/img_send.py
    scripts/mono_odom.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()