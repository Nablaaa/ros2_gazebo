# ROS 2 Gazebo

This repo is the follow-up of the [ROS2 introduction](https://github.com/Nablaaa/ROS2.git), based on the book "ROS 2 from Scratch".

## Naming
The package inside the workspace will get the robot name with the ending "_description", to make it clear for other developers, that I am using this package for URDF files for the robot.

```bash
cd src/
ros2 pkg create robot_car_description --build-type ament_cmake 

# remove include and src (we do not plan to write nodes)
cd robot_car_description/
rm -r include/ src/
```

Clean up the CMakeLists.txt, bu removing comments after find_package (ament....) line and also remove if (BUILD_TESTING) section.
