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

## Get the URDF files ready for the package

Clean up the CMakeLists.txt, by removing comments after find_package (ament....) line and also remove if (BUILD_TESTING) section. Later on, we add instructions to build our package.

Now create a folder for the urdf and place the urdf.xacro files in it (or create them from scratch). Inside the main file, we have to make the "include" part more robust, we can do this by replacing:

```xml
// replace this
<xacro:include filename="common_properties.xacro" />

// with this
<xacro:include filename="$(find robot_car_description)/urdf/common_properties.xacro" />
```

and then add the instructions to the CMakeLists.txt
```cmake
find_package(ament_cmake REQUIRED)
install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)
ament_package()
```
That allows to find the urdf files from anywhere in the workspace. It is very similar to launch files and param files.

### Custom meshes
It is also possible to include custom .stl meshes. For this it is common practice, to create a folder "meshes" inside robot_car_description (like with urdf), where I can save all my custom files.
In the URDF file, I can then include the mesh like this:
```xml
<mesh filenmae="file://$(find robot_car_description)/meshes/my_mesh.stl" />
```
and in the CmakeLists.txt I have to just write `DIRECTORY urdf meshes` to include the meshes in the installation. (so just add a space after urdf and type the foldername). And that's it.