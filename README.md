## dependencies

ensure you have the following dependencies installed on your Ubuntu system:

- ROS2
- Gazebo
- CMake
- Git
- ROS2 Nav2
- Slam Toolbox

## usage

1. clone the repository:

   ```bash
   git clone https://github.com/ulastanersoyak/autonomus_driving_robot.git
   ```
2. navigate to the cloned directory:

   ```bash
   cd autonomus_driving_robot
   ```

3. source the setup file depending on your shell:

   - bash:
     ```bash
     source install/setup.bash
     ```

   - zsh:
     ```bash
     source install/setup.zsh
     ```
4. launch the Gazebo simulation:

   ```bash
   ros2 launch bringup gazebo.launch.xml
   ```
