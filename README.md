# Robot System Programming Project

## Dependencies

This project was built for ROS2 Galactic and a ROS2 environment with Ubuntu 20.04 is necessary for execution. 

Some packages in this project use the Eigen3 math library. Please ensure that Eigen3 is installed

```
sudo apt install libeigen3-dev
```
Moveit is used to control the UR5. Please ensure that moveit is installed

```
sudo apt install ros-galactic-moveit
```

If moveit is not visible in Rviz planners tab while execution try installing the following package to fix the problem
(https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/438)

```
sudo apt install ros-galactic-backward-ros
```
The main launch file in this package uses xterm as a command window. If [xterm](https://zoomadmin.com/HowToInstall/UbuntuPackage/xterm) is not already installed, run

```
sudo apt-get install xterm
```
Some tools are required to install other dependencies and build the code, follow the commands to ensure all of them are installed.
```
sudo apt-get install git-all
sudo apt install python3-vcstool
sudo apt install python3-colcon-common-extensions
sudo apt-get install python3-rosdep
```

## Installation
Download all the packages for this project.
```
vcs import src < main.repos
```
Since the `scaled_joint_trajectory_controller` does not work on ROS2 Galactic. You must change the default controller to `joint_trajectory_controller` in the `src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/controllers.yaml` file. Please see [this](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/galactic#fake-hardware-on-ros2-galactic) for detail instructions.

Install rust plugins for gripper driver. For detail instruction, please follow this [README](https://github.com/yhdeng-ryan/robotiq_2f/blob/master/README.md).
Here are all the commands:
```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
sudo apt install -y git libclang-dev python3-pip python3-vcstool
```
Open new terminal then,
```
cargo install cargo-ament-build
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
```

Lastly, run rosdep and use colcon to build everything.
```
sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths src -y --ignore-src --skip-keys="Eigen3"
colcon build
```
## Structure

    ├── src
    │   ├── Universal_Robots_ROS2_Driver          # UR5 Offical Drivers
    │   ├── hololens2-ur5-pick-and-place          # Hololens App written in Unity
    │   ├── ROS2_pick_and_place_UR5               # Pick and place ROS2 action for UR5 and Robotiq gripper
    │   ├── ros2_fiducial_registration_server     # Fiducial registration service
    │   ├── ros2-hololens2-communication          # TCP Connection between Unity and ROS2
    │   ├── robotiq_2f                            # Robotiq gripper driver
    │   ├── dummy_hololens                        # A dummy hololens message publisher for simulation and testing
    │   └── main                                  # Main function to start pick and place with Hololens
    └── ...

## Usage
### Actual robot
1. Set up UR5 connection in the teach pendant and your computer. Please follow this [instruction](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/galactic#readme). We use same setup in the instruction.

2. Turn on Hololens, choose all APP's and click `Holo Ros Comm`.

3. Launch `main.launch.xml` and you will see Rviz open up with the UR5 and gripper models loaded. Another command window will show, this will be the control termianl for the following steps. 
```
source install/setup.bash
ros2 launch main main.launch.xml
```
IMPORTANT NOTE: Close the joint state publisher gui window after running the launch file. The robot will not move correctly if the gui is running while the robot tries to move.

4. In the teach pendant, go to `Load Program`, `Installation`, `External Control`, put in
```
Host IP: 192.168.1.101
Custom port: 50002
```
Ensure that the robot is not in a singular configuration by moving it with the teach pendant.

5. In the contorl termianl, it will ask "Perform registration? (y/n)", answer `y`. Hololens needs to be register with the robot's frame everytime you open the Hololens app. The registration positions are predefined in `src/main/json/reg_cfg.json`(required colcon build after edit) and `install/main/share/main/json/reg_cfg.json`.

6. Follow the instructions in the control termianl and let the gripper hold on to the marker. In Hololens, press `Connect` then press `send marker` 6 times when UR5 arrives it's registrion positions to send marker's positions to ROS to complete the data collection for registraion.

7. In Hololen, press `Spawn pick` and use gestures to drag green cube to your desired pick position. Then press `Spawn place` and move the red cube to your desired place position.

8. Press `Send pick` and `Send place` and the robot will start a pick and place routine.

9. To repeat, simply click `Send pick` and `Send place` and it will start again.


### Simulated robot
1. Launch `main_sim.launch.xml` and you will see Rviz open up with the UR5 and gripper models loaded. Four other command windows will also open up. 
```
source install/setup.bash
ros2 launch main main_sim.launch.xml
```
IMPORTANT NOTE: Close the joint state publisher gui window after running the launch file. The robot will not move correctly if the gui is running while the robot tries to move.

2. In Rviz, ensure that a planning library is loaded in the context tab. Command the robot in rviz to a non singular pose by dragging the end effector of the robot and clicking the `Plan & Execute` button.

3. In the main_node xterm window, enter `y` to begin the registration process. Press enter when promped to attach the marker since no physical marker is required for simulation. The robot will move to the various registration poses one by one.

4. Each time the robot moves to a registration position, enter the coordinates of the end effector in the `dummy_reg_pub` xterm window one by one. These positions should be consistent with a transformed version of the registration positions. For simplicity, you can input the coordinates of the registration positions directly (implying a unity transform). The default coordinates are:
```
Position 1: 0.3, 0.5. 0.5
Position 2: 0.3, 0.5. 0.6
Position 3: 0.0, 0.7. 0.6
Position 4: 0.0, 0.7. 0.5
Position 5: -0.4, 0.5. 0.5
Position 6: -0.4, 0.5. 0.6
```

5. Once the robot finishes moving to all the registration positions, press enter in the `main_node` xterm window to end the registration process. The robot is now ready for pick and place commands. Send one pick position through the `dummy_pick_pub` xterm window and one place position through the `dummy_place_pub` xterm window. The robot will now move to both positions and be ready to accept another command.

## Demo
To watch the full demo video, please visit [HoloLens 2 Pick and Place with UR5](https://youtu.be/8-yKbps1ocE)
### Registration
<img src="/Demo/Registration.png" width="720">

### Setting Pick Position
<img src="/Demo/Pick.png" width="720">

### Setting Place Position
<img src="/Demo/Place.png" width="720">

### Pick and Place
<img src="/Demo/ShortPnP.gif" width="720">