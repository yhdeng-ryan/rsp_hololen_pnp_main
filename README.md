# Robot System Programming Project

## Installation
Download all the packages for this project.
```
vcs import src < main.repos
```
Since the `scaled_joint_trajectory_controller` does not work on ROS2 Galactic. You must change the default controller to `joint_trajectory_controller` in the `controllers.yaml` file. Please see [this](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/galactic#readme) for detail instructions.

Install rust plugins for gripper driver. For detail instruction, please follow this [README](https://github.com/yhdeng-ryan/robotiq_2f/blob/master/README.md).
Here are all the commands:
```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
sudo apt install -y git libclang-dev python3-pip python3-vcstool
cargo install cargo-ament-build
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
```

Lastly, run rosdep and use colcon to build everything.
```
sudo rosdep init
rosdep update
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
    │   └── main                                  # Main function to start pick and place with Hololens
    └── ...

## Usage
### Actual robot
1. Set up UR5 connection in the teach pendant and your computer. Please follow this [instruction](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/galactic#readme). We use same setup in the instruction.

2. Turn on Hololens, choose all APP's and click `Holo Ros Comm`.

3. Launch `main.launch.xml` and you will see Rviz open up with the UR5 and gripper models loaded. Another command window will show, this will be the control interface following steps. 
```
source install/setup.bash
ros2 launch main main.launch.xml
```

4. In the teach pendant, go to `Load Program`, `Installation`, `External Control`, put in
```
Host IP: 192.168.1.101
Custom port: 50002
```

5. In the contorl interface, it will ask "Perform registration? (y/n)", answer `y`. Hololens needs to be register with the robot's frame everytime you open the Hololens app. The registration positions are predefined in `src/main/json/reg_cfg.json`(required colcon build after edit) and `install/main/share/main/json/reg_cfg.json`.

6. Follow the instructions in the control interface and let the gripper hold on to the marker. In Hololens, press `Connect` then press `send marker` 6 times when UR5 arrives it's registrion positions to send marker's positions to ROS to complete the data collection for registraion.

7. In Hololen, press `Spawn pick` and use gestures to drag green cube to your desired pick position. Then press `Spawn place` and move the red cube to your desired place position.

8. Press `Send pick` and `Send place` and the robot will start a pick and place routine.

9. To repeat, simply click `Send pick` and `Send place` and it will start again.
