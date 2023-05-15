# RPS Project

## Installation
Download all the packages for this project.
```
vcs import src < main.repos
```

Install rust plugins for gripper driver. For detail instruction, please follow this [README](https://github.com/yhdeng-ryan/robotiq_2f/blob/master/README.md).
Here are all the commands:
```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
sudo apt install -y git libclang-dev python3-pip python3-vcstool
cargo install cargo-ament-build
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
```

Lastly, use colcon to build everything.
```
colcon build
```
