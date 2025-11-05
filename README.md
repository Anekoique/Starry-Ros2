# Build img with minimal ros2 (RISC-V)

Target arch：riscv

Prerequisites: 

```shell
sudo pacman -S debootstrap qemu-user-static qemu-user-static-binfmt
```

Compile content：A min ros2 jazzy which run demo successfully

- ros2cli  ros2run (cli tools)
- demo_nodes_cpp   demo_nodes_py (demo)s
- rclcpp  rclpy  (library)
- rwm_fastrtps_cpp fastrtps (RMW/DDS)  

Compile method：QEMU-user emulated Native Compilation

[A Prebuild img](https://github.com/Anekoique/Starry-Ros2/releases/tag/debian-ros2-jazzy)

Prepare debian img:  

```shell
# build debian img
make init
# chroot
make root
```

Compile ros2 from source:

```shell
# chroot
make root

# dependencies
apt-get update
apt-get install -y python3-pip   python3-venv \
	wget git build-essential cmake ninja-build curl pkg-config \
	libacl1-dev liblttng-ctl-dev liblttng-ust-dev lttng-tools libasio-dev libtinyxml2-dev
pip install colcon-common-extensions vcstool  numpy lark empy

# colcon with python venv
python3 -m venv /opt/colconenv
. /opt/colconenv/bin/activate

# Get ros2 jazzy source
mkdir -p /root/ros2_ws/src && cd /root/ros2_ws
wget https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos -O ros2.repos
vcs import src < ros2.repos

# Replace repo
sed -z -E -i 's@(ament_vendor\(\s*mimick_vendor[^)]*VCS_URL\s+)(\"[^\"]*\"|[^[:space:])]+)@\1https://github.com/ziyao233/Mimick.git@' src/ros2/mimick_vendor/CMakeLists.txt
sed -z -E -i 's@(ament_vendor\(\s*mimick_vendor[^)]*VCS_VERSION\s+)(\"[^\"]*\"|[^[:space:])]+)@\190d02296025f38da2e33c67b02b7fa0c7c7d460c@' src/ros2/mimick_vendor/CMakeLists.txt

# build
colcon build \
	--merge-install  \
	--packages-up-to ros2run ros2cli demo_nodes_cpp demo_nodes_py rclcpp rclpy rmw_fastrtps_cpp fastrtps   \
	--cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF  \
    --event-handlers console_direct+
```

A minimal test demo

```shell
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

# TODO

- clone3
- timerfd
- netlink
- ioctl command: 21505

# FIX:

arceos:

- openat is_valid()

Starry:

- setpgid pid == proc().id()

# Starry for ros2s

在Starry实现ros2基础功能的移植，大致需要实现以下库的移植

1. DDS移植（使用默认Fast DDS）
2. 移植ros2工具库

- rcutils
- rcpputils
- rosidl

3. 移植ros2 rmw中间件
4. 移植ros2上层接口rcl rclcpp

已知存在的困难和问题：

- ros2基于glibc，而目前starry运行的是alpine linux镜像
- ros2官方只支持x86和arm架构

后续需要对Starry添加的支持：

- cmake
- ...
