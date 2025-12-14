# Build img with ros2 (RISC-V)

**Target arch**：riscv

**Prerequisites**: 

```shell
sudo pacman -S debootstrap qemu-user-static qemu-user-static-binfmt
```

**Compile method**：QEMU-user emulated Native Compilation

**Compile content**：

**1. A minimal ros2 jazzy (Base)**
[A minimal img](https://github.com/Anekoique/Starry-Ros2/releases/tag/debian-jazzy-minimal)

- CLI Tools: `ros2cli` (core), `ros2run` (executable launcher)
- Demos: `demo_nodes_cpp`, `demo_nodes_py` (talker/listener)
- Core Libraries: `rclcpp`, `rclpy` (client libraries)
- Middleware: `rmw_fastrtps_cpp`, `fastrtps` (Fast DDS implementation)

**2. More features compiled/support (Enhanced)**
[A prebuild img](https://github.com/Anekoique/Starry-Ros2/releases/tag/debian-jazzy-v0.1)

- CLI Verbs: `topic`, `node`, `service`, `param`, `interface`, `doctor` `action`  `lifecycle`  `component` `pkg` `ros2launch` 
- Advanced C++ Patterns: `rclcpp_action`  `rclcpp_lifecycle`  `rclcpp_components` 
- Robotics Foundation: `tf2_ros`, `tf2_geometry_msgs` 
- Interfaces: `common_interfaces` 

**Prepare debian img:**  

```shell
# build debian img
make init
# chroot
make root
```

**Compile ros2 from source:**

```shell
# chroot
make root

# dependencies
apt-get update
apt-get install -y python3-pip   python3-venv \
	wget git build-essential cmake ninja-build curl pkg-config \
	libacl1-dev liblttng-ctl-dev liblttng-ust-dev \
	lttng-tools libasio-dev libtinyxml2-dev libeigen3-dev
pip install colcon-common-extensions vcstool  numpy lark empy psutil

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

colcon build --merge-install \
  --packages-up-to \
    ros2cli ros2run ros2launch ros2pkg \
    ros2topic ros2node ros2service ros2param ros2interface ros2doctor \
    ros2action ros2lifecycle ros2component \
    demo_nodes_cpp demo_nodes_py \
    rclcpp_action rclcpp_lifecycle rclcpp_components \
    tf2_ros tf2_geometry_msgs \
    common_interfaces \
  --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct+
```

**A minimal test demo**

```shell
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

# TODO

- timer check
- more features support

# FIX

arceos:

- openat is_valid() https://github.com/Starry-OS/arceos/pull/25
- handle concurrent page_faults on same page https://github.com/Starry-OS/arceos/pull/16
- The `timeout_at` function should handle the case where `timeout=0` and the timeout check branch should be executed first.
- `bind_check`  handle ANY_ADDR correctly
- enable feature support IPV4 fragmentation to handle packet correctly 

Starry:

- setpgid pid == proc().id() https://github.com/Starry-OS/StarryOS/pull/71
- getcwd should return filled len
- futex with FUTEX_WAIT_BITSET should use absolute time

# FEAT

arceos:

- netlink route https://github.com/Starry-OS/arceos/pull/25
- enhance axio https://github.com/Starry-OS/axio/pull/1
- Multicast

Starry:

- netlink route https://github.com/Starry-OS/StarryOS/pull/71
- ioctl with FIOCLEX or FIONCLEX
- refactor setsockopt to handle C structs with different len

# Starry for ros2

在Starry实现ros2基础功能的支持，大致需要实现以下库的支持

1. DDS支持 [default Fast DDS]
2. 支持ros2工具库，[rcutils，rcpputils，rosidl]

3. 支持ros2 rmw和上层接口rcl rclcpp

构建支持:`cmake` `colcon`

相关功能支持：`topic` `node` `run` `service`

已知存在的困难和问题：

- ros2基于glibc，而目前starry运行的是alpine linux镜像 -> [build debian(trixie) img]
- ros2官方只支持x86和arm架构 -> [replace related repo]

