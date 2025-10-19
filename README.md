# 前期调研

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
