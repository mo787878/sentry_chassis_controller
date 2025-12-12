# sentry_chassis_controller

## 项目简介

这是mo为最终考核而写的一个控制舵轮底盘运动的项目，主要功能有以给定速度控制小车运动，将小车速度与姿态以里程计形式发布等



## 安装

### 从源代码构建

#### 依赖

1.[Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

2.[rm_description_for_task 任务的描述](https://github.com/Whathelp233/rm_description_for_task.git)

3.controller_interface

4.hardware_interface

5.pluginlib

6.[chassis_teleop](https://github.com/mo787878/chassis_teleop.git)



#### 构建

从源代码构建，将最新版本从此存储库克隆到您的 catkin 工作区，并使用以下命令编译该软件包：

> ```
> cd catkin_workspace/src
> git clone git@github.com:mo787878/sentry_chassis_controller.git
> # git clone https://github.com/mo787878/sentry_chassis_controller.git
> cd ../
> rosdep install --from-paths . --ignore-src
> catkin build
> ```



## 使用

运行模拟和控制器：

> roslaunch sentry_chassis_controller sentry_chassis_controller.launch



### 配置文件

1.**controllers.yam** simple_chassis_controller 的参数和joint_state_controller.

2.**pid_params.yaml** pid的参数和其他的各种参数



### 发布文件

**1.sentry_chassis_controller.launch**



### 联系方式

如有问题，请联系：2474717264@qq.com
