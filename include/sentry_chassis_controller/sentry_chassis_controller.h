//
// Created by abc on 25-11-25.
//

#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <cmath>
#include <ros/time.h>
#include <sentry_chassis_controller/SentryChassisParamsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



namespace sentry_chassis_controller {
class SentryChassisController : public controller_interface ::Controller<hardware_interface::EffortJointInterface>{
public:
    SentryChassisController();
    ~SentryChassisController() override = default;

    bool init(hardware_interface::EffortJointInterface* effortJointInterface,
              ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void update(const ros::Time &time, const ros::Duration &Period) override;



    hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
    hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_, back_left_wheel_joint_, back_right_wheel_joint_;
private:
    ros::Time last_change_;
    double wheel_track_;
    double wheel_base_;
    double wheel_radius_;// 轮子半径
    control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;
    control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_;
    double p_, i_, d_, i_max_, i_min_, p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel, chassis_speed;
    ros::Subscriber cmd_vel_sub_;  // 用于接收速度指令
    geometry_msgs::Twist cmd_vel_; // 存储当前速度指令
    ros::Publisher  cmd_vel_pub_;  // /cmd_vel 发布者

    ros::Time last_publish_time_;   // 上一次发布时间
    ros::Duration publish_interval_;// 发布间隔（默认0.1秒=10Hz）

    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    double x_, y_, yaw_;  // 里程计坐标和航向角
    ros::Time last_update_time_;

    tf2_ros::Buffer tf_buffer_;                // TF缓冲区
    tf2_ros::TransformListener tf_listener_;   // TF监听器
    bool use_global_vel_mode_;                 // 全局坐标系速度模式开关

    double max_linear_accel_;  // 最大线加速度 (m/s²)
    double max_angular_accel_; // 最大角加速度 (rad/s²)

    double last_vx_cmd_;
    double last_vy_cmd_;
    double last_wz_cmd_;

    ros::Time last_cmd_time_;  // 最后接收速度指令的时间
    double cmd_timeout_;       // 超时阈值（秒），超过此时间判定为无命令
    bool is_locked_;           // 是否处于自锁状态

    double k1;
    double k2;
    double P_max;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    boost::shared_ptr<dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisParamsConfig>> dyn_reconfig_server_;
    dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisParamsConfig>::CallbackType dyn_reconfig_callback_;

    void reconfigureCallback(sentry_chassis_controller::SentryChassisParamsConfig &config, uint32_t level);

    geometry_msgs::Twist transformGlobalToBase(const geometry_msgs::Twist& global_twist, const ros::Time& time);

    double limitAcceleration(double target, double current, double max_accel, double dt);

    void PublishTF(ros::Time time);
    void PublishOdometry(ros::Time time, double vx_get, double vy_get, double wz_get);
    double Compute_K(double fl_target_vel, double fr_target_vel, double bl_target_vel, double br_target_vel, ros::Duration period);
    void PublishCmdmsg(ros::Time time);
};
};


#endif //SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
