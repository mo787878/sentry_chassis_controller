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
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

namespace sentry_chassis_controller {
class SentryChassisController : public controller_interface ::Controller<hardware_interface::EffortJointInterface>{
public:
    SentryChassisController() = default;
    ~SentryChassisController() override = default;

    bool init(hardware_interface::EffortJointInterface* effortJointInterface,
              ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void update(const ros::Time &time, const ros::Duration &Period) override;



    hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
    hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_,
                                    back_left_wheel_joint_, back_right_wheel_joint_;
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

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    boost::shared_ptr<dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisParamsConfig>> dyn_reconfig_server_;
    dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisParamsConfig>::CallbackType dyn_reconfig_callback_;

    void reconfigureCallback(sentry_chassis_controller::SentryChassisParamsConfig &config, uint32_t level);
};
}

#endif //SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
