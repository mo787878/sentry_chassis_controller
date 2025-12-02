//
// Created by abc on 25-11-25.
//

#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
//#include <sentry_chassis_controller/SentryChassisParamsConfig.h>

namespace sentry_chassis_controller {
class SentryChassisController : public controller_interface ::Controller<hardware_interface::EffortJointInterface>{
public:
    SentryChassisController() = default;
    ~SentryChassisController() override = default;

    bool init(hardware_interface::EffortJointInterface* effortJointInterface,
              ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void update(const ros::Time &time, const ros::Duration &Period) override;

    //void  reconfigureCallback(sentry_chassis_controller::SentryChassisParamsConfig &config, uint32_t level);


    hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
    hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_,
                                    back_left_wheel_joint_, back_right_wheel_joint_;
private:
    ros::Time last_change_;
    double wheel_track_;
    double wheel_base_;
    control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;
    control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_;
    double p_, i_, d_, i_max_, i_min_, p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel, chassis_speed;
    //dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisParamsConfig> dyn_srv_;
    //bool params_initialized_ = false;
};
}

#endif //SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
