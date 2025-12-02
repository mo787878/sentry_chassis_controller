//
// Created by abc on 25-11-25.
//

#include "sentry_chassis_controller/sentry_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace sentry_chassis_controller{
    /*   void SentryChassisController::reconfigureCallback(sentry_chassis_controller::SentryChassisParamsConfig &config, uint32_t level){
          if(params_initialized_) {
              p_ = config.p_;
              p_wheel = config.p_wheel;
              i_ = config.i_;
              i_wheel = config.i_wheel;
              d_ = config.d_;
              d_wheel = config.d_wheel;
              i_max_ = config.i_max_;
              i_max_wheel = config.i_max_wheel;
              i_min_ = config.i_min_;
              i_min_wheel = config.i_min_wheel;
              chassis_speed = config.chassis_speed;
   //注释这段代码有问题
            pid_lf_.setGains(p_, i_, d_, i_max_, i_min_);
              pid_rf_.setGains(p_, i_, d_, i_max_, i_min_);
              pid_lb_.setGains(p_, i_, d_, i_max_, i_min_);
              pid_rb_.setGains(p_, i_, d_, i_max_, i_min_);

              pid_lf_wheel_.setGains(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);
              pid_rf_wheel_.setGains(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);
              pid_lb_wheel_.setGains(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);
              pid_rb_wheel_.setGains(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);

           setlocale(LC_ALL,"");
           ROS_INFO("动态参数更新");
       }
    }
*/
    bool SentryChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                       ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){

        front_left_wheel_joint_ =
                effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_wheel_joint_ =
                effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_wheel_joint_ =
                effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_wheel_joint_ =
                effort_joint_interface->getHandle("right_back_wheel_joint");

        front_left_pivot_joint_ =
                effort_joint_interface->getHandle("left_front_pivot_joint");
        front_right_pivot_joint_ =
                effort_joint_interface->getHandle("right_front_pivot_joint");
        back_left_pivot_joint_ =
                effort_joint_interface->getHandle("left_back_pivot_joint");
        back_right_pivot_joint_ =
                effort_joint_interface->getHandle("right_back_pivot_joint");

        wheel_track_ = controller_nh.param("wheel_track", 0.362);
        wheel_base_ = controller_nh.param("wheel_base", 0.362);

        controller_nh.param("pid/p_", p_, 0.5);
        controller_nh.param("pid/i_", i_, 0.0);
        controller_nh.param("pid/d_", d_, 0.0);
        controller_nh.param("pid/i_max_", i_max_, 1.0);
        controller_nh.param("pid/i_min_", i_min_, -1.0);

        controller_nh.param("pid/p_wheel", p_wheel, 0.5);
        controller_nh.param("pid/i_wheel", i_wheel, 0.0);
        controller_nh.param("pid/d_wheel", d_wheel, 0.0);
        controller_nh.param("pid/i_max_wheel", i_max_wheel, 1.0);
        controller_nh.param("pid/i_min_wheel", i_min_wheel, -1.0);
        controller_nh.param("pid/chassis_speed", chassis_speed, 8.0);

        //this->params_initialized_ = true;

        pid_lf_.initPid(p_, i_, d_, i_max_, i_min_);
        pid_rf_.initPid(p_, i_, d_, i_max_, i_min_);
        pid_lb_.initPid(p_, i_, d_, i_max_, i_min_);
        pid_rb_.initPid(p_, i_, d_, i_max_, i_min_);

        pid_lf_wheel_.initPid(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);
        pid_rf_wheel_.initPid(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);
        pid_lb_wheel_.initPid(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);
        pid_rb_wheel_.initPid(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);

 /*       ros::NodeHandle dyn_nh(controller_nh, "dynamic_params");
        dyn_srv_.setCallback(boost::bind(&SentryChassisController::reconfigureCallback, this, _1, _2));
*/
        return true;
    }
    void SentryChassisController::update(const ros::Time &time, const ros::Duration &period) {
        if ((time - last_change_).toSec() > 8) {

            last_change_ = time;
        }
        front_left_wheel_joint_.setCommand(pid_lf_wheel_.computeCommand( chassis_speed - front_left_wheel_joint_.getVelocity(), period));
        front_right_wheel_joint_.setCommand(pid_rf_wheel_.computeCommand( chassis_speed - front_right_wheel_joint_.getVelocity(), period));
        back_left_wheel_joint_.setCommand(pid_lb_wheel_.computeCommand( chassis_speed - back_left_wheel_joint_.getVelocity(), period));
        back_right_wheel_joint_.setCommand(pid_rb_wheel_.computeCommand( chassis_speed - back_right_wheel_joint_.getVelocity(), period));

        front_left_pivot_joint_.setCommand(pid_lf_.computeCommand( 0.0 - front_left_pivot_joint_.getPosition(), period));
        front_right_pivot_joint_.setCommand(pid_rf_.computeCommand( 0.0 - front_right_pivot_joint_.getPosition(), period));
        back_left_pivot_joint_.setCommand(pid_lb_.computeCommand( 0.0 - back_left_pivot_joint_.getPosition(), period));
        back_right_pivot_joint_.setCommand(pid_rb_.computeCommand( 0.0 - back_right_pivot_joint_.getPosition(), period));


}
    PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
}