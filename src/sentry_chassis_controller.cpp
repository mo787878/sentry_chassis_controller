//
// Created by abc on 25-11-25.
//

#include "sentry_chassis_controller/sentry_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace sentry_chassis_controller{
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
        return true;
    }
    void SentryChassisController::update(const ros::Time &time, const ros::Duration &period) {





}
    PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
}