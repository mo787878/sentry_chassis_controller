//
// Created by abc on 25-11-25.
//

#include "sentry_chassis_controller/sentry_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace sentry_chassis_controller {
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
    bool SentryChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                       ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {

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
        wheel_radius_ = controller_nh.param("wheel_radius", 0.05);
        cmd_vel_pub_ = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
        setlocale(LC_ALL,"");
        ROS_INFO("cmd_vel 发布者初始化成功！");
        cmd_vel_sub_ = root_nh.subscribe("/cmd_vel", 10, &SentryChassisController::cmdVelCallback, this);
        if (!cmd_vel_sub_) {
            ROS_ERROR("ERROR: /cmd_vel 订阅者创建失败！");
            return false;
        }
        ROS_INFO("cmd_vel 订阅者初始化成功！正在监听话题...");

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
        last_publish_time_ = ros::Time::now();
        publish_interval_ = ros::Duration(0.1);  // 发布频率10Hz（可调整）

        return true;
    }

    void SentryChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        cmd_vel_ = *msg;
        static ros::Time last_log_time = ros::Time::now();
        if ((ros::Time::now() - last_log_time).toSec() > 1.0) {
            setlocale(LC_ALL,"");
            ROS_INFO("成功收到 /cmd_vel 消息：vx=%.2f m/s, vy=%.2f m/s, wz=%.2f rad/s",
                     msg->linear.x, msg->linear.y, msg->angular.z);
            last_log_time = ros::Time::now();
        }
    }

    void SentryChassisController::update(const ros::Time &time, const ros::Duration &period) {
        if ((time - last_publish_time_) >= publish_interval_) {
            geometry_msgs::Twist cmd_msg;
            // 配置发布的速度指令（示例：缓慢前进+轻微转向，可根据需求修改）
            cmd_msg.linear.x = 0.0;    // 前进速度0.2m/s
            cmd_msg.linear.y = 0.0;    // 横向速度0
            cmd_msg.angular.z = 3.0;   // 转向角速度0.1rad/s（缓慢右转）

            // 发布消息
            cmd_vel_pub_.publish(cmd_msg);
            setlocale(LC_ALL,"");
            ROS_DEBUG("主动发布 /cmd_vel 消息：vx=%.2f, wz=%.2f", cmd_msg.linear.x, cmd_msg.angular.z);

            // 更新发布时间
            last_publish_time_ = time;
        }




        double vx = cmd_vel_.linear.x;    // 前进/后退速度（x轴，m/s）
            double vy = cmd_vel_.linear.y;    // 左右平移速度（y轴，m/s）
            double wz = cmd_vel_.angular.z;   // 旋转角速度（z轴，rad/s）

            // 第二步：计算每个轮子的旋转半径（精确版：直角三角形斜边）
            double half_wheel_base = wheel_base_ / 2.0;    // 底盘中心到前后轮的距离
            double half_wheel_track = wheel_track_ / 2.0;  // 底盘中心到左右轮的距离

            // 第三步：逆运动学计算——每个轮子的 目标线速度（轮机） 和 目标转向角（舵机）
            // 轮子布局：前左(fl)、前右(fr)、后左(bl)、后右(br)
            double fl_linear, fr_linear, bl_linear, br_linear;  // 轮子线速度（m/s）
            double fl_angle, fr_angle, bl_angle, br_angle;      // 舵机转向角（rad，0=向前）

            // -------------------------- 前左轮（fl） --------------------------
            // 1. 计算轮子的速度矢量（x、y方向分量）
            double fl_vx = vx - wz * half_wheel_base;   // 前左轮x方向速度分量
            double fl_vy = vy + wz * half_wheel_track;  // 前左轮y方向速度分量（注意符号：左轮+，右轮-）
            // 2. 计算舵机转向角：速度矢量与x轴的夹角（atan2(vy, vx)）
            fl_angle = atan2(fl_vy, fl_vx);  // 范围：[-π, π]，对应转向角（向左为正，向右为负）
            // 3. 计算轮子线速度：速度矢量的模长（sqrt(vx²+vy²)）
            fl_linear = sqrt(pow(fl_vx, 2) + pow(fl_vy, 2));

            // -------------------------- 前右轮（fr） --------------------------
            double fr_vx = vx - wz * half_wheel_base;
            double fr_vy = vy - wz * half_wheel_track;  // 右轮vy分量符号为负
            fr_angle = atan2(fr_vy, fr_vx);
            fr_linear = sqrt(pow(fr_vx, 2) + pow(fr_vy, 2));

            // -------------------------- 后左轮（bl） --------------------------
            double bl_vx = vx + wz * half_wheel_base;   // 后轮vx分量符号与前轮相反
            double bl_vy = vy + wz * half_wheel_track;
            bl_angle = atan2(bl_vy, bl_vx);
            bl_linear = sqrt(pow(bl_vx, 2) + pow(bl_vy, 2));

            // -------------------------- 后右轮（br） --------------------------
            double br_vx = vx + wz * half_wheel_base;
            double br_vy = vy - wz * half_wheel_track;
            br_angle = atan2(br_vy, br_vx);
            br_linear = sqrt(pow(br_vx, 2) + pow(br_vy, 2));

            // 第四步：线速度 → 角速度（轮机目标转速，rad/s）
            double fl_target_vel = fl_linear / wheel_radius_;
            double fr_target_vel = fr_linear / wheel_radius_;
            double bl_target_vel = bl_linear / wheel_radius_;
            double br_target_vel = br_linear / wheel_radius_;

            // 第五步：控制指令下发——轮机（转速控制）+ 舵机（角度控制）
            // 1. 轮机控制（原有逻辑，不变）
            front_left_wheel_joint_.setCommand(
                    pid_lf_wheel_.computeCommand(fl_target_vel - front_left_wheel_joint_.getVelocity(), period));
            front_right_wheel_joint_.setCommand(
                    pid_rf_wheel_.computeCommand(fr_target_vel - front_right_wheel_joint_.getVelocity(), period));
            back_left_wheel_joint_.setCommand(
                    pid_lb_wheel_.computeCommand(bl_target_vel - back_left_wheel_joint_.getVelocity(), period));
            back_right_wheel_joint_.setCommand(
                    pid_rb_wheel_.computeCommand(br_target_vel - back_right_wheel_joint_.getVelocity(), period));

            // 2. 舵机控制（新增逻辑：跟踪目标转向角）
            front_left_pivot_joint_.setCommand(
                    pid_lf_.computeCommand(fl_angle - front_left_pivot_joint_.getPosition(), period));
            front_right_pivot_joint_.setCommand(
                    pid_rf_.computeCommand(fr_angle - front_right_pivot_joint_.getPosition(), period));
            back_left_pivot_joint_.setCommand(
                    pid_lb_.computeCommand(bl_angle - back_left_pivot_joint_.getPosition(), period));
            back_right_pivot_joint_.setCommand(
                    pid_rb_.computeCommand(br_angle - back_right_pivot_joint_.getPosition(), period));
        }




    PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
}