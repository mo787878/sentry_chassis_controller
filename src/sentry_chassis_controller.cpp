//
// Created by abc on 25-11-25.
//

#include "sentry_chassis_controller/sentry_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace sentry_chassis_controller {

    SentryChassisController::SentryChassisController()
            : tf_listener_(tf_buffer_) {}
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
        wheel_radius_ = controller_nh.param("wheel_radius", 0.07625);

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
        controller_nh.param("pid/i_", i_, 0.005);
        controller_nh.param("pid/d_", d_, 0.0);
        controller_nh.param("pid/i_max_", i_max_, 1.0);
        controller_nh.param("pid/i_min_", i_min_, -1.0);

        controller_nh.param("pid/p_wheel", p_wheel, 0.5);
        controller_nh.param("pid/i_wheel", i_wheel, 0.005);
        controller_nh.param("pid/d_wheel", d_wheel, 0.0);
        controller_nh.param("pid/i_max_wheel", i_max_wheel, 1.0);
        controller_nh.param("pid/i_min_wheel", i_min_wheel, -1.0);



        pid_lf_.initPid(p_, i_, d_, i_max_, i_min_);
        pid_rf_.initPid(p_, i_, d_, i_max_, i_min_);
        pid_lb_.initPid(p_, i_, d_, i_max_, i_min_);
        pid_rb_.initPid(p_, i_, d_, i_max_, i_min_);

        pid_lf_wheel_.initPid(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);
        pid_rf_wheel_.initPid(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);
        pid_lb_wheel_.initPid(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);
        pid_rb_wheel_.initPid(p_wheel, i_wheel, d_wheel, i_max_wheel, i_min_wheel);


        last_publish_time_ = ros::Time::now();
        publish_interval_ = ros::Duration(1.0);  // 发布频率10Hz（可调整）

        dyn_reconfig_callback_ = boost::bind(&SentryChassisController::reconfigureCallback, this, _1, _2);
        dyn_reconfig_server_.reset(new dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisParamsConfig>(controller_nh));
        dyn_reconfig_server_->setCallback(dyn_reconfig_callback_);

        odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom", 10);
        ROS_INFO("里程计发布者初始化成功！");

        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;
        last_update_time_ = ros::Time::now();

        // 读取速度控制模式参数
        controller_nh.param("use_global_vel_mode", use_global_vel_mode_, true);
        // 输出模式信息
        if (use_global_vel_mode_) {
            ROS_INFO("使用全局坐标系速度模式");
        } else {
            ROS_INFO("使用底盘坐标系速度模式");
        }

        return true;

    }

    geometry_msgs::Twist SentryChassisController::transformGlobalToBase(const geometry_msgs::Twist& global_twist, const ros::Time& time) {
        geometry_msgs::Twist base_twist = global_twist;  // 默认返回原速度

        try {
            // 获取从控制坐标系到底盘坐标系的变换
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                    "base_link", "odom", time, ros::Duration(0.1));

            // 提取旋转分量(仅yaw角)
            tf2::Quaternion q(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // 应用坐标变换: 世界坐标系 -> 底盘坐标系
            // 仅考虑平面运动(x, y, yaw)
            double vx = global_twist.linear.x * cos(yaw) + global_twist.linear.y * sin(yaw);
            double vy = global_twist.linear.y * cos(yaw) - global_twist.linear.x * sin(yaw);

            base_twist.linear.x = vx;
            base_twist.linear.y = vy;
            // 角速度不需要变换，因为是绕z轴的旋转

        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "无法获取TF变换: %s", ex.what());
        }

        return base_twist;
    }
    void SentryChassisController::reconfigureCallback(sentry_chassis_controller::SentryChassisParamsConfig &config, uint32_t level) {
        // 1. 更新舵机PID参数：调用setGains(p, i, d, i_max, i_min, antiwindup=true)
        // control_toolbox::Pid的setGains参数顺序：p, i, d, i_max, i_min, antiwindup
        pid_lf_.setGains(config.p_, config.i_, config.d_, config.i_max_, config.i_min_, true);
        pid_rf_.setGains(config.p_, config.i_, config.d_, config.i_max_, config.i_min_, true);
        pid_lb_.setGains(config.p_, config.i_, config.d_, config.i_max_, config.i_min_, true);
        pid_rb_.setGains(config.p_, config.i_, config.d_, config.i_max_, config.i_min_, true);

        // 2. 更新轮系PID参数
        pid_lf_wheel_.setGains(config.p_wheel, config.i_wheel, config.d_wheel, config.i_max_wheel, config.i_min_wheel, true);
        pid_rf_wheel_.setGains(config.p_wheel, config.i_wheel, config.d_wheel, config.i_max_wheel, config.i_min_wheel, true);
        pid_lb_wheel_.setGains(config.p_wheel, config.i_wheel, config.d_wheel, config.i_max_wheel, config.i_min_wheel, true);
        pid_rb_wheel_.setGains(config.p_wheel, config.i_wheel, config.d_wheel, config.i_max_wheel, config.i_min_wheel, true);

        // 3. 日志输出（可选：验证参数更新）
        setlocale(LC_ALL,"");
        ROS_INFO("[动态更新] 舵机PID参数：P=%.3f, I=%.3f, D=%.3f, I_max=%.3f, I_min=%.3f",
                 config.p_, config.i_, config.d_, config.i_max_, config.i_min_);
        ROS_INFO("[动态更新] 轮系PID参数：P=%.3f, I=%.3f, D=%.3f, I_max=%.3f, I_min=%.3f",
                 config.p_wheel, config.i_wheel, config.d_wheel, config.i_max_wheel, config.i_min_wheel);
    }
    void SentryChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        if (use_global_vel_mode_) {
            // 全局速度模式：转换到底盘坐标系
            ros::Time current_time = ros::Time::now();
            cmd_vel_ = transformGlobalToBase(*msg, current_time);
        } else {
            // 底盘速度模式：直接使用原始指令
            cmd_vel_ = *msg;
        }

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



        ros::Duration dt = time - last_update_time_;
        if (dt.toSec() <= 0.0) {
            last_update_time_ = time;
            return;
        }
        double fl_vel_get = front_left_wheel_joint_.getVelocity();
        double fr_vel_get = front_right_wheel_joint_.getVelocity();
        double bl_vel_get = back_left_wheel_joint_.getVelocity();
        double br_vel_get = back_right_wheel_joint_.getVelocity();

        double fl_angle_get = front_left_pivot_joint_.getPosition();
        double fr_angle_get = front_right_pivot_joint_.getPosition();
        double bl_angle_get = back_left_pivot_joint_.getPosition();
        double br_angle_get = back_right_pivot_joint_.getPosition();

        // 2. 将轮子角速度转换为线速度（m/s）
        double fl_linear_get = fl_vel_get * wheel_radius_;
        double fr_linear_get = fr_vel_get * wheel_radius_;
        double bl_linear_get = bl_vel_get * wheel_radius_;
        double br_linear_get = br_vel_get * wheel_radius_;

        // 3. 计算每个轮子在机体坐标系下的速度分量
        double fl_vx_get = fl_linear * cos(fl_angle_get);  // 前左轮x方向速度
        double fl_vy_get = fl_linear * sin(fl_angle_get);  // 前左轮y方向速度
        double fr_vx_get = fr_linear * cos(fr_angle_get);
        double fr_vy_get = fr_linear * sin(fr_angle_get);
        double bl_vx_get = bl_linear * cos(bl_angle_get);
        double bl_vy_get = bl_linear * sin(bl_angle_get);
        double br_vx_get = br_linear * cos(br_angle_get);
        double br_vy_get = br_linear * sin(br_angle_get);

        // 4. 计算底盘速度（取四个轮子的平均值）
        double vx_get = (fl_vx_get + fr_vx_get + bl_vx_get + br_vx_get) / 4.0;  // 机体x方向速度
        double vy_get = (fl_vy_get + fr_vy_get + bl_vy_get + br_vy_get) / 4.0;  // 机体y方向速度

        // 5. 计算角速度（基于轮子速度差）
        // 前轴平均速度
        double front_v_get = (fl_linear_get + fr_linear_get) / 2.0;
        // 后轴平均速度
        double rear_v_get = (bl_linear_get + br_linear_get) / 2.0;
        // 左右轮平均速度差
        double left_v_get = (fl_linear_get + bl_linear_get) / 2.0;
        double right_v_get = (fr_linear_get + br_linear_get) / 2.0;

        // 合成角速度（取两种计算方式的平均值提高精度）
        double wz1_get = (rear_v_get - front_v_get) / wheel_base_;               // 前后轴速度差计算
        double wz2_get = (right_v_get - left_v_get) / wheel_track_;              // 左右轮速度差计算
        double wz_get = (wz1_get + wz2_get) / 2.0;                               // 角速度平均值

        // 6. 积分计算里程计位置和航向
        double dt_sec = dt.toSec();
        yaw_ += wz_get * dt_sec;  // 更新航向角（弧度）

        // 转换为世界坐标系下的位移
        double dx = (vx_get * cos(yaw_) - vy_get * sin(yaw_)) * dt_sec;
        double dy = (vx_get * sin(yaw_) + vy_get * cos(yaw_)) * dt_sec;

        x_ += dx;  // 更新x坐标
        y_ += dy;  // 更新y坐标

        // 7. 发布Odometry消息
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // 设置位置
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // 设置姿态（仅yaw有值，转换为四元数）
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // 设置速度
        odom_msg.twist.twist.linear.x = vx_get;
        odom_msg.twist.twist.linear.y = vy_get;
        odom_msg.twist.twist.angular.z = wz_get;

        odom_pub_.publish(odom_msg);

        // 8. 发布tf变换（odom -> base_link）
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = time;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";

        transform_stamped.transform.translation.x = x_;
        transform_stamped.transform.translation.y = y_;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster_.sendTransform(transform_stamped);

        last_update_time_ = time;
    }
    PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)
}