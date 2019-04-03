#ifndef VSSS_SIMU_SIMUROBOTIDS_H
#define VSSS_SIMU_SIMUROBOTIDS_H

#include <string>
#include <mujoco.h>
#include <SimuRobotControl.h>
#include <ros/ros.h>
//#include "vsss_simulator/vsss_control.h"
#include "vsss_msgs/Control.h"
#include "geometry_msgs/Pose2D.h"

class SimuRobotIDs {
public:
	int body_id = -1;
	int left_wheel_id = -1;
	int right_wheel_id = -1;

	SimuRobotIDs() = default;
	SimuRobotIDs(mjModel *model, int id) {
		auto id_str = std::to_string(id);
		body_id = mj_name2id(model, mjOBJ_BODY,
		                     ("robot_0" + id_str).c_str());
		left_wheel_id = mj_name2id(model, mjOBJ_ACTUATOR,
		                           ("l" + id_str).c_str());
		right_wheel_id = mj_name2id(model, mjOBJ_ACTUATOR,
		                            ("r" + id_str).c_str());
	}
};

class SimuRobot {
    using Twist = geometry_msgs::Twist;
    using Control = vsss_msgs::Control;
    using Pose2D = geometry_msgs::Pose2D;
public:
	SimuRobotIDs ids;
	SimuRobotControl control{};

    ros::Subscriber control_sub;
    ros::Publisher pose_pub;

    void on_command_received(const Control &msg) {
        control.set_target(static_cast<Command>(msg.command),
                           Target{{static_cast<float>(msg.pose.x),
                                   static_cast<float>(msg.pose.y)},
                                  static_cast<float>(msg.pose.theta),
                                  static_cast<float>(msg.velocity.linear.x),
                                  static_cast<float>(msg.velocity.angular.z)});
    }

    void publish_pose() {
        geometry_msgs::Pose2D pose_msg;
        pose_msg.x = control.position.x;
        pose_msg.y = control.position.y;
        pose_msg.theta = control.orientation;
        pose_pub.publish(pose_msg);
    }

	SimuRobot() = default;
	SimuRobot(mjModel *model, int id, ros::NodeHandle& nh): ids(model, id) {
        control_sub = nh.subscribe("robot" + std::to_string(id) + "/control", 1,
                &SimuRobot::on_command_received, this);
        pose_pub = nh.advertise<Pose2D>("robot" + std::to_string(id) + "/pose", 1);
	}
};

#endif //VSSS_SIMU_SIMUROBOTIDS_H
