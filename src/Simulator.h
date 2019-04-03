#ifndef VSSS_SIMU_SIMULATOR_H
#define VSSS_SIMU_SIMULATOR_H

#include <string>
#include <mujoco.h>
#include <thread>
#include <unordered_map>
#include <ostream>
#include <iostream>
#include "glfw3.h"
#include "SimuRobotControl.h"
#include "SimuRobot.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

struct Ball {
	int id = -1;
	ros::Publisher position_pub;

	Ball() = default;
	Ball(int id, ros::NodeHandle& nh): id(id) {
		position_pub = nh.advertise<geometry_msgs::Point>("ball/position", 1);
	}

	void publish_position(Point position) {
		geometry_msgs::Point position_msg;
		position_msg.x = position.x;
		position_msg.y = position.y;
		position_msg.z = 0;
		position_pub.publish(position_msg);
	}
};

class Simulator {
public:
	mjModel *model;  // MuJoCo model
	mjData *data;    // Mujoco data

	mjvCamera cam;              // abstract camera
	mjvOption opt;              // visualization options
	mjvScene scn;               // abstract scene
	mjrContext con;             // custom GPU context

	Ball ball;

	double last_control_update = 0;
	std::unordered_map<int, SimuRobot> robots{};

public:
	Simulator(const std::string &key_path, const std::string &model_path, ros::NodeHandle& nh);
	~Simulator();
	void step();
	void run_robot_control(double time);
	void set_velocity(int robot_id, WheelVelocity vel);
	WheelVelocity get_velocity(int robot_id);
	Point get_position(int robot_id);
	float get_orientation(int robot_id);
	Point get_ball();
	void add_robots(ros::NodeHandle& nh);
};


#endif //VSSS_SIMU_SIMULATOR_H
