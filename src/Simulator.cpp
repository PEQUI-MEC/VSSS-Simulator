#include "Simulator.h"
#include <utility>

Simulator::Simulator(const std::string &key_path,
                     const std::string &model_path,
					 ros::NodeHandle& nh) : nh(nh) {
//	Initialize Mujoco
	mj_activate(key_path.c_str());
	load_model(6, model_path);

	if (!glfwInit()) throw std::runtime_error("Could not initialize GLFW");

//	initialize visualization data structures
	mjv_defaultCamera(&cam);
	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);

//	create scene and context
	mjv_makeScene(model, &scn, 2000);

    add_robots(6);
	ball = Ball(mj_name2id(model, mjOBJ_BODY, "ball"), nh);

	last_control_update = data->time;
}

void Simulator::load_model(int robot_count, const std::string &model_path) {
    model = mj_loadXML(model_path.c_str(), nullptr, nullptr, 1000);
    if (!model) throw std::runtime_error("Could not load model");
    data = mj_makeData(model);
    if (!data) throw std::runtime_error("Could not load data");
    add_robots(robot_count);
    stop_robots();
}

void Simulator::step() {
	mjtNum sim_start = data->time;
	while (data->time - sim_start < 1.0 / 60.0) {
//		Run control for each robot
		double elapsed = data->time - last_control_update;
		if (elapsed >= 0.01) {
//			Run callbacks for each robot
			ros::spinOnce();
//			Run control for each robot
			run_robot_control(elapsed);
			last_control_update = data->time;
		}

//		Step simulator
		mj_step(model, data);
	}

	mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);

	auto now = ros::Time::now();
    ball.publish_position(seq, now, get_ball());
	for (auto& [id, robot] : robots) {
        robot.publish_pose(seq, now, get_quaternion(id));
	}
	seq++;
}


void Simulator::run_robot_control(double time) {
	for (auto& r : robots) {
		auto id = r.first;
		auto& robot = r.second;
		auto pos = get_position(id);
		auto theta = get_orientation(id);
		auto wheel_vel = get_velocity(id);
		auto target_wheel_vel = robot.control.control_step(pos, theta, wheel_vel, (float) time);
		set_velocity(id, target_wheel_vel);
	}
}


void Simulator::set_velocity(int robot_id, WheelVelocity vel) {
	auto& r = robots[robot_id];
	data->ctrl[r.ids.left_wheel_id] = vel.left / 0.03f;
	data->ctrl[r.ids.right_wheel_id] = vel.right / 0.03f;
}

WheelVelocity Simulator::get_velocity(int robot_id) {
	auto& r = robots[robot_id];
	return { (float) data->actuator_velocity[r.ids.left_wheel_id] * 0.03f,
	         (float) data->actuator_velocity[r.ids.right_wheel_id] * 0.03f };
}

Point Simulator::get_position(int robot_id) {
	auto id = robots[robot_id].ids.body_id;
	return Point::from_simulator_point(data->xpos[id * 3],
	                                   data->xpos[id * 3 + 1]);
}

float Simulator::get_orientation(int robot_id) {
	auto id = robots[robot_id].ids.body_id;
	auto a = data->xquat[id * 4];
	auto b = data->xquat[id * 4 + 1];
	auto c = data->xquat[id * 4 + 2];
	auto d = data->xquat[id * 4 + 3];
    return (float) std::atan2(2 * (a * d + b * c),
                              1 - 2 * (std::pow(c, 2) + std::pow(d, 2)));
}

tf2::Quaternion Simulator::get_quaternion(int robot_id) {
    auto id = robots[robot_id].ids.body_id;
    auto w = data->xquat[id * 4];
    auto x = data->xquat[id * 4 + 1];
    auto y = data->xquat[id * 4 + 2];
    auto z = data->xquat[id * 4 + 3];
    return {x, y, z, w};
}

Point Simulator::get_ball() {
	return Point::from_simulator_point(data->xpos[ball.id * 3],
									   data->xpos[ball.id * 3 + 1]);
}

void Simulator::add_robots(int robot_count) {
    robots.clear();
//	emplace is required due to ROS storing pointers for each robot
//	during the execution of their constructors
    for (int i = 0; i < robot_count; i++)
        robots.try_emplace(i, model, i + 1, nh);
}

void Simulator::stop_robots() {
    for (auto& [i, robot] : robots) {
        robot.control.stop();
    }
    last_control_update = 0;
}

Simulator::~Simulator() {
//	free visualization storage
	mjv_freeScene(&scn);
	mjr_freeContext(&con);

//	free MuJoCo model and data, deactivate
	mj_deleteData(data);
	mj_deleteModel(model);
	mj_deactivate();
}

