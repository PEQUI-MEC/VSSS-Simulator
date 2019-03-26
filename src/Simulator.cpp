#include "Simulator.h"

Simulator::Simulator(const std::string &key_path,
                     const std::string &model_path) {
//	Initialize Mujoco
	mj_activate(key_path.c_str());

	model = mj_loadXML(model_path.c_str(), nullptr, nullptr, 1000);
	if (!model) throw std::string("Could not load model");

	data = mj_makeData(model);
	if (!data) throw std::string("Could not load data");

	if (!glfwInit()) throw "Could not initialize GLFW";

//	initialize visualization data structures
	mjv_defaultCamera(&cam);
	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);

//	create scene and context
	mjv_makeScene(model, &scn, 2000);

	add_robots();
	ball_id = mj_name2id(model, mjOBJ_BODY, "ball");

	last_control_update = data->time;
}

void Simulator::step() {
	mjtNum sim_start = data->time;
	while (data->time - sim_start < 1.0 / 60.0) {
//		Run control for each robot
		double elapsed = data->time - last_control_update;
		if (elapsed >= 0.01) {
			run_robot_control(elapsed);
			last_control_update = data->time;
		}

//		Step simulator
		mj_step(model, data);
	}

	mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
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

Point Simulator::get_ball() {
	return Point::from_simulator_point(data->xpos[ball_id * 3],
									   data->xpos[ball_id * 3 + 1]);
}

void Simulator::add_robots() {
	robots[0] = SimuRobot{model, 1};
	robots[1] = SimuRobot{model, 2};
	robots[2] = SimuRobot{model, 3};
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

