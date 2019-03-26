#ifndef VSSS_SIMU_SIMUROBOTIDS_H
#define VSSS_SIMU_SIMUROBOTIDS_H

#include <string>
#include <mujoco.h>
#include <SimuRobotControl.h>

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
public:
	SimuRobotIDs ids;
	SimuRobotControl control{};

	SimuRobot() = default;
	SimuRobot(mjModel *model, int id): ids(model, id) {}
};

#endif //VSSS_SIMU_SIMUROBOTIDS_H
