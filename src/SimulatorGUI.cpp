#include <iostream>
#include "SimulatorGUI.h"

Simulator *SimulatorGUI::simulator = nullptr;
bool SimulatorGUI::button_left = false;
bool SimulatorGUI::button_middle = false;
bool SimulatorGUI::button_right = false;
double SimulatorGUI::lastx = 0;
double SimulatorGUI::lasty = 0;

SimulatorGUI::SimulatorGUI(Simulator &simulator) {
	SimulatorGUI::simulator = &simulator;

//	create window, make OpenGL context current, request v-sync
	window = glfwCreateWindow(1024, 720, "VSSS", nullptr, nullptr);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	mjr_makeContext(simulator.model, &simulator.con, mjFONTSCALE_150);

//  install GLFW mouse and keyboard callbacks
	glfwSetKeyCallback(window, keyboard_callback);
	glfwSetCursorPosCallback(window, mouse_move_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetScrollCallback(window, scroll_callback);
}

float to_rads(double val) {
	return float(val * M_PI / 180.0);
}

void SimulatorGUI::run() {
	while (!glfwWindowShouldClose(window)) {
		simulator->step();

		// get framebuffer viewport
		mjrRect viewport = {0, 0, 0, 0};
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

		// Render scene
		mjr_render(viewport, &simulator->scn, &simulator->con);

		// swap OpenGL buffers (blocking call due to v-sync)
		glfwSwapBuffers(window);

//		auto ball = simulator->get_ball();
//		auto direction = Point{ball.x + 0.1f, ball.y + 0.1f};
//        auto direction2 = Point{ball.x - 0.1f, ball.y - 0.1f};

//        simulator->robots[0].control.set_target(Command::UVF,
//                                                {ball, to_rads(45), 0.8, 7, direction, 2});

		// process pending GUI events, call GLFW callbacks
		glfwPollEvents();
	}
}

void SimulatorGUI::keyboard_callback(GLFWwindow *window, int key, int scancode, int act, int mods) {
//	backspace: reset simulation
	if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
		mj_resetData(simulator->model, simulator->data);
		mj_forward(simulator->model, simulator->data);
	}
}


void SimulatorGUI::mouse_button_callback(GLFWwindow *window, int button, int act, int mods) {
	// update button state
	button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
	button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
	button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

	// update mouse position
	glfwGetCursorPos(window, &lastx, &lasty);
}

void SimulatorGUI::mouse_move_callback(GLFWwindow *window, double xpos, double ypos) {
	// no buttons down: nothing to do
	if (!button_left && !button_middle && !button_right)
		return;

	// compute mouse displacement, save
	double dx = xpos - lastx;
	double dy = ypos - lasty;
	lastx = xpos;
	lasty = ypos;

	// get current window size
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// get shift key state
	bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
	                  glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

	// determine action based on mouse button
	mjtMouse action;
	if (button_right)
		action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	else if (button_left)
		action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	else
		action = mjMOUSE_ZOOM;

	// move camera
	mjv_moveCamera(simulator->model, action, dx / height, dy / height, &simulator->scn, &simulator->cam);
}

void SimulatorGUI::scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
	// emulate vertical mouse motion = 5% of window height
	mjv_moveCamera(simulator->model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &simulator->scn, &simulator->cam);
}
