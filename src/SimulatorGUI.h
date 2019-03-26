#ifndef VSSS_SIMU_SIMULATORGUI_H
#define VSSS_SIMU_SIMULATORGUI_H

#include "Simulator.h"

class SimulatorGUI {
	GLFWwindow *window;
	static Simulator *simulator;

	static bool button_left;
	static bool button_middle;
	static bool button_right;
	static double lastx;
	static double lasty;

	static void keyboard_callback(GLFWwindow *window, int key, int scancode, int act, int mods);
	static void mouse_button_callback(GLFWwindow *window, int button, int act, int mods);
	static void mouse_move_callback(GLFWwindow *window, double xpos, double ypos);
	static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

public:
	explicit SimulatorGUI(Simulator &simulator);

	void run();
};


#endif //VSSS_SIMU_SIMULATORGUI_H
