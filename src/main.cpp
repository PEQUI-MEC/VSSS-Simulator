#include "Simulator.h"
#include "SimulatorGUI.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vsss_simu");
    ros::NodeHandle nh;

    Simulator simulator("../mjkey.txt", "../src/scene_2teams.xml", nh);
    SimulatorGUI gui(simulator);
    gui.run();
}
