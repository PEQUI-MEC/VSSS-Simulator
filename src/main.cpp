#include "Simulator.h"
#include "SimulatorGUI.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

void on_command_received(const std_msgs::String &msg) {
    std::cout << "Received: " << msg.data << std::endl;
}

int main(int argc, char **argv) {
    auto sim_thread = std::thread([]() {
        Simulator simulator("../mjkey.txt", "../src/scene_1team.xml");
        SimulatorGUI gui(simulator);
        gui.run();
    });

    ros::init(argc, argv, "vsss_simulator");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("ros_test_receive", 2, on_command_received);
    ros::Publisher pub = n.advertise<std_msgs::String>("ros_test_send", 2);
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = std::to_string(count);
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

//    ros::spin();
    sim_thread.join();
}
