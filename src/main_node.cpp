#include <ros/ros.h>
#include <trignoclient/trignoclient.hpp>
#include "interface_wrapper.hpp"
#include "data_wrapper.hpp"
#include "bindings.hpp"

void interruptHandler(int signum) {
    ros::shutdown();
}


int main(int argc, char *argv[]) {
    signal(SIGINT, interruptHandler);

    ros::init(argc, argv, "trignoclient_ros", ros::init_options::NoSigintHandler);
    ros::NodeHandle handle("~");

    // instantiate idle/unitialized Client
    trigno::network::Client trigno;

    // parametrize & initialize client from ROS parameters
    // ros::initializtrignoclient_e(:: &trigno);

    // instantiate ROS wrapper classes
    trignoclient_ros::InterfaceWrapper interface(&handle, &trigno, "trigno");
    trignoclient_ros::DataWrapper emg(&handle, &trigno.EMG, "emg");
    trignoclient_ros::DataWrapper aux(&handle, &trigno.AUX, "aux");

    while (ros::ok()) {
        ros::spinOnce();
    }
    // ros::spin();

    ros::shutdown();

    return 0;
}
