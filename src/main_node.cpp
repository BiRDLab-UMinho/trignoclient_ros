#include <ros/ros.h>
#include "trignoclient/network.hpp"
#include "Interface.hpp"
#include "DataPublisher.hpp"

using namespace trigno;

void interruptHandler(int signum) {
    ros::shutdown();
}


int main(int argc, char const *argv[]) {
    signal(SIGINT, interruptHandler);

    ros::init(argc, argv, "trignoclient_ros", ros::init_options::NoSigintHandler);
    ros::NodeHandle handle("~");

    // instantiate idle/unitialized Client
    trigno::network::Client trigno;

    // parametrize & initialize client from ROS parameters
    trigno::ros::initialize(&trigno);

    // instantiate ROS wrapper classes
    ros::InterfaceWrapper interface(&handle, &trigno, "trigno");
    ros::DataWrapper emg(&handle, &trigno.EMG, "emg");
    ros::DataWrapper aux(&handle, &trigno.AUX, "aux");


    while (ros::ok()) {
        ros::spinOnce();
    }
    // ros::spin();

    ros::shutdown();

    return 0;
}
