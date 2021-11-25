#include <ros/ros.h>
#include "trignoclient/network.hpp"
#include "TrignoInterface.hpp"
#include "TrignoDataPublisher.hpp"


void interruptHandler(int signum) {
    ros::shutdown();
}


void initialize(trigno::network::Client* client) {
    // parse ROS parameters

    //
    client->initialize(address, timeout)
}


int main(int argc, char const *argv[]) {
    signal(SIGINT, interruptHandler);

    ros::init(argc, argv, "trignoclient", ros::init_options::NoSigintHandler);
    ros::NodeHandle handle("~");

    // instantiate idle/unitialized Client     
    trigno::network::Client trigno;
    // parametrize & initialize client from ROS parameters
    initialize(&trigno);

    // instantiate ROS wrapper classes
    trigno::ros::Interface interface(&handle, &trigno, "trigno");
    trigno::ros::DataPublisher emg(&handle, &trigno.EMG, "emg");
    trigno::ros::DataPublisher aux(&handle, &trigno.AUX, "aux");

    while (ros::ok()) {
        ros::spinOnce();
    }
    // ros::spin();

    ros::shutdown();

    return 0;
}
