#include <string>
#include <ros/ros.h>
#include "trignoclient/trignoclient.hpp"  // trigno::network::BasicDataClient
#include "bindings.hpp"                   // ros::msg
#include "recorder_publisher.hpp"

namespace trigno::ros {

RecorderPublisher::RecorderPublisher(ros::NodeHandle* node, trigno::network::BasicDataClient* data_client, trigno::Sequence* out, const std::string& name) :
    Recorder(data_client, out) {
        /* ... */
        _publisher = node->advertise(node->getNamespace() + "/" + name, 10 /* queue size */);
}



void RecorderPublisher::execute() {
    // read frame & append to sequence
    // delegate to base class method
    Recorder::execute();
    // publish last frame
    _publisher.publish(ros::msg(_sequence->back()));
}

}  // trigno::ros
