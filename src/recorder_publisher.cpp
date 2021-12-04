#include <string>
#include <ros/ros.h>
#include <trignoclient/trignoclient.hpp>    // trigno::network::BasicDataClient
#include <trignoclient_ros/FrameStamped.h>  // trignoclient_ros::FrameStamped
#include "bindings.hpp"                     // trignoclient_ros::msg
#include "recorder_publisher.hpp"

namespace trignoclient_ros {

RecorderPublisher::RecorderPublisher(ros::NodeHandle* node, trigno::network::BasicDataClient* data_client, trigno::Sequence* out, const std::string& name) :
    Recorder(data_client, out) {
        /* ... */
        _publisher = node->advertise< trignoclient_ros::FrameStamped >(node->getNamespace() + "/" + name, 10 /* queue size */);
}



void RecorderPublisher::execute() {
    // read frame & append to sequence
    // delegate to base class method
    Recorder::execute();
    // publish last frame
    _publisher.publish(trignoclient_ros::msg(_out->back()));
}

}  // namespace trignoclient_ros
