#include <string>
#include <ros/ros.h>
#include "trignoclient/network.hpp"  // trigno::network::BasicDataClient
#include "utils.hpp"  // toROSMessage
#include "DataPublisher.hpp"

namespace trigno::ros {

DataPublisher::DataPublisher(ros::NodeHandle* node, const network::BasicDataClient* client, const std::string& topic, const Duration& timeout):
    _node(node),
    _client(client),
    _timeout(timeout),
    _enabled(false) {
        _publisher = _node->advertise(_node->getNamespace() + "/" + topic);
        // parse ROS param w/ IO timeout, use trignoclient default
}


void DataPublisher::enable() {
    _done.wait();
    // perfom sequential network read & publish asynchronousloy
    _enabled = true;
    _done = std::async(std::launch::async, [this] {
        while (_enabled) {
            _publisher.publish(toROSMessage(_client.read(_timeout)));
        }
    });
}


void DataPublisher::disable() {
    _enabled = false;
}


void DataPublisher::spin() {


}

}  // trigno::ros
