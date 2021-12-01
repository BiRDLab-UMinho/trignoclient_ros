#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "trignoclient_ros/GetSensorInfo.h"
#include "trignoclient_ros/Query.h"
#include "bindings.hpp"  // trigno::ros::msg(), trigno::ros::initialize()

namespace trigno::ros {

InterfaceWrapper::InterfaceWrapper(ros::NodeHandle* node, const InterfaceWrapper::Client* client, const std::string& namespace):
    _node(node),
    _client(client) {
        // advertise services
        _get_sensor_info_service  = _node->advertiseService(_node->getNamespace() + "/" + namespace + "/get_sensor_info", &TrignoInterfaceWrapper::getSensorInfoServiceCallback, this);
        _query_server_service     = _node->advertiseService(_node->getNamespace() + "/" + namespace + "/query_server", &TrignoInterfaceWrapper::queryServerServiceCallback, this);
        _reset_connection_service = _node->advertiseService(_node->getNamespace() + "/" + namespace + "/reset_connection", &TrignoInterfaceWrapper::resetConnectionServiceCallback, this);
        // ...
}


bool InterfaceWrapper::getSensorInfoServiceCallback(trignoclient_ros::GetSensorInfo::Request& request, trignoclient_ros::GetSensorInfo::Response& response) {
    if (!request.id) {
        // use label
        try {
            response.info.emplace_back(trigno::ros::msg(_client->sensor[request.label]));
        }
    }
    if (request.id > request.MAX_ID) {
        // return all active sensors
    }
    response.info.emplace_back(trigno::ros::msg(_client->sensor[request.id]));
    return true;
}



bool InterfaceWrapper::queryServerServiceCallback(trignoclient_ros::Query::Request& request, trignoclient_ros::Query::Response& response) {
    _response.answer = _client->query(request.query, request.timeout);
    return true;
}



bool InterfaceWrapper::resetConnectionServiceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    // close connection to server
    // this will stop any ongoing data streaming/recording
    if (client->connected()) {
        client->shutdown();
    }
    // parametrize & connect to remote Trigno server
    tringo::ros::initialize(_node, _client);
    // assign
    response.success = true;
    response.message = "Connection to Trigno server sucessfully reset.";
    return true;
}

}  // namespace trigno::ros
