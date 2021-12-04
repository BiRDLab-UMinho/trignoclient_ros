#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "trignoclient_ros/GetSensorInfo.h"
#include "trignoclient_ros/Query.h"
#include "bindings.hpp"  // ros::trigno::msg(), ros::trigno::initialize()
#include "interface_wrapper.hpp"

namespace trignoclient_ros {

InterfaceWrapper::InterfaceWrapper(ros::NodeHandle* node, InterfaceWrapper::Client* client, const std::string& name):
    _node(node),
    _client(client) {
        // advertise services
        _get_sensor_info_service  = _node->advertiseService(_node->getNamespace() + "/" + name + "/get_sensor_info", &InterfaceWrapper::getSensorInfoServiceCallback, this);
        _query_server_service     = _node->advertiseService(_node->getNamespace() + "/" + name + "/query_server", &InterfaceWrapper::queryServerServiceCallback, this);
        _reset_connection_service = _node->advertiseService(_node->getNamespace() + "/" + name + "/reset_connection", &InterfaceWrapper::resetConnectionServiceCallback, this);
        // ...
        // parametrize & connect to remote Trigno server
        trignoclient_ros::initialize(_node, _client);
}


bool InterfaceWrapper::getSensorInfoServiceCallback(trignoclient_ros::GetSensorInfo::Request& request, trignoclient_ros::GetSensorInfo::Response& response) {
    if (!request.sensors.size()) {
        // return all active sensors
        for (const auto& id : trigno::sensor::all) {
            try {
                // try id lookup (if valid, O(1) complexity)
                response.info.emplace_back(trignoclient_ros::msg(_client->sensor[id]));
            } catch (std::exception&) { /* ... */ }
        }
    } else {
        // return all active sensors
        for (const auto& sensor : request.sensors) {
            try {
                // try id lookup (if valid, O(1) complexity)
                response.info.emplace_back(trignoclient_ros::msg(_client->sensor[sensor.id]));
            } catch (std::exception&) {
                // if id lookup fails, try label (slower i.e. linear search O(n) complexity)
                try {
                    response.info.emplace_back(trignoclient_ros::msg(_client->sensor[sensor.label]));
                } catch (std::exception&) { /* ... */ }
            }
        }
    }
    return true;
}



bool InterfaceWrapper::queryServerServiceCallback(trignoclient_ros::Query::Request& request, trignoclient_ros::Query::Response& response) {
    response.answer = _client->server.query(request.query, trignoclient_ros::duration(request.timeout));
    return true;
}



bool InterfaceWrapper::resetConnectionServiceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    // close connection to server
    // this will stop any ongoing data streaming/recording
    if (_client->connected()) {
        _client->shutdown();
    }
    // parametrize & connect to remote Trigno server
    trignoclient_ros::initialize(_node, _client);
    // assign
    response.success = true;
    response.message = "Connection to Trigno server sucessfully reset.";
    return true;
}

}  // namespace trignoclient_ros
