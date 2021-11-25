#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "trignoclient_ros/GetSensorInfo.h"
#include "trignoclient_ros/Query.h"

namespace trigno::ros {

Interface::Interface(NodeHandle* node, const Client* client, const std::string& namespace):
    _node(node),
    _client(client) {
        //
        _get_sensor_info_service = _node->advertiseService(_node->getNamespace() + namespace + "/get_sensor_info", &TrignoInterface::getSensorInfoServiceCallback, this);
        _query_service = _node->advertiseService(_node->getNamespace() + namespace + "/query", &TrignoInterface::queryServiceCallback, this);
        _start_service = _node->advertiseService(_node->getNamespace() + namespace + "/start", &TrignoInterface::startServiceCallback, this);
        _stop_service = _node->advertiseService(_node->getNamespace() + namespace + "/stop", &TrignoInterface::stopServiceCallback, this);
}


bool Interface::getSensorInfoServiceCallback(trignoclient_ros::GetSensorInfo::Request& request, trignoclient_ros::GetSensorInfo::Response& response) {
    if (!request.id) {
        // use label
        try {
            response.info.emplace_back(toInfoMsg(_client->sensor[request.label]));
        }
    }
    if (request.id > request.MAX_ID) {
        // return all active sensors
    }
    response.info.emplace_back(toInfoMsg(_client->sensor[request.id]));
    return true;
}


bool Interface::queryServiceCallback(trignoclient_ros::Query::Request& request, trignoclient_ros::Query::Response& response) {
    _response.answer = _client->query(request.query, request.timeout);
    return true;
}


bool Interface::startServiceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    _client->start();
    response.success = true;
    response.message = "Data streaming sucessfully started.";
    return true;
}


bool Interface::stopServiceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    _client->stop();
    response.success = true;
    response.message = "Data streaming sucessfully stopped.";
    return true;
}

}  // namespace trigno::ros
