#include <ros/ros.h>
#include "bindings.hpp"      // trigno::ros::sensors(), trigno::ros::parametrize()
#include "data_wrapper.hpp"

namespace trigno::ros {

DataWrapper::DataWrapper(ros::NodeHandle* node, trigno::network::BasicDataClient* client, const std::string& name) :
    _recorder_publisher(node, client, &_data, name + "/data") {
        // advertise services
        _start_service = node->advertiseService(node->getNamespace() + "/" + name + "/record", DataWrapper::startServiceCallback, this);
        _stop_service  = node->advertiseService(node->getNamespace() + "/" + name + "/stop", DataWrapper::stopServiceCallback, this);
        // @note       exporter is only initialized if requested when starting recording.
        //             this avoids overhead whenever exporting is not required
}



DataWrapper::~DataWrapper() {
    // send kill signal & wait until executors are finished!
    // this may lead to problems if DataWrapper is prematurely destroyed, but it won't block
    _recorder_publisher.kill();
    _exporter.kill();
}



bool DataWrapper::startServiceCallback(trignoclient_ros::Record::Request& request, trignoclient_ros::Record::Response& response) {
    // parse request arguments
    auto duration = request.duration > 0 ? request.duration : trigno::Duration::max();
    auto sensors = ros::sensors(request.sensors);

    // launch recorder asynchronously
    _recorder_publisher.launch(request.duration, sensors);
    response.message = "Started recording data for " + std::to_string(duration.count()) + " ms successfully";

    // lauch export if file name is provided (ignore otherwise)
    if (!request.export_path.empty()) {
        // parametrize from ROS parameter server
        size_t batch;
        parametrize(&_node, &_exporter, &batch);
        // _exporter.wait();  // no need to wait, exporter will launch on a separate thread
        _exporter.launch(_data.begin(batch), sensors);
        response.message += " and data export to " + request.export_path + " with batch size " + std::to_string(batch) + " succesfully.";
    }

    // assign output message
    response.success = true;
    return true;
}



bool DataWrapper::stopServiceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    // kill underlying executor
    _publisher.kill();
    // _exporter.kill();  // no need to kill exporter: it will self-terminate when it runs out of data

    // check if publisher is still running
    // @note exporter may still be writing frames
    if (_publisher.active()) {
        response.message = "Failed to stop data recording/exporting!";
        response.success = false;
    } else if (_exporter.active()) {
        response.message = "Successfully stopped data recording, but still exporting previous data.";
        response.success = true;
    } else {
        response.message = "Sucessfully stopped data recording/exporting";
        response.success = true;
    }
    return true;
}

}  // namespace trigno::ros
