#include <ros/ros.h>
#include "bindings.hpp"      // ros::trigno::sensors(), ros::trigno::parametrize()
#include "data_wrapper.hpp"

#ifndef DEFAULT_EXPORT_BATCH
#define DEFAULT_EXPORT_BATCH 1000
#endif

namespace trignoclient_ros {

DataWrapper::DataWrapper(ros::NodeHandle* node, DataWrapper::DataClient* client, const std::string& name) :
    _publisher(node, client, &_data, name + "/data") {
        // advertise services
        _start_service = node->advertiseService(node->getNamespace() + "/" + name + "/record", &DataWrapper::startServiceCallback, this);
        _stop_service  = node->advertiseService(node->getNamespace() + "/" + name + "/stop", &DataWrapper::stopServiceCallback, this);
        // @note       exporter is only initialized if requested when starting recording.
        //             this avoids overhead whenever exporting is not required
}



DataWrapper::~DataWrapper() {
    // send kill signal & wait until executors are finished!
    // this may lead to problems if DataWrapper is prematurely destroyed, but it won't block
    _publisher.kill();
    _exporter.kill();
}



bool DataWrapper::startServiceCallback(trignoclient_ros::Record::Request& request, trignoclient_ros::Record::Response& response) {
    // parse request arguments
    auto duration = trignoclient_ros::duration(request.time);
    if (duration <= trigno::Duration(0)) {
        duration = trigno::Duration::max();
    }
    auto sensors = trignoclient_ros::sensors(request.sensors);

    // launch recorder asynchronously
    _publisher.launch(duration, sensors);
    response.message = "Started recording data for " + std::to_string(duration.count()) + " ms successfully";

    // lauch export if file name is provided (ignore otherwise)
    if (!request.file.empty()) {
        // parametrize from ROS parameter server
        // set target file (defaults to home folder)
        auto target_root = _node->param< std::string >("export/target_root", "~");
        _exporter.target(target_root + "/" + request.file);
        // eval consume flag (defaults to true!)
        auto consume = _node->param< bool >("export/consume_input", true);
        _exporter.source(consume ? &_data : nullptr);
        // fetch export batch parameter (range in which to write data to disk)
        size_t batch = _node->param< int >("export_batch_size", DEFAULT_EXPORT_BATCH);
        // _exporter.wait();  // no need to wait, exporter will launch on a separate thread
        _exporter.launch(_data.begin(batch), sensors);
        response.message += " and data export to " + request.file + " with batch size " + std::to_string(batch) + " succesfully.";
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

}  // namespace trignoclient_ros
