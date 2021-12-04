#ifndef TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_TRIGNOTrignoDataWrapper_HPP_
#define TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_TRIGNOTrignoDataWrapper_HPP_

#include <string>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <trignoclient/trignoclient.hpp>  // trigno::network::BasicDataClient
#include "trignoclient_ros/Record.h"      // trignoclient_ros::Record
#include "recorder_publisher.hpp"         // ros::RecorderPublisher

namespace trignoclient_ros {

//------------------------------------------------------------------------------
/// @brief      Class that exposes *trignoclient* recording & exporting functionalities to ROS enviroment i.e.
///             start/stop services and frame publishing.
///
class DataWrapper {
 public:
    //--------------------------------------------------------------------------
    /// @brief      Data client type.
    ///
    using DataClient = trigno::network::BasicDataClient;

    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @param      node    The node
    /// @param      client  The client
    /// @param[in]  name    The name
    ///
    DataWrapper(ros::NodeHandle* node, DataClient* client, const std::string& name);

    //--------------------------------------------------------------------------
    /// @brief      Destroys the object.
    ///
    virtual ~DataWrapper();

 protected:
    //--------------------------------------------------------------------------
    /// @brief      Data/frame sequence.
    ///
    trigno::Sequence _data;

    //--------------------------------------------------------------------------
    /// @brief      Data export executor.
    ///
    trigno::tools::Exporter _exporter;

    //--------------------------------------------------------------------------
    /// @brief      Handle to parent ROS node.
    ///
    ros::NodeHandle* _node;

    //--------------------------------------------------------------------------
    /// @brief      Recording (timed) & publishing executor.
    ///
    /// @note       RecorderPublisher extends base Recorder functionality, no need to have an additional dedicated Recorder.
    ///
    RecorderPublisher _publisher;

    //--------------------------------------------------------------------------
    /// @brief      ROS 'start' service.
    ///
    ros::ServiceServer _start_service;

    //--------------------------------------------------------------------------
    /// @brief      ROS 'stop' service.
    ///
    ros::ServiceServer _stop_service;

    //--------------------------------------------------------------------------
    /// @brief      ROS 'start' service callback function.
    ///
    bool startServiceCallback(trignoclient_ros::Record::Request& request, trignoclient_ros::Record::Response& response);

    //--------------------------------------------------------------------------
    /// @brief      ROS 'stop' service callback function.
    ///
    bool stopServiceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
};

}  // namespace ros

#endif  // TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_DATAWAPPER_HPP_
