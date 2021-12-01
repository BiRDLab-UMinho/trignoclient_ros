#ifndef TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_DATAWRAPPER_HPP_
#define TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_DATAWRAPPER_HPP_

#include <string>
#include <ros/ros.h>
#include <trignoclient/trignoclient.hpp>

namespace trigno::ros {

//------------------------------------------------------------------------------
/// @brief      Class that exposes *trignoclient* recording & exporting functionalities to ROS enviroment i.e.
///             start/stop services and frame publishing.
///
class DataWrapper {
 public:
    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @param      node    The node
    /// @param      client  The client
    /// @param[in]  name    The name
    ///
    DataWrapper(ros::NodeHandle* node, trigno::network::BasicDataClient* client, const std::string& name);

    //--------------------------------------------------------------------------
    /// @brief      Destroys the object.
    ///
    virtual ~DataWrapper() = default;

 protected:
    //--------------------------------------------------------------------------
    /// @brief      Data/frame sequence.
    ///
    trigno::Sequence _data;

    //--------------------------------------------------------------------------
    /// @brief      Recording (timed) & publishing executor.
    ///
    /// @note       RecorderPublisher extends Recorder functionality, no need to have a dedicated Recorder.
    ///
    trigno::ros::RecorderPublisher _publisher;

    //--------------------------------------------------------------------------
    /// @brief      Data export executor.
    ///
    trigno::tools::Exporter _exporter;

    //--------------------------------------------------------------------------
    /// @brief      Handle to parent ROS node.
    ///
    ros::NodeHandle* _node;

    //--------------------------------------------------------------------------
    /// @brief      ROS 'start' service.
    ///
    ros::ServiceServer _start_service;

    //--------------------------------------------------------------------------
    /// @brief      ROS 'stop' service.
    ///
    ros::ServiceServer _stop_service;
};

}  // namespace trigno::ros

#endif  // TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_DATAWAPPER_HPP_
