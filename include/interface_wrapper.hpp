#ifndef TRIGNOCLIENTROS_INCLUDE_INTERFACEWRAPPER_HPP_
#define TRIGNOCLIENTROS_INCLUDE_INTERFACEWRAPPER_HPP_

#include <string>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "trignoclient_ros/GetSensorInfo.h"
#include "trignoclient_ros/Query.h"
#include <trignoclient/network.hpp>

namespace trignoclient_ros {

//------------------------------------------------------------------------------
/// @brief      Class that (partially atm) exposes Trigno SDK interface to ROS. Allows basic system configuration and streaming management.
///
/// @note       Provides a generic service to send queries to the Trigno server, as well specific trigger services to start and stop stream.
///             Additionally, a service to get information on current active sensors is advertised.
///             Under development, new wrapper services around *trignoclient* implementation may be added in the future.
///
class InterfaceWrapper {
 public:
    //--------------------------------------------------------------------------
    /// @brief      Client type.
    ///
    using Client = trigno::network::Client;

    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @param[in]  node    Handle to ROS node.
    /// @param[in]  client  Trigno client instance.
    /// @param[in]  name    Namespace to advertise service(s). Defaults to "[*node* namespace]/interface".
    ///
    InterfaceWrapper(ros::NodeHandle* node, Client* client, const std::string& name = "interface");

    //--------------------------------------------------------------------------
    /// @brief      Destroys the object.
    ///
    virtual ~InterfaceWrapper() = default;

 protected:
    //--------------------------------------------------------------------------
    /// @brief      Handle to ROS node.
    ///
    ros::NodeHandle* _node;

    //--------------------------------------------------------------------------
    /// @brief      Trigno data client (source).
    ///
    Client* _client;

    //--------------------------------------------------------------------------
    /// @brief      Service server handle for 'get_sensor_info' service.
    ///
    ros::ServiceServer _get_sensor_info_service;

    //--------------------------------------------------------------------------
    /// @brief      Service server handle for 'query_server' service.
    ///
    ros::ServiceServer _query_server_service;

    //--------------------------------------------------------------------------
    /// @brief      Service server handle for 'reset_connection' service.
    ///
    ros::ServiceServer _reset_connection_service;

    //--------------------------------------------------------------------------
    /// @brief      Service callback for 'get_sensor_info' service.
    ///
    /// @param      request   Service request instance [as trignoclient_ros::GetSensorInfo custom srv]
    /// @param      response  Service response instance [as trignoclient_ros::GetSensorInfo custom srv]
    ///
    /// @return     Service call status, true if service was successfully called, false otherwise.
    ///
    bool getSensorInfoServiceCallback(trignoclient_ros::GetSensorInfo::Request& request, trignoclient_ros::GetSensorInfo::Response& response);

    //--------------------------------------------------------------------------
    /// @brief      Service callback for 'query_server' service.
    ///
    /// @param      request   Service request instance [as trignoclient_ros::Query custom srv]
    /// @param      response  Service response instance [as trignoclient_ros::Query custom srv]
    ///
    /// @return     Service call status, true if service was successfully called, false otherwise.
    ///
    bool queryServerServiceCallback(trignoclient_ros::Query::Request& request, trignoclient_ros::Query::Response& response);

    //--------------------------------------------------------------------------
    /// @brief      Service callback for 'reset_connection' service.
    ///
    /// @param      request   Service request instance [as std_srvs::Trigger]
    /// @param      response  Service response instance [as std_srvs::Trigger]
    ///
    /// @return     Service call status, true if service was successfully called, false otherwise.
    ///
    bool resetConnectionServiceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
};

}  // namespace trignoclient_ros

#endif  // TRIGNOCLIENTROS_INCLUDE_INTERFACEWRAPPER_HPP_
