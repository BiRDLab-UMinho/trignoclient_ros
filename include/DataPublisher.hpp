#ifndef TRIGNOCLIENTROS_INCLUDE_BASICPUBLISHER_HPP_
#define TRIGNOCLIENTROS_INCLUDE_BASICPUBLISHER_HPP_

#include <string>
#include <atomic>
#include <future>
#include <ros/ros.h>
#include "trignoclient/network.hpp"  // trigno::network::BasicDataClient, trigno::Duration

namespace trigno::ros {

//------------------------------------------------------------------------------
/// @brief      Class for EMG/Aux data publishers. Publishes EMG/Aux data as soon as received (if enabled).
///
class DataPublisher {
 public:
    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @param[in]  node    Handle to ROS node.
    /// @param[in]  client  Trigno client instance.
    /// @param[in]  name    Name of data topic.
    ///
    DataPublisher(ros::NodeHandle* node, const network::BasicDataClient* client, const std::string& name, const Duration& timeout);

    //--------------------------------------------------------------------------
    /// @brief      Destroys the object.
    ///
    virtual ~DataPublisher() = default;

    //--------------------------------------------------------------------------
    /// @brief      Enable data publishing, whether or not Trigno system is streaming.
    ///
    void enable();

    //--------------------------------------------------------------------------
    /// @brief      Disable data publishing, whether or not Trigno system is streaming.
    ///
    void disable();

 protected:
    std::future< void > _publishing;
    std::atomic< bool > _enabled;

    Duration _timeout;

    //--------------------------------------------------------------------------
    /// @brief      Trigno data client (source).
    ///
    network::BasicDataClient* _trigno;

    //--------------------------------------------------------------------------
    /// @brief      Handle to ROS node.
    ///
    ros::NodeHandle* _node;

    //--------------------------------------------------------------------------
    /// @brief      ROS Publisher.
    ///
    ros::Publisher _publisher;
};


}  // namespace trigno::ros

#endif  // TRIGNOCLIENTROS_INCLUDE_BASICPUBLISHER_HPP_
