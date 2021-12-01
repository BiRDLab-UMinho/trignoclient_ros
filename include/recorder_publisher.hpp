#ifndef TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_DATAPUBLISHER_HPP_
#define TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_DATAPUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include "trignoclient/trignoclient.hpp"  // trigno::network::BasicDataClient, trigno::tools::Recorder

namespace trigno::ros {

//------------------------------------------------------------------------------
/// @brief      Publisher class that extends trigno::tools::Recorder to provide a ROS interface,
///             i.e. start/stop services and data frame publishing.
///
/// @note       Deriving from Recorder executor simplifies implementation. In alternative, could be derived
///             from BasicSequenceProcessor, but would publish from a separate thread (no real need).
///
class RecorderPublisher : public trigno::tools::Recorder {
 public:
    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @param[in]  data_client  Data client instance to source data from.
    /// @param[in]  out          Sequence to write to.
    /// @param[in]  name         Topic to publish read data.
    ///
    explicit RecorderPublisher(ros::NodeHandle* node, trigno::network::BasicDataClient* data_client, trigno::Sequence* out, const std::string& name);

    //--------------------------------------------------------------------------
    /// @brief      Destroys the object.
    ///
    virtual ~RecorderPublisher() = default;

 protected:
    //--------------------------------------------------------------------------
    /// @brief      Reads & exports a single *DataFrame* from the data client.
    ///
    void execute() override;

    //--------------------------------------------------------------------------
    /// @brief      ROS Publisher.
    ///
    ros::Publisher _publisher;
};

}  // namespace trigno::ros

#endif  // TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_DATAPUBLISHER_HPP_

