//------------------------------------------------------------------------------
/// @file       bindings.hpp
/// @author     João André
///
/// @brief      Header file providing definituion of inlined converson functions between *trignoclient* structures and their
///             analogous/translated ROS messages.
///
//------------------------------------------------------------------------------

#ifndef TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_BINDINGS_HPP_
#define TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_BINDINGS_HPP_

#include <string>
#include <vector>
#include <ros/ros.h>
#include "trignoclient/network.hpp"  // trigno::Frame, trigno::network::SensorInfo
#include "trignoclient_ros/SensorInfo.h"
#include "trignoclient_ros/Frame.h"
#include "trignoclient_ros/FrameStamped.h"

#ifndef DEFAULT_CMD_PORT
#define DEFAULT_CMD_PORT 50040
#endif

#ifndef DEFAULT_CMD_PORT
#define DEFAULT_CMD_PORT 50040
#endif

#ifndef DEFAULT_CMD_PORT
#define DEFAULT_CMD_PORT 50040
#endif

#ifndef DEFAULT_CONNECT_TIMEOUT
#define DEFAULT_CONNECT_TIMEOUT 5000
#endif

#ifndef DEFAULT_EXPORT_BATCH
#define DEFAULT_EXPORT_BATCH 1000
#endif

#ifndef DEFAULT_IO_TIMEOUT
#define DEFAULT_IO_TIMEOUT 500
#endif

#ifndef DEFAULT_IDLE_TIMEOUT
#define DEFAULT_IDLE_TIMEOUT 10000
#endif

namespace trigno::ros {

//------------------------------------------------------------------------------
/// @brief      Configures & resets a Client instance.
///
/// @param      node      Parent node used to fetch parameters (parameter names are parsed according to node name if private handle).
/// @param      client    Client instance to parametrize.
///
inline void initialize(ros::NodeHandle* node, trigno::network::Client* client) {
    auto address  = node->param< std::string >("network/server_address", "127.0.0.1");
    auto cmd_port = node->param< int >("network/command_port", DEFAULT_CMD_PORT);
    auto emg_port = node->param< int >("network/emg_data_port", DEFAULT_EMG_PORT);
    auto aux_port = node->param< int >("network/aux_data_port", DEFAULT_AUX_PORT);
    auto timeout  = node->param< int >("network/connect_timeout", DEFAULT_CONNECT_TIMEOUT);
    // reset network connection
    // client->shutdown();
    client->initialize(address, cmd_port, emg_port, aux_port, trigno::Duration(timeout));
}



//------------------------------------------------------------------------------
/// @brief      Parametrizes an exporter though the ROS parameter server.
///
/// @param      node      Parent node used to fetch parameters (parameter names are parsed according to node name if private handle).
/// @param      exporter  Exporter instance to configure
///
inline void parametrize(ros::NodeHandle* node, trigno::tools::Exporter* exporter, size_t* batch_size) {
    // set target file (defaults to home folder)
    auto target_root = node->param< std::string >("export/target_root", "~");
    _exporter.target(target_root + "/" + target_file);
    // eval consume flag (defaults to true!)
    auto consume = node->param< bool >("export/consume_input", true);
    _exporter.source(consume ? &_data : nullptr);
    // fetch export batch parameter (range in which to write data to disk, )
    node->param< int >("export_batch_size", *batch_size, DEFAULT_EXPORT_BATCH);
}



//------------------------------------------------------------------------------
/// @brief      Creates a sensor list (a.k.a. sensor::List) from ROS sensor descriptors
///
/// @param[in]  sensors  The sensors
///
/// @return     List of sensor (IDs) matching valid inputs.
///
inline trigno::sensor::List sensors(const std::vector< trignoclient_ros::Sensor >& sensors) {
    trigno::sensor::List out;
    for (const auto& sensor : sensors) {
        if (sensor.id < trigno::sensor::ID::MAX) {
            continue;
        }
        out.emplace_back(static_cast< trigno::sensor::ID >(sensor.id));
    }
    return out;
}



//------------------------------------------------------------------------------
/// @brief      Converts a SensorInfo instance into its associated ROS message.
///
/// @param[in]  info  Input SensorInfo instance, as accessed/returned by a trigno::network::Client.
///
/// @return     Newly instantiated trignoclient_ros::SensorInfo message structure.
///
inline trignoclient_ros::SensorInfo msg(const trigno::network::SensorInfo& info) {
    trignoclient_ros::SensorInfo out;

    out.id = info.id();
    out.type = info.type();
    out.mode = info.mode();
    out.serial = info.serial();
    out.firmware = info.firmware();
    out.n_emg_channels = info.nEMGChannels();
    out.n_aux_channels = info.nAUXChannels();
    out.sample_rate = info.sampleRate()[0];
    out.units = info.units()[0];
    out.gain = info.gain()[0];
    out.range = info.lowRange();
    out.bandwidth = info.narrowBandwidth();

    return out;
}



//------------------------------------------------------------------------------
/// @brief      Converts a BaseInfo instance into its associated ROS message.
///
/// @param[in]  info  Input BaseInfo instance, as accessed/returned by a trigno::network::Client.
///
/// @return     Newly instantiated trignoclient_ros::BaseInfo message structure.
///
inline trignoclient_ros::BaseInfo msg(const trigno::network::BaseInfo& info) {
    trignoclient_ros::BaseInfo out;

    out.serial = info.serial();
    out.firmware = info.firmware();

    return out;
}


// ... SystemControl, ConnectionInfo, etc


//------------------------------------------------------------------------------
/// @brief      Converts a trigno::Sample instance into its associated ROS message.
///
/// @param[in]  info  Input trigno::Sample instance with sensor channel readings.
///
/// @return     Newly instantiated trignoclient_ros::Sample message structure.
///
inline trignoclient_ros::Sample msg(const trigno::Sample& sample) {
    trignoclient_ros::Sample out;

    // sample metadata (id)
    out.sensor.id = sample.id();  // note: frame has no label!
    // channel data
    out.channels = sample.data();

    return out;
}



//------------------------------------------------------------------------------
/// @brief      Converts a trigno::Frame instance into its associated ROS message.
///
/// @param[in]  info  Input trigno::Frame instance with multisensor data (labelled).
///
/// @return     Newly instantiated trignoclient_ros::Frame message structure.
///
inline trignoclient_ros::Frame msg(const trigno::Frame& frame) {
    trignoclient_ros::Frame out;

    for (size_t idx = 0; idx < frame.size(); idx++) {
        // create instance @ end of frame
        out.emplace_back(msg(frame[idx]));
        // assign label
        out.back().sensor.label = frame.key(idx)
    }

    return out;
}



//------------------------------------------------------------------------------
/// @brief      Converts a trigno::Frame::Stamped instance into its associated ROS message.
///
/// @param[in]  info  Input trigno::Frame::Stamped instance with time-stamped sensor channel readings.
///
/// @return     Newly instantiated trignoclient_ros::FrameStamped message structure.
///
inline trignoclient_ros::FrameStamped msg(const trigno::Frame::Stamped& frame) {
    trignoclient_ros::FrameStamped out;

    out.header.stamp = frame.key;
    out.data = msg(frame.get());

    return out;
}

}  // namespace trigno::ros

#endif  // TRIGNOCLIENTROS_INCLUDE_TRIGNOCLIENTROS_BINDINGS_HPP_
