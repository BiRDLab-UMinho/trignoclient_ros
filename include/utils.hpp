#include "trignoclient/network.hpp"  // trigno::Frame, trigno::network::SensorInfo
#include "trignoclient_ros/SensorInfo.h"
#include "trignoclient_ros/Frame.h"
#include "trignoclient_ros/FrameStamped.h"

namespace trigno::ros {
    
inline trignoclient_ros::SensorInfo toROSMessage(const trigno::network::SensorInfo& info) {

}

inline trignoclient_ros::Frame toROSMessage(const trigno::Frame& info) {

}

inline trignoclient_ros::FrameStamped toROSMessage(const trigno::Frame::Stamped& info) {

}

}
