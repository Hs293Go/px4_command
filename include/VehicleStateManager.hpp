#include <boost/optional/optional.hpp>
#include <fmt/core.h>
#include <iostream>
#include <map>
#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

enum class Mode { kOffBoard, kStabilized, kAltCtl, kPosCtl };

class VehicleStateManager {
  std::string uav_tag_;
  ros::NodeHandle nh_;

  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::Subscriber state_sub_;

  mavros_msgs::State current_state_;

  void state_cb_(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
  }

  std::string mode_to_string_(const Mode &mode) {
    switch (mode) {
    case Mode::kOffBoard:
      return "OFFBOARD";
    case Mode::kStabilized:
      return "STABILIZED";
    case Mode::kAltCtl:
      return "ALTCTL";
    case Mode::kPosCtl:
      return "POSCTL";
    default:
      return "";
    }
  }

public:
  // Using boost optional to avoid using a magic number to signal single
  // vehicle
  VehicleStateManager(const boost::optional<int> &id = {}) : nh_("~") {
    std::string mavros_prefix = "/mavros";
    if (id) {
      uav_tag_ = fmt::format("UAV{}", id.get());
      mavros_prefix = fmt::format("/uav{}/mavros", id.get());
    } else {
      uav_tag_ = "UAV";
    }

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
        fmt::format("{}/cmd/arming", mavros_prefix));

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
        fmt::format("{}/set_mode", mavros_prefix));

    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        fmt::format("{}/state", mavros_prefix), 10,
        &VehicleStateManager::state_cb_, this);
  }

  bool setMode(const Mode &mode) {
    std::string mode_str = mode_to_string_(mode);

    if (current_state_.mode != mode_to_string_(mode)) {
      std::cout << fmt::format("Setting {} to {} mode\n", uav_tag_, mode_str);
      mavros_msgs::SetMode mode_cmd;

      mode_cmd.request.custom_mode = mode_str;
      return set_mode_client_.call(mode_cmd);
    } else {
      std::cout << fmt::format("{} is already in {} mode\n", uav_tag_,
                               mode_str);
      return true;
    }
  }

  bool arm() {
    if (!current_state_.armed) {
      std::cout << fmt::format("Arming {}\n", uav_tag_);
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;
      return arming_client_.call(arm_cmd);
    } else {
      std::cout << fmt::format("{} is already armed\n", uav_tag_);
      return true;
    }
  }

  bool disarm() {
    if (!current_state_.armed) {
      std::cout << fmt::format("Disarming {}\n", uav_tag_);
      mavros_msgs::CommandBool disarm_cmd;
      disarm_cmd.request.value = false;
      return arming_client_.call(disarm_cmd);
    } else {
      std::cout << fmt::format("{} is already disarmed\n", uav_tag_);
      return true;
    }
  }
};
