#include "VehicleStateManager.hpp"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "set_mode");

  ros::NodeHandle nh("~");
  VehicleStateManager state_man;
  int uav_id;
  if (nh.getParam("uav_id", uav_id)) {
    state_man = VehicleStateManager(uav_id);
  }

  ros::start();
  ros::Rate rate(10.0);

  enum AugmentedMode {
    kOffBoard,
    kStabilized,
    kAltCtl,
    kPosCtl,
    kArm,
    kDisarm,
    kPrompt
  } internal_state;

  int input = 0;

  internal_state = kPrompt;

  int attempts = 0;

  while (ros::ok()) {
    switch (internal_state) {
    case kPrompt:
      attempts = 0;
      std::cout << "Input mode: 0: OFFBOARD, 1: STABILIZED, 2: ALTCTL, "
                   "3: POSCTL, 4: ARM, 5: DISARM, any other number to exit: ";

      std::cin >> input;

      if (!std::cin.good() || input > 5 || input < 0) {
        std::cin.ignore();
        std::cin.clear();
        nh.deleteParam("uav_id");
        return 0;
      }

      internal_state = static_cast<AugmentedMode>(input);
      break;

    case kOffBoard:
      if (state_man.setMode(Mode::kOffBoard)) {
        internal_state = kPrompt;
        break;
      }
      attempts++;
      break;

    case kStabilized:
      if (state_man.setMode(Mode::kStabilized)) {
        internal_state = kPrompt;
        break;
      }
      attempts++;
      break;

    case kAltCtl:
      if (state_man.setMode(Mode::kAltCtl)) {
        internal_state = kPrompt;
        break;
      }
      attempts++;
      break;

    case kPosCtl:
      if (state_man.setMode(Mode::kPosCtl)) {
        internal_state = kPrompt;
        break;
      }
      attempts++;
      break;

    case kArm:
      if (state_man.arm()) {
        internal_state = kPrompt;
        break;
      }
      attempts++;
      break;

    case kDisarm:
      if (state_man.disarm()) {
        internal_state = kPrompt;
        break;
      }
      attempts++;
      break;
    }

    if (attempts > 5) {
      std::cout << "Giving up after 5 attempts.\n";
      internal_state = kPrompt;
    }
  }
  nh.deleteParam("uav_id");
  return 0;
}