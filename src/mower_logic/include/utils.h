// TODO: This file should live in xbot_msgs, but I couldn't make it available to other packages.

#include "xbot_msgs/ActionInfo.h"

#ifndef XBOT_MSGS_UTILS_H
#define XBOT_MSGS_UTILS_H

namespace xbot_msgs {

ActionInfo create_action(std::string id, std::string name, bool enabled = false) {
  xbot_msgs::ActionInfo action;
  action.action_id = id;
  action.enabled = enabled;
  action.action_name = name;
  return action;
}

}  // namespace xbot_msgs

#endif  // XBOT_MSGS_UTILS_H
