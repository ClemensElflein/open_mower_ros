#include "PerimeterDocking.h"

#include <mower_msgs/Power.h>

#include "IdleBehavior.h"
#include "mower_msgs/Perimeter.h"
#include "mower_msgs/PerimeterControlSrv.h"

#define MIN_SIGNAL 5
#define SEARCH_SPEED 0.1
#define ANGULAR_SPEED 0.5

/* Distance from center to outer coil */
#define COIL_Y_OFFSET 0.11
/* Distance from rear axis to coils */
#define COIL_X_OFFSET 0.40

#define FOLLOW_STATE_FOLLOW 0
#define FOLLOW_STATE_TURN_IN 1
#define FOLLOW_STATE_TURN_OUT 2

extern ros::NodeHandle* n;
extern ros::Publisher cmd_vel_pub;
extern mower_msgs::Status getStatus();
extern mower_msgs::Power getPower();
extern void setGPS(bool enabled);

static ros::Subscriber perimeterSubscriber;
static ros::ServiceClient perimeterClient;

static mower_msgs::Perimeter lastPerimeter;
static int perimeterUpdated = 0;
static int direction;  // 1 if we approach counter clockwise, -1 for clockwise
/* Sensitivty of the coils, sign makes calibration*rawSignal positive inside the perimeter */
static double calibrationLeft, calibrationRight, maxCenter;
static int signCenter;

PerimeterSearchBehavior PerimeterSearchBehavior::INSTANCE;
PerimeterDockingBehavior PerimeterDockingBehavior::INSTANCE;
PerimeterUndockingBehavior PerimeterUndockingBehavior::INSTANCE;
PerimeterMoveToGpsBehavior PerimeterMoveToGpsBehavior::INSTANCE;

static void perimeterReceived(const mower_msgs::Perimeter::ConstPtr& msg) {
  lastPerimeter = *msg;
  perimeterUpdated = 1;
}

static Behavior* shutdownConnections() {
  mower_msgs::PerimeterControlSrv p;
  p.request.listenOn = 0;
  if (perimeterClient.call(p)) {
    ROS_INFO("Perimeter deactivated.");
  } else {
    ROS_ERROR("Failed to deactivate perimeter.");
  }
  perimeterSubscriber.shutdown();
  perimeterClient.shutdown();
  return &IdleBehavior::INSTANCE;
}

static int isPerimeterUpdated() {
  if (perimeterUpdated) {
    perimeterUpdated = 0;
    return 1;
  }
  return 0;
}

static float innerSignal() {
  return direction > 0 ? lastPerimeter.left * calibrationLeft : lastPerimeter.right * calibrationRight;
}

static float outerSignal() {
  return direction > 0 ? lastPerimeter.right * calibrationRight : lastPerimeter.left * calibrationLeft;
}

int PerimeterSearchBehavior::configured(const mower_logic::MowerLogicConfig& config) {
  return config.perimeter_signal != 0;
}

std::string PerimeterSearchBehavior::state_name() {
  return "DOCKING";
}

Behavior* PerimeterSearchBehavior::execute() {
  if (!setupConnections()) return shutdownConnections();
  ros::Rate rate(10);
  int tries = 100;
  int toFind = 5;
  /* Wait ten seconds for initial perimeter signal */
  while (tries-- && toFind) {
    if (isPerimeterUpdated()) {
      toFind--;
    }
    rate.sleep();
  }

  if (toFind) {
    ROS_ERROR("Failed to activate perimeter");
    return shutdownConnections();
  }

  /* We expect to be currently outside the perimeter wire => negative signal */
  calibrationLeft = calibrationRight = signCenter =
      lastPerimeter.left < 0 ? 1 : -1;  // Determine polarity of the cable.

  if (innerSignal() > -MIN_SIGNAL || outerSignal() > -MIN_SIGNAL) {
    ROS_ERROR("Signal too weak.");
    return shutdownConnections();
  }
  calibrationLeft /= fabs(lastPerimeter.left);
  maxCenter = fabs(lastPerimeter.center);
  calibrationRight /= fabs(lastPerimeter.right);

  geometry_msgs::Twist vel;
  /* Move straight until one of the signals changes to positive (inside) */
  vel.angular.z = 0;
  vel.linear.x = SEARCH_SPEED;
  tries = 10;  // One second to next perimeter msg.
  while (tries-- > 0) {
    cmd_vel_pub.publish(vel);
    rate.sleep();
    if (isPerimeterUpdated()) {
      tries = 10;
      if (innerSignal() > 0 || outerSignal() > 0) break;
    }
  }

  if (!tries) {
    ROS_ERROR("Signal timeout");
    return shutdownConnections();
  }

  /* Move further until the wheels are over the perimeter (approx. 30 cm) */
  for (tries = 20; tries-- > 0;) {
    cmd_vel_pub.publish(vel);
    rate.sleep();
  }

  return &PerimeterDockingBehavior::INSTANCE;
}

int PerimeterUndockingBehavior::configured(const mower_logic::MowerLogicConfig& config) {
  return config.perimeter_signal != 0 && config.undock_distance > 1.0;
}

std::string PerimeterUndockingBehavior::state_name() {
  return "UNDOCKING";
}

Behavior* PerimeterUndockingBehavior::execute() {
  if (!setupConnections()) return shutdownConnections();
  ros::Rate rate(10);
  int tries = 100;
  int toFind = 5;
  /* Wait ten seconds for initial perimeter signal */
  while (tries-- && toFind) {
    if (isPerimeterUpdated()) {
      toFind--;
    }
    rate.sleep();
  }

  if (toFind) {
    ROS_ERROR("Failed to activate perimeter");
    return shutdownConnections();
  }

  geometry_msgs::Twist vel;
  /* Move straight back 0.5 m */
  vel.angular.z = 0;
  vel.linear.x = -SEARCH_SPEED;
  double travelled = 0;
  while (travelled < 0.5) {
    cmd_vel_pub.publish(vel);
    rate.sleep();
    travelled += rate.expectedCycleTime().toSec() * SEARCH_SPEED;
  }

  // Determine polarity of the cable.
  calibrationLeft = calibrationRight = signCenter = 1;
  if (innerSignal() < 0) calibrationLeft = calibrationRight = signCenter = -1;

  float maxLeft = lastPerimeter.left * calibrationLeft;
  maxCenter = fabs(lastPerimeter.center);
  float maxRight = lastPerimeter.right * calibrationRight;
  ;

  /* Now make a 180 degree turn inwards */
  vel.angular.z = direction * ANGULAR_SPEED;
  vel.linear.x = 0;
  tries = 240;
  while (innerSignal() > 0 && --tries) {
    cmd_vel_pub.publish(vel);
    rate.sleep();
    if (isPerimeterUpdated()) {
      float x = lastPerimeter.left * calibrationLeft;
      if (x > maxLeft) maxLeft = x;
      x = fabs(lastPerimeter.center);
      if (x > maxCenter) maxCenter = x;
      x = lastPerimeter.right * calibrationRight;
      if (x > maxRight) maxRight = x;
    }
  }
  if (!tries) {
    ROS_ERROR("Could not turn inwards");
    return shutdownConnections();
  }

  if (maxLeft < MIN_SIGNAL || maxRight < MIN_SIGNAL) {
    ROS_ERROR("Signal too weak.");
    return shutdownConnections();
  }

  /* After turning we toggle turning direction */
  direction = -direction;
  calibrationLeft /= maxLeft;
  calibrationRight /= maxRight;
  return &PerimeterMoveToGpsBehavior::INSTANCE;
}

std::string PerimeterDockingBehavior::state_name() {
  return "DOCKING";
}

Behavior* PerimeterDockingBehavior::arrived() {
  if (travelled > config.docking_distance) {
    ROS_WARN("Travelled %.f meters before reaching the station", travelled);
    return &IdleBehavior::INSTANCE;
  }
  if (getPower().v_charge > 5.0) {
    chargeSeen++;
    if (chargeSeen >= 2) {
      chargeSeen = 0;
      return &IdleBehavior::DOCKED_INSTANCE;
    }
  } else {
    chargeSeen = 0;
  }
  return NULL;
}

Behavior* PerimeterFollowBehavior::execute() {
  ros::Rate rate(10);
  geometry_msgs::Twist vel;
  travelled = 0;
  int state = FOLLOW_STATE_FOLLOW;
  double travelTimeSinceUpdate = 0;
  double lastAlpha0 = 0;
  int tries = 0;
  perimeterUpdated = 1;  // Use first measurement
  Behavior* toReturn;
  double drift = 0;            // The angular velocity of the mower, if we want to go strait on.
  double averageInterval = 5;  // Average five seconds.
  while (ros::ok() && !(toReturn = arrived())) {
    if (!isPerimeterUpdated()) {
      cmd_vel_pub.publish(vel);
      rate.sleep();
      if (tries && --tries == 0) {
        ROS_ERROR("Timeout of action %d", state);
        toReturn = &IdleBehavior::INSTANCE;
        break;
      }
      double d = rate.expectedCycleTime().toSec();
      travelled += d * vel.linear.x;
      travelTimeSinceUpdate += d;
      continue;
    }
    /* Turn into direction of perimeter */
    if (innerSignal() < 0) {
      /* Inner coil is outside */
      vel.linear.x = 0;
      vel.angular.z = ANGULAR_SPEED * direction;
      if (state != FOLLOW_STATE_TURN_IN) {
        tries = 240;
        state = FOLLOW_STATE_TURN_IN;
      }
    } else if (outerSignal() > 0) {
      /* Outer coil is inside */
      vel.linear.x = 0;
      vel.angular.z = -ANGULAR_SPEED * direction;
      if (state != FOLLOW_STATE_TURN_OUT) {
        tries = 240;
        state = FOLLOW_STATE_TURN_OUT;
      }
    } else {
      vel.linear.x = SEARCH_SPEED;
      if (fabs(lastPerimeter.center) > maxCenter) {
        maxCenter = fabs(lastPerimeter.center);
      }
      double c = lastPerimeter.center * signCenter;
      /* Signal of the center coil is proportional to distance from the wire */
      /* y0: Position of the wire in mower coordinates */
      double y0 = -direction * COIL_Y_OFFSET * c / maxCenter;
      double alpha0 = y0 / COIL_X_OFFSET;  // deflection
      if (state != FOLLOW_STATE_FOLLOW) {
        state = FOLLOW_STATE_FOLLOW;
        tries = 0;
      } else {
        if (travelTimeSinceUpdate > 0) {
          double d0 = -vel.angular.z + (alpha0 - lastAlpha0) / travelTimeSinceUpdate; /* rad/s */
          double f = exp(-travelTimeSinceUpdate / averageInterval);
          drift = drift * f + d0 * (1 - f);
        }
      }
      vel.angular.z = drift + alpha0 / 2;  // Correct deflection within 2 seconds
      if (vel.angular.z > ANGULAR_SPEED)
        vel.angular.z = ANGULAR_SPEED;
      else if (vel.angular.z < -ANGULAR_SPEED)
        vel.angular.z = -ANGULAR_SPEED;
      travelTimeSinceUpdate = 0;
      lastAlpha0 = alpha0;
    }
  }
  /* And stop */
  vel.linear.x = 0;
  vel.angular.z = 0;
  cmd_vel_pub.publish(vel);
  shutdownConnections();
  return toReturn;
}

void PerimeterBase::enter() {
  paused = aborted = false;
}

void PerimeterBase::exit() {
}

void PerimeterBase::reset() {
}

uint8_t PerimeterBase::get_sub_state() {
  return 1;
}

uint8_t PerimeterBase::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

bool PerimeterBase::needs_gps() {
  return false;
}

bool PerimeterBase::mower_enabled() {
  // No mower during docking
  return false;
}

void PerimeterBase::command_home() {
}

void PerimeterBase::command_start() {
}

void PerimeterBase::command_s1() {
}

void PerimeterBase::command_s2() {
}

bool PerimeterBase::redirect_joystick() {
  return false;
}

void PerimeterBase::handle_action(std::string action) {
}

/**
 * @return success
 */
int PerimeterBase::setupConnections() {
  perimeterSubscriber = n->subscribe("/mower/perimeter", 0, perimeterReceived, ros::TransportHints().tcpNoDelay(true));
  perimeterClient = n->serviceClient<mower_msgs::PerimeterControlSrv>("/mower_service/perimeter_listen");
  mower_msgs::PerimeterControlSrv p;
  direction = config.perimeter_signal > 0 ? 1 : -1;
  p.request.listenOn = direction * config.perimeter_signal;
  if (perimeterClient.call(p)) {
    ROS_INFO("Perimeter activated");
    return 1;
  }
  ROS_ERROR("Failed to activate perimeter");
  return 0;
}

void PerimeterMoveToGpsBehavior::enter() {
  setGPS(true);
}

std::string PerimeterMoveToGpsBehavior::state_name() {
  return "UNDOCKING";
}

Behavior* PerimeterMoveToGpsBehavior::arrived() {
  return travelled >= config.undock_distance - 0.9 ? &MowingBehavior::INSTANCE : NULL;
}
