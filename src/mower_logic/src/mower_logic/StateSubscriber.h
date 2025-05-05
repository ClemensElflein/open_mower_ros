//
// Created by clemens on 29.11.24.
//

#ifndef STATESUBSCRIBER_H
#define STATESUBSCRIBER_H

#include "ros/ros.h"

template <typename MESSAGE>
class StateSubscriber {
 public:
  explicit StateSubscriber(const std::string &topic);

  void Start(ros::NodeHandle *n);

  MESSAGE getMessage();

  bool hasMessage();

  ros::Time getMessageTime();

  void setMessage(const MESSAGE &message);

 private:
  std::string topic_;
  std::mutex message_mutex_{};
  MESSAGE message_{};
  ros::Time last_message_time_{};
  bool has_message_ = false;
  ros::Subscriber subscriber_{};
};

template <typename MESSAGE>
StateSubscriber<MESSAGE>::StateSubscriber(const std::string &topic) : topic_{topic} {
}

template <typename MESSAGE>
void StateSubscriber<MESSAGE>::Start(ros::NodeHandle *n) {
  subscriber_ = n->subscribe(topic_, 10, &StateSubscriber::setMessage, this);
}

template <typename MESSAGE>
MESSAGE StateSubscriber<MESSAGE>::getMessage() {
  std::lock_guard<std::mutex> lk{message_mutex_};
  return message_;
}

template <typename MESSAGE>
bool StateSubscriber<MESSAGE>::hasMessage() {
  std::lock_guard<std::mutex> lk{message_mutex_};
  return has_message_;
}

template <typename MESSAGE>
ros::Time StateSubscriber<MESSAGE>::getMessageTime() {
  std::lock_guard<std::mutex> lk{message_mutex_};
  return last_message_time_;
}

template <typename MESSAGE>
void StateSubscriber<MESSAGE>::setMessage(const MESSAGE &message) {
  std::lock_guard<std::mutex> lk{message_mutex_};
  last_message_time_ = ros::Time::now();
  message_ = message;
  has_message_ = true;
}

#endif  // STATESUBSCRIBER_H
