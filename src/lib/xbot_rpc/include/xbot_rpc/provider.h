#pragma once

#include <ros/ros.h>
#include <xbot_rpc/RpcError.h>
#include <xbot_rpc/RpcRequest.h>
#include <xbot_rpc/RpcResponse.h>

#include <nlohmann/json.hpp>

#include "constants.h"

#define RPC_METHOD(id, body) \
  { id, [](const std::string& topic, const nlohmann::basic_json<>& params) body }

namespace xbot_rpc {

typedef std::function<nlohmann::basic_json<>(const std::string& topic, const nlohmann::basic_json<>& params)>
    callback_t;

class RpcException : public std::exception {
 public:
  const int16_t code;
  const std::string message;
  const nlohmann::basic_json<> data;

  RpcException(int16_t code, const std::string& message, const nlohmann::basic_json<>& data = nullptr)
      : code(code), message(message), data(data) {
  }
};

class RpcProvider {
 private:
  std::string node_id;
  std::map<std::string, callback_t> methods;

  ros::Subscriber request_sub;
  ros::Publisher response_pub;
  ros::Publisher error_pub;
  ros::ServiceClient registration_client;

  void handleRequest(const xbot_rpc::RpcRequest::ConstPtr& request);
  void publishResponse(const xbot_rpc::RpcRequest::ConstPtr& request, const nlohmann::basic_json<>& response);
  void publishError(const xbot_rpc::RpcRequest::ConstPtr& request, int16_t code, const std::string& message,
                    const nlohmann::basic_json<>& data = nullptr);

 public:
  RpcProvider(const std::string& node_id, const std::map<std::string, callback_t>& methods = {}) : node_id(node_id), methods(methods) {}

  void init();

  void addMethod(const std::string& id, callback_t callback) {
    methods.emplace(id, callback);
  }

  void addMethod(const std::pair<std::string, callback_t>& method) {
    methods.insert(method);
  }

  void publishMethods();
};

}  // namespace xbot_rpc
