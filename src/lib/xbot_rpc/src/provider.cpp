#include "xbot_rpc/provider.h"

#include "xbot_rpc/RegisterMethodsSrv.h"

namespace xbot_rpc {

void RpcProvider::init() {
  ros::NodeHandle n;
  request_sub =
      n.subscribe(TOPIC_REQUEST, 0, &RpcProvider::handleRequest, this, ros::TransportHints().tcpNoDelay(true));
  response_pub = n.advertise<xbot_rpc::RpcResponse>(TOPIC_RESPONSE, 100);
  error_pub = n.advertise<xbot_rpc::RpcError>(TOPIC_ERROR, 100);
  registration_client = n.serviceClient<xbot_rpc::RegisterMethodsSrv>(SERVICE_REGISTER_METHODS);
  publishMethods();
}

void RpcProvider::publishMethods() {
  xbot_rpc::RegisterMethodsSrv srv;
  srv.request.node_id = node_id;
  srv.request.methods.reserve(methods.size());
  for (const auto& [method_id, _] : methods) {
    srv.request.methods.push_back(method_id);
  }

  ros::Rate retry_delay(1);
  for (int i = 0; i < 10; i++) {
    if (registration_client.call(srv)) {
      ROS_INFO_STREAM("successfully registered methods for " << node_id);
      break;
    }
    ROS_ERROR_STREAM("Error registering methods for " << node_id << ". Retrying.");
    retry_delay.sleep();
  }
}

void RpcProvider::handleRequest(const xbot_rpc::RpcRequest::ConstPtr& request) {
  // Look up the method. Ignore if not found, as it might be handled by another node.
  auto it = methods.find(request->method);
  if (it == methods.end()) {
    return;
  }

  // Parse the parameters.
  nlohmann::basic_json<> params;
  if (!request->params.empty()) {
    try {
      params = nlohmann::ordered_json::parse(request->params);
    } catch (const nlohmann::json::parse_error& e) {
      publishError(request, RpcError::ERROR_INVALID_JSON, std::string("Invalid parameters JSON: ") + e.what());
      return;
    }
  }

  // Execute the method callback and publish the response.
  try {
    nlohmann::basic_json<> response = it->second(request->method, params);
    publishResponse(request, response);
  } catch (const RpcException& e) {
    publishError(request, e.code, e.message, e.data);
  } catch (std::exception& e) {
    publishError(request, RpcError::ERROR_INTERNAL, std::string("Internal error: ") + e.what());
  }
}

void RpcProvider::publishResponse(const xbot_rpc::RpcRequest::ConstPtr& request,
                                  const nlohmann::basic_json<>& response) {
  if (request->id.empty()) {
    return;
  }
  xbot_rpc::RpcResponse response_msg;
  response_msg.result = response.dump();
  response_msg.id = request->id;
  response_pub.publish(response_msg);
}

void RpcProvider::publishError(const xbot_rpc::RpcRequest::ConstPtr& request, int16_t code, const std::string& message,
                               const nlohmann::basic_json<>& data) {
  if (request->id.empty()) {
    return;
  }
  xbot_rpc::RpcError err_msg;
  err_msg.id = request->id;
  err_msg.code = code;
  err_msg.message = message;
  if (data != nullptr) {
    err_msg.data = data.dump();
  }
  error_pub.publish(err_msg);
}

}  // namespace xbot_rpc
