// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "plansys2_planner/PlannerClient.hpp"

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

namespace plansys2
{

PlannerClient::PlannerClient()
{
  node_ = rclcpp::Node::make_shared("planner_client");

  get_plan_client_ = node_->create_client<plansys2_msgs::srv::GetPlan>("planner/get_plan");
  client_ptr_ = rclcpp_action::create_client<SolvePlan>(node_, "solve_plan");
}

std::optional<plansys2_msgs::msg::Plan>
PlannerClient::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace)
{
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(30))) {
       RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
       return {};
     }

     auto goal_msg = SolvePlan::Goal();
     goal_msg.problem.domain = domain;
     goal_msg.problem.problem = problem;

     RCLCPP_INFO(node_->get_logger(), "Sending goal");
     auto send_goal_options = rclcpp_action::Client<SolvePlan>::SendGoalOptions();
//  50    send_goal_options.goal_response_callback =
//  51      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
//  52    send_goal_options.feedback_callback =
//  53      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
//  54    send_goal_options.result_callback =jj
//  55      std::bind(&FibonacciActionClient::result_callback, this, _1);

  auto future_goal = client_ptr_->async_send_goal(goal_msg, send_goal_options);

if (rclcpp::spin_until_future_complete(node_, future_goal)
    != rclcpp::executor::FutureReturnCode::SUCCESS)
{
  RCLCPP_ERROR(node_->get_logger(), "Failed");
}

// if (future_goal.wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
//   {
//     RCLCPP_ERROR_STREAM(node_->get_logger(), "Timed out waiting for goal response callback");
//     return {};
//   }

  auto goal_handle = future_goal.get();

  auto future_result =  client_ptr_->async_get_result(goal_handle);

  // while (!get_plan_client_->wait_for_service(std::chrono::seconds(30))) {
  //   if (!rclcpp::ok()) {
  //     return {};
  //   }
  //   RCLCPP_ERROR_STREAM(
  //     node_->get_logger(),
  //     get_plan_client_->get_service_name() <<
  //       " service  client: waiting for service to appear...");
  // }

  // auto request = std::make_shared<plansys2_msgs::srv::GetPlan::Request>();
  // request->domain = domain;
  // request->problem = problem;

  // auto future_result = get_plan_client_->async_send_request(request);
  RCLCPP_ERROR(node_->get_logger(), "Got here");
   if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(15)) !=
     rclcpp::FutureReturnCode::SUCCESS)
   {
     return {};
   }
  RCLCPP_ERROR(node_->get_logger(), "And here");

   if (future_result.get().result->success) {
       RCLCPP_ERROR(node_->get_logger(), "Got plan");

     return future_result.get().result->plan;
   } else {
      RCLCPP_ERROR(node_->get_logger(), "Nope");

     return {};
   }
}

}  // namespace plansys2
