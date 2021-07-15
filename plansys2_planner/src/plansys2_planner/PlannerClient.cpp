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

  RCLCPP_INFO(node_->get_logger(), "Sending planning problem to planner node.");
  auto send_goal_options = rclcpp_action::Client<SolvePlan>::SendGoalOptions();
  auto future_goal = client_ptr_->async_send_goal(goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(node_, future_goal) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Planning problem not accepted by planner action server.");
  }

  auto goal_handle = future_goal.get();
  auto future_result = client_ptr_->async_get_result(goal_handle);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(15)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Action server timed out or could not generate a plan.");
    return {};
  }

  if (future_result.get().result->success) {
    RCLCPP_INFO(node_->get_logger(), "Plan successfully generated.");

    return future_result.get().result->plan;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Action server did not produce a valid plan.");
    return {};
  }
}

}  // namespace plansys2
