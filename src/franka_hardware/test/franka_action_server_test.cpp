#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "test_utils.hpp"

using namespace std::chrono_literals;

class FrankaActionServerTests
    : public ::testing::TestWithParam<
          std::pair<std::function<void(std::shared_ptr<MockRobot> mock_robot)>,
                    rclcpp_action::ResultCode>> {};

template <typename action_client_type>
void get_action_service_response(
    std::function<void(std::shared_ptr<MockRobot> mock_robot)> mock_function,
    const std::string& action_name,
    rclcpp_action::ResultCode result_code) {
  auto mock_robot = std::make_shared<MockRobot>();
  mock_function(mock_robot);

  std::string arm_id{"fr3"};
  franka_hardware::FrankaHardwareInterface franka_hardware_interface(mock_robot, arm_id);

  const auto hardware_info = createHardwareInfo();
  franka_hardware_interface.on_init(hardware_info);

  auto node = rclcpp::Node::make_shared("test_node");

  auto client = rclcpp_action::create_client<action_client_type>(node, action_name);
  if (!client->wait_for_action_server(20s)) {
    ASSERT_TRUE(false) << "Action not available after waiting";
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  bool is_finished = false;
  auto goal_msg = typename action_client_type::Goal();
  auto send_goal_options = typename rclcpp_action::Client<action_client_type>::SendGoalOptions();
  send_goal_options.goal_response_callback = [&](const auto& future_result) {
    auto goal_handle = future_result.get();
    ASSERT_TRUE(goal_handle);
  };
  send_goal_options.feedback_callback = [&](auto, auto) { ASSERT_TRUE(false); };
  send_goal_options.result_callback = [&](const auto& result) {
    ASSERT_EQ(result.code, result_code);
    is_finished = true;
  };

  auto action_accepted = client->async_send_goal(goal_msg);
  auto start_point = std::chrono::system_clock::now();
  auto end_point = start_point + 5s;
  while (action_accepted.wait_for(0s) != std::future_status::ready) {
    executor.spin_some();

    ASSERT_LE(std::chrono::system_clock::now(), end_point);
  }
  auto goal_handle = action_accepted.get();

  auto result = client->async_get_result(goal_handle, send_goal_options.result_callback);
  start_point = std::chrono::system_clock::now();
  end_point = start_point + 5s;
  while (!is_finished || result.wait_for(0s) != std::future_status::ready) {
    executor.spin_some();

    ASSERT_LE(std::chrono::system_clock::now(), end_point);
  }

  ASSERT_TRUE(is_finished);
}

TEST_P(FrankaActionServerTests,
       whenErrorRecoveryActionTriggered_thenErrorRecoveryServiceCallExecuted) {
  auto param = GetParam();

  get_action_service_response<franka_msgs::action::ErrorRecovery>(
      param.first, "action_server/error_recovery", param.second);
}

INSTANTIATE_TEST_SUITE_P(
    FrankaActionServerTestsInstantiation,
    FrankaActionServerTests,
    ::testing::Values(std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery()).Times(1);
                          },
                          rclcpp_action::ResultCode::SUCCEEDED),
                      std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery())
                                .Times(1)
                                .WillRepeatedly(testing::Throw(franka::CommandException("")));
                          },
                          rclcpp_action::ResultCode::ABORTED),
                      std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery())
                                .Times(1)
                                .WillRepeatedly(testing::Throw(franka::NetworkException("")));
                          },
                          rclcpp_action::ResultCode::ABORTED)));
