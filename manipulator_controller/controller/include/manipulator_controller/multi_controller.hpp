// Copyright 2023 ros2_control Development Team
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

#ifndef MULTI_CONTROLLER_HPP_
#define MULTI_CONTROLLER_HPP_


#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"

#include <Eigen/Dense>
#include <std_msgs/msg/string.hpp>
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>


#include <controller_error_msgs/msg/operational_space_error.hpp>
#include <controller_error_msgs/msg/desired_configuration.hpp>
#include <controller_error_msgs/msg/gains.hpp>

#include <panda_generated/thunder_panda.h>


namespace manipulator_controller
{

enum class ControlMode {
    CT,
    B,
    PD,
    AB,
};


const std::unordered_map<std::string, ControlMode> stringToControlMode = {
    {"CT", ControlMode::CT},
    {"B", ControlMode::B},
    {"PD", ControlMode::PD},
    {"AB", ControlMode::AB}
};





class MultiController : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_INTERFACE_PUBLIC
  MultiController();

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;



    void description_callback(const std::shared_ptr<std_msgs::msg::String> description);
    void command_callback(const std::shared_ptr<controller_error_msgs::msg::DesiredConfiguration> msg);
    void gain_callback(const std::shared_ptr<controller_error_msgs::msg::Gains> gains);
    void update_values();
    void publish_error(rclcpp::Time time);
    void computed_torque();
    void backstepping();
    void adaptive_backstepping();

protected:

  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  rclcpp::Subscription<controller_error_msgs::msg::DesiredConfiguration>::SharedPtr command_subscriber_;

  rclcpp::Subscription<controller_error_msgs::msg::Gains>::SharedPtr gains_subscriber_;
  
  rclcpp::Publisher<controller_error_msgs::msg::OperationalSpaceError>::SharedPtr error_pub;

  bool new_msg_ = false;
  bool new_single_msg_ = false;
  rclcpp::Time start_time_;

  thunder_panda robot;

  trajectory_msgs::msg::JointTrajectoryPoint point_interp_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_effort_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_effort_state_interface_;

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
    command_interface_map_ = {
      {"position", &joint_position_command_interface_},
      {"velocity", &joint_velocity_command_interface_},
      {"effort", &joint_effort_command_interface_}};

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
    state_interface_map_ = {
      {"position", &joint_position_state_interface_},
      {"velocity", &joint_velocity_state_interface_},
      {"effort", &joint_effort_state_interface_}};


public:
  rclcpp::Logger logger;

private:

  double Kp_p, Kp_o, Kv_p, Kv_o, r;

  urdf::Model model;
  KDL::Tree tree;
  KDL::Chain chain;
  int dofs;

  Eigen::MatrixXd Kp, Kv, Kv_j, R;


  std::string description_topic, description_msg;
  std::string controller_type;
  std::string base_link, ee_link;

  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber;
  


  controller_error_msgs::msg::OperationalSpaceError error_msg;
  geometry_msgs::msg::Pose desired_pose;
  geometry_msgs::msg::Twist desired_twist;

  int msg_index = 0;
  bool first = true;


  KDL::JntArray q;
  KDL::JntArray q_dot;
  KDL::JntArray q_ddot;

  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
  std::shared_ptr<KDL::ChainDynParam> param_solver;
  std::shared_ptr<KDL::ChainJntToJacDotSolver> jac_dot_solver;


  KDL::JntSpaceInertiaMatrix M;
  KDL::JntArray C, G, C_backstepping;

  Eigen::MatrixXd M_xi;
  Eigen::VectorXd h_xi;

  KDL::JntArray q_dot_ref;
  Eigen::VectorXd q_ddot_ref;
  Eigen::VectorXd s;


  Eigen::VectorXd xi = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd xi_dot = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd xi_ddot = Eigen::VectorXd::Zero(6);

  Eigen::VectorXd xi_desired = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd xi_dot_desired = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd xi_ddot_desired = Eigen::VectorXd::Zero(6);

  Eigen::VectorXd err = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd err_dot = Eigen::VectorXd::Zero(6);
  

  Eigen::Quaterniond attitude;
  Eigen::Quaterniond attitude_desired;

  Eigen::Quaterniond attitude_error;

  Eigen::VectorXd torque;


  KDL::Jacobian J;
  KDL::Jacobian dJ;

  KDL::Frame ee_fk_frame;


	Eigen::VectorXd params;
  std::string yaml_file;
  Eigen::MatrixXd Yr;
  Eigen::VectorXd pi_hat;



};

}  // namespace ros2_control_demo_example_7

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_CONTROLLER_HPP_