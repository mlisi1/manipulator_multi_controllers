#include "manipulator_controller/multi_controller.hpp"


using config_type = controller_interface::interface_configuration_type;


namespace manipulator_controller
{
    

MultiController::MultiController() : controller_interface::ControllerInterface(),  logger(rclcpp::get_logger("MultiController")) {}



void MultiController::description_callback(const std::shared_ptr<std_msgs::msg::String> description) {

    description_msg = description->data;
    RCLCPP_INFO_STREAM(logger, "Robot description received");

}


void MultiController::command_callback(const std::shared_ptr<controller_error_msgs::msg::DesiredConfiguration> msg) {

  desired_pose = msg->pose;
  desired_twist = msg->twist;
  RCLCPP_DEBUG_STREAM(logger, "Received desired position");

}



void MultiController::update_values() {

  //Update position and velocity
  RCLCPP_DEBUG_STREAM(logger, "Updating q and q_dot");
  for (size_t i = 0; i < joint_position_state_interface_.size(); i++) {

      hardware_interface::LoanedStateInterface & position_interface = joint_position_state_interface_[i].get();
      q(i) = position_interface.get_value();

      hardware_interface::LoanedStateInterface & velocity_interface = joint_velocity_state_interface_[i].get();
      q_dot(i) = velocity_interface.get_value();

  }   

  //Update robot matrices
  param_solver->JntToMass(q, M);
  param_solver->JntToCoriolis(q, q_dot, C);
  param_solver->JntToGravity(q, G);


  attitude_desired = Eigen::Quaterniond(
    desired_pose.orientation.w,
    desired_pose.orientation.x,
    desired_pose.orientation.y,
    desired_pose.orientation.z
  );


  xi_dot_desired <<   desired_twist.linear.x,
              desired_twist.linear.y,
              desired_twist.linear.z,
              desired_twist.angular.x,
              desired_twist.angular.y,
              desired_twist.angular.z;

  xi_desired.head<3>() << desired_pose.position.x, desired_pose.position.y, desired_pose.position.z;
  // Eigen::Vector3d desired_euler = attitude_desired.toRotationMatrix().eulerAngles(2, 1, 0);
  // xi_desired.tail<3>() << desired_euler[2], desired_euler[1], desired_euler[0];

  jac_solver->JntToJac(q, J);

  fk_solver->JntToCart(q, ee_fk_frame);

  double x, y, z, w;
  ee_fk_frame.M.GetQuaternion(x, y, z, w);
  attitude = Eigen::Quaterniond(w, x, y, z);

  xi.head<3>() << ee_fk_frame.p.data[0], ee_fk_frame.p.data[1], ee_fk_frame.p.data[2];
  xi_dot = J.data * q_dot.data;

  attitude_desired.normalize();
  attitude.normalize();


}



void MultiController::computed_torque() {

  KDL::JntArrayVel jnt_q_qdot;
  jnt_q_qdot.q = q;
  jnt_q_qdot.qdot = q_dot;
  jac_dot_solver->JntToJacDot(jnt_q_qdot, dJ);

  M_xi = J.data * M.data.inverse() * J.data.transpose();
  M_xi = M_xi.inverse();

  h_xi = J.data.transpose().completeOrthogonalDecomposition().pseudoInverse() * (C.data + G.data) - M_xi * dJ.data * q_dot.data;

}




void MultiController::backstepping() {

  q_ref.data = J.data.completeOrthogonalDecomposition().pseudoInverse() * (xi_dot_desired + Kp * err);
  s = J.data.completeOrthogonalDecomposition().pseudoInverse() * (err_dot + Kp * err);

  param_solver->JntToCoriolis(q, q_ref, C_backstepping);

}




void MultiController::publish_error(rclcpp::Time time) {

  error_msg.stamp = time;

  error_msg.position_error.position.x = err[0];
  error_msg.position_error.position.y = err[1];
  error_msg.position_error.position.z = err[2];

  error_msg.position_error.orientation.x = attitude_error.x();
  error_msg.position_error.orientation.y = attitude_error.y();
  error_msg.position_error.orientation.z = attitude_error.z();
  error_msg.position_error.orientation.w = attitude_error.w();

  error_msg.velocity_error.linear.x = err_dot[0];
  error_msg.velocity_error.linear.y = err_dot[1];
  error_msg.velocity_error.linear.z = err_dot[2];

  error_msg.velocity_error.angular.x = err_dot[3];
  error_msg.velocity_error.angular.y = err_dot[4];
  error_msg.velocity_error.angular.z = err_dot[5];

  error_pub->publish(error_msg);

}






controller_interface::CallbackReturn MultiController::on_init()
{
  // should have error handling
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);



  controller_type = auto_declare<std::string>("controller_type", "");

  if (controller_type != "PD" && controller_type != "B" && controller_type != "CT") {

    RCLCPP_ERROR_STREAM(logger, "Controller type not recognized: " << controller_type);
    return CallbackReturn::ERROR;

  }


  switch (stringToControlMode.find(controller_type)->second) {

    case ControlMode::PD:

      Kp_p = auto_declare<double>("PD.Kp_p", 0.0);
      Kp_o = auto_declare<double>("PD.Kp_o", 0.0);
      Kv_p = auto_declare<double>("PD.Kv", 0.0);
      break;


    case ControlMode::B:
      Kp_p = auto_declare<double>("B.Kp_p", 0.0);
      Kp_o = auto_declare<double>("B.Kp_o", 0.0);
      Kv_p = auto_declare<double>("B.Kv", 0.0);
      break;

    case ControlMode::CT:

      Kp_p = auto_declare<double>("CT.Kp_p", 0.0);
      Kp_o = auto_declare<double>("CT.Kp_o", 0.0);
      Kv_p = auto_declare<double>("CT.Kv_p", 0.0);
      Kv_o = auto_declare<double>("CT.Kv_0", 0.0);
      break;

  }

  description_topic = auto_declare<std::string>("description_topic", "");

  base_link = auto_declare<std::string>("base_link", "");
  ee_link = auto_declare<std::string>("ee_link", "");


  if (base_link == "" || ee_link == "") {
    RCLCPP_ERROR_STREAM(logger, "Base link or end effector link not specified");
    return CallbackReturn::ERROR;
  }


  error_pub = get_node()->create_publisher<controller_error_msgs::msg::OperationalSpaceError>("~/error", 10);

  if (description_topic == "") {
    RCLCPP_ERROR_STREAM(logger, "Robot description topic is empty");
    return CallbackReturn::ERROR;
  }


    auto callback = std::bind(
        &MultiController::description_callback,
        this,
        std::placeholders::_1
    );
  

  rclcpp::QoS qos(10); 
  qos.transient_local();

  robot_description_subscriber = get_node()->create_subscription<std_msgs::msg::String>(
      description_topic, 
      qos, 
      callback);


  // desired_pose.pose.position.x = -0.4;
  desired_pose.position.x = 0.088;
  desired_pose.position.y = 0.0;
  desired_pose.position.z = .9;
  desired_pose.orientation.x = 1.0;
  desired_pose.orientation.y = 0.0;
  desired_pose.orientation.z = 0.0;
  desired_pose.orientation.w = 0.0;


  desired_twist.linear.x = 0.0;
  desired_twist.linear.y = 0.0;
  desired_twist.linear.z = 0.0;
  desired_twist.angular.x = 0.0;
  desired_twist.angular.y = 0.0;
  desired_twist.angular.z = 0.0;

  attitude_desired = Eigen::Quaterniond(
    desired_pose.orientation.w,
    desired_pose.orientation.x,
    desired_pose.orientation.y,
    desired_pose.orientation.z
  );

  xi_dot_desired <<   desired_twist.linear.x,
                      desired_twist.linear.y,
                      desired_twist.linear.z,
                      desired_twist.angular.x,
                      desired_twist.angular.y,
                      desired_twist.angular.z;




  //Allow for robot description to be published
  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  return CallbackReturn::SUCCESS;
}






controller_interface::InterfaceConfiguration MultiController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  RCLCPP_INFO_STREAM(logger, "Configuring commad interfaces");

  return conf;
}


controller_interface::InterfaceConfiguration MultiController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  RCLCPP_INFO_STREAM(logger, "Configuring state interfaces");

  return conf;
}











controller_interface::CallbackReturn MultiController::on_configure(const rclcpp_lifecycle::State &) {


  auto callback = std::bind(&MultiController::command_callback, this, std::placeholders::_1);

  command_subscriber_ =
    get_node()->create_subscription<controller_error_msgs::msg::DesiredConfiguration>(
              "~/end_effector_configuration", 
              rclcpp::SystemDefaultsQoS(), 
              callback);

  if (description_msg == "") {
    RCLCPP_ERROR_STREAM(logger, "Robot description not received yet");  
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(logger, "Parsing model");
  model.initString(description_msg);
  kdl_parser::treeFromUrdfModel(model, tree);
  tree.getChain(base_link, ee_link, chain);
  dofs = chain.getNrOfJoints();

  Kp = Eigen::MatrixXd::Identity(6, 6) * Kp_p;
  Kv = Eigen::MatrixXd::Identity(6, 6) * Kv_p;
  Kv_j = Eigen::MatrixXd::Identity(dofs, dofs) * Kv_p;

  Kp(5, 5) = Kp_o; Kp(4, 4) = Kp_o; Kp(3, 3) = Kp_o;
  Kv(5, 5) = Kv_o; Kv(4, 4) = Kv_o; Kv(3, 3) = Kv_o;

  switch (stringToControlMode.find(controller_type)->second) {

    case ControlMode::PD:

      RCLCPP_INFO_STREAM(logger, "Kp:\n" << Kp);
      RCLCPP_INFO_STREAM(logger, "Kv:\n" << Kv_j);
      break;


    case ControlMode::B:

      RCLCPP_INFO_STREAM(logger, "Kp\n:" << Kp);
      RCLCPP_INFO_STREAM(logger, "Kv\n:" << Kv_j);
      break;

    case ControlMode::CT:

      RCLCPP_INFO_STREAM(logger, "Kp\n:" << Kp);
      RCLCPP_INFO_STREAM(logger, "Kv\n:" << Kv);
      break;

  }


  fk_solver =  std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
  jac_solver = std::make_shared<KDL::ChainJntToJacSolver>(chain);
  param_solver = std::make_shared<KDL::ChainDynParam>(chain, KDL::Vector(0, 0, -9.81));
  jac_dot_solver = std::make_shared<KDL::ChainJntToJacDotSolver>(chain);

  q.resize(dofs);
  q_dot.resize(dofs);
  torque.resize(dofs);

  q_ref.resize(dofs);

  M.resize(dofs);
  C.resize(dofs);
  G.resize(dofs);
  C_backstepping.resize(dofs);

  J.resize(dofs);
  dJ.resize(dofs);

  RCLCPP_INFO_STREAM(logger, "Configuration complete");
  return CallbackReturn::SUCCESS;
}





controller_interface::CallbackReturn MultiController::on_activate(const rclcpp_lifecycle::State &)
{
  // clear out vectors in case of restart
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_effort_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  joint_effort_state_interface_.clear();

  // assign command interfaces
  for (auto & interface : command_interfaces_)
  {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }

  // assign state interfaces
  for (auto & interface : state_interfaces_)
  {
    state_interface_map_[interface.get_interface_name()]->push_back(interface);  
  }

  RCLCPP_INFO_STREAM(logger, "Activation complete");

  return CallbackReturn::SUCCESS;
}















controller_interface::return_type MultiController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period) {

  
    update_values();

    attitude_error = attitude_desired * attitude.inverse();
    attitude_error.normalize();

    err = xi_desired - xi;
    err_dot = xi_dot_desired - xi_dot;
    err.tail<3>() << attitude_error.x(), attitude_error.y(), attitude_error.z();



    switch (stringToControlMode.find(controller_type)->second) {

      case ControlMode::PD:

        torque = J.data.transpose() * Kp * err - Kv_j * q_dot.data + G.data;
        RCLCPP_DEBUG_STREAM(logger, "Torque: " << torque.transpose());
        break;


      case ControlMode::B:

        backstepping();
        torque = C_backstepping.data + G.data + Kv_j * s + J.data.transpose() * err;
        RCLCPP_DEBUG_STREAM(logger, "Torque: " << torque.transpose());
        break;

      case ControlMode::CT:

        computed_torque();
        torque = J.data.transpose() * (M_xi * (xi_ddot_desired + Kv * err_dot + Kp * err) + h_xi);
        RCLCPP_DEBUG_STREAM(logger, "Torque: " << torque.transpose());
        break;


    }






    for (size_t i = 0; i < joint_effort_command_interface_.size(); i++)
    {
      joint_effort_command_interface_[i].get().set_value(torque[i]);
    }

    publish_error(time);


    return controller_interface::return_type::OK;

}





























controller_interface::CallbackReturn MultiController::on_deactivate(const rclcpp_lifecycle::State &)
{
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MultiController::on_cleanup(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MultiController::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MultiController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}



} // namespace manipulator_controller




#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  manipulator_controller::MultiController, controller_interface::ControllerInterface)
