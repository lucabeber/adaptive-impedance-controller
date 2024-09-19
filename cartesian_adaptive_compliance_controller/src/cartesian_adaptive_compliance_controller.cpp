#include <cartesian_adaptive_compliance_controller/cartesian_adaptive_compliance_controller.h>

#include <iostream>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"

using namespace std;
namespace cartesian_adaptive_compliance_controller
{

CartesianAdaptiveComplianceController::CartesianAdaptiveComplianceController()
// Base constructor won't be called in diamond inheritance, so call that
// explicitly
: Base::CartesianControllerBase(),
  MotionBase::CartesianMotionController(),
  ForceBase::CartesianForceController()
{
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || true
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianAdaptiveComplianceController::on_init()
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_init() != TYPE::SUCCESS || ForceBase::on_init() != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  auto_declare<std::string>("compliance_ref_link", "");

  constexpr double default_lin_stiff = 500.0;
  constexpr double default_rot_stiff = 50.0;
  auto_declare<double>("stiffness.trans_x", default_lin_stiff);
  auto_declare<double>("stiffness.trans_y", default_lin_stiff);
  auto_declare<double>("stiffness.trans_z", default_lin_stiff);
  auto_declare<double>("stiffness.rot_x", default_rot_stiff);
  auto_declare<double>("stiffness.rot_y", default_rot_stiff);
  auto_declare<double>("stiffness.rot_z", default_rot_stiff);

  return TYPE::SUCCESS;
}
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianAdaptiveComplianceController::init(
  const std::string & controller_name)
{
  using TYPE = controller_interface::return_type;
  if (MotionBase::init(controller_name) != TYPE::OK || ForceBase::init(controller_name) != TYPE::OK)
  {
    return TYPE::ERROR;
  }

  auto_declare<std::string>("compliance_ref_link", "");

  return TYPE::OK;
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianAdaptiveComplianceController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_configure(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_configure(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  // Make sure compliance link is part of the robot chain
  m_compliance_ref_link = get_node()->get_parameter("compliance_ref_link").as_string();
  if (!Base::robotChainContains(m_compliance_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_compliance_ref_link
                                                    << " is not part of the kinematic chain from "
                                                    << Base::m_robot_base_link << " to "
                                                    << Base::m_end_effector_link);
    return TYPE::ERROR;
  }

  // Make sure sensor wrenches are interpreted correctly
  ForceBase::setFtSensorReferenceFrame(m_compliance_ref_link);

  m_fk_solver.reset(new KDL::ChainFkSolverVel_recursive(Base::m_robot_chain));
  old_z = 0.098;
  // Read data from files
  dataReader(m_x_coordinates, m_y_coordinates, m_z_values, m_stiffness_values, m_damping_values);
  cout << m_x_coordinates.size() << " " << m_y_coordinates.size() << " " << m_z_values.size() << " "
       << m_stiffness_values.size() << " " << m_damping_values.size() << endl;
  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianAdaptiveComplianceController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // Base::on_activation(..) will get called twice,
  // but that's fine.
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_activate(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_activate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }
  // Subscriber
  m_ft_sensor_wrench_subscriber =
    get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_node()->get_name() + std::string("/ft_sensor_wrench"), 10,
      std::bind(&CartesianAdaptiveComplianceController::ftSensorWrenchCallback, this,
                std::placeholders::_1));
  // Publisher
  m_data_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    std::string("/adaptive_stiffness_data"), 10);

  m_target_pose_publisher = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
    get_node()->get_name() + std::string("/target_frame"), 10);

  m_starting_pose(0) = MotionBase::m_current_frame.p.x();
  m_starting_pose(1) = MotionBase::m_current_frame.p.y();
  m_starting_pose(2) = MotionBase::m_current_frame.p.z();
  cout << "Starting position: " << m_starting_pose(0) << " " << m_starting_pose(1) << " "
       << m_starting_pose(2) << endl;

  old_time = current_time = start_time = get_node()->get_clock()->now();
  Xt = 1.0;
  dXt = 0.0;
  tank_energy = 0.5 * Xt * Xt;
  tank_energy_threshold = 0.4;

  USING_NAMESPACE_QPOASES
  Options options;
  options.printLevel = PL_LOW;
  // redeclare solver with options
  min_problem = QProblem(3, 5);

  m_ft_sensor_wrench = ctrl::Vector3D::Zero();

  x_d_old << m_starting_pose(0), m_starting_pose(1), m_starting_pose(2);
  m_prev_error = ctrl::Vector6D::Zero();
  for (size_t i = 0; i < 10; i++)
  {
    m_surf_vel.push(0.0);
  }
  m_surf_vel_sum = 0.0;

  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianAdaptiveComplianceController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_deactivate(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_deactivate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }
  return TYPE::SUCCESS;
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE
controller_interface::return_type CartesianAdaptiveComplianceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianAdaptiveComplianceController::update()
#endif
{
  current_time = get_node()->get_clock()->now();
  if (abs((current_time - old_time).nanoseconds()) <
      rclcpp::Duration::from_seconds(0.0001).nanoseconds())
  {
    return controller_interface::return_type::OK;
  }
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  ctrl::Vector6D tmp = CartesianAdaptiveComplianceController::computeStiffness();
  tmp[3] = get_node()->get_parameter("stiffness.rot_x").as_double();
  tmp[4] = get_node()->get_parameter("stiffness.rot_y").as_double();
  tmp[5] = get_node()->get_parameter("stiffness.rot_z").as_double();

  m_stiffness = tmp.asDiagonal();
  m_damping = 2.0 * 0.707 * m_stiffness.cwiseSqrt();

  // Control the robot motion in such a way that the resulting net force
  // vanishes. This internal control needs some simulation time steps.
  for (int i = 0; i < Base::m_iterations; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    auto internal_period = rclcpp::Duration::from_seconds(0.02);

    // Compute the net force
    ctrl::Vector6D error = computeComplianceError();
    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error, internal_period);
  }

  // publish target frame
  // publishTargetFrame();

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();
  old_time = current_time;
  x_d_old << MotionBase::m_target_frame.p.x(), MotionBase::m_target_frame.p.y(),
    MotionBase::m_target_frame.p.z();
  return controller_interface::return_type::OK;
}

void CartesianAdaptiveComplianceController::publishTargetFrame()
{
  double timeFromStart = (current_time - start_time).nanoseconds() * 1e-9;
  int step = timeFromStart / step_seconds;

  geometry_msgs::msg::PoseStamped target_pose;
  if (step < 2)
  {
    target_pose.pose.position.z = 0.122;
  }
  else
  {
    target_pose.pose.position.z = 0.122 - (step - 2) * z_step;
  }

  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = current_time;
  target_pose.pose.position.x = m_starting_pose(0);
  target_pose.pose.position.y = m_starting_pose(1);
  target_pose.pose.orientation.x = 1.0;
  target_pose.pose.orientation.y = 0.0;
  target_pose.pose.orientation.z = 0.0;
  target_pose.pose.orientation.w = 0.0;

  m_target_pose_publisher->publish(target_pose);
}

ctrl::Vector6D CartesianAdaptiveComplianceController::computeComplianceError()
{
  ctrl::Vector6D error = computeMotionError();

  ctrl::Vector6D net_force =
    // Spring force in base orientation
    Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) * error
    // Damping force in base orientation
    - Base::displayInBaseLink(m_damping, m_compliance_ref_link) *
        Base::m_ik_solver->getEndEffectorVel()
    // Sensor and target force in base orientation
    + ForceBase::computeForceError();

  return net_force;
}

void CartesianAdaptiveComplianceController::ftSensorWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  KDL::Wrench tmp;
  tmp[0] = wrench->wrench.force.x;
  tmp[1] = wrench->wrench.force.y;
  tmp[2] = wrench->wrench.force.z;

  m_ft_sensor_wrench(0) = tmp[0];
  m_ft_sensor_wrench(1) = tmp[1];
  m_ft_sensor_wrench(2) = tmp[2];
}

ctrl::Vector6D CartesianAdaptiveComplianceController::computeStiffness()
{
  USING_NAMESPACE_QPOASES
  getEndEffectorPoseReal();

  rclcpp::Duration deltaT_ros = current_time - old_time;

  double max_pen = 0.008;
  double power_limit = 0.1;

  m_deltaT = deltaT_ros.nanoseconds() * 1e-9;
  // retrieve target position
  ctrl::Vector3D x;
  x(0) = m_x(0);
  x(1) = m_x(1);
  x(2) = m_x(2);

  // retrieve current position
  ctrl::Vector3D x_d;
  x_d(0) = MotionBase::m_target_frame.p.x();
  x_d(1) = MotionBase::m_target_frame.p.y();
  x_d(2) = MotionBase::m_target_frame.p.z();

  // retrieve current velocity
  ctrl::Vector3D velocity_error;
  velocity_error << -m_x_dot(0), -m_x_dot(1), -m_x_dot(2);

  // Get the position of the data corresponding to the current position
  int x_index = findClosestIndex(m_x_coordinates, x(0));
  int y_index = findClosestIndex(m_y_coordinates, x(1));

  // Get the z, stiffness and damping values corresponding to the current position
  double z_value = m_z_values[x_index][y_index];
  double stiffness_value = m_stiffness_values[x_index][y_index];
  double damping_value = m_damping_values[x_index][y_index];

  m_surf_vel_sum -= m_surf_vel.front();
  m_surf_vel.pop();
  double sv = (z_value - old_z) / m_deltaT;
  m_surf_vel.push(sv);
  m_surf_vel_sum += sv;
  old_z = z_value;

  // mean of the last 5 values
  double surf_vel = m_surf_vel_sum / m_surf_vel.size();

  // retrieve current velocity
  ctrl::Vector6D xdot = Base::m_ik_solver->getEndEffectorVel();

  // // retrieve material stiffness
  // ctrl::Vector3D kl = {kl_, kl_, kl_};

  // // define material steady-state position
  // ctrl::Vector3D sl = {0.0, 0.0, 0.12};
  // // define material current position
  // ctrl::Vector3D l = {0.0, 0.0, 0.12};

  // // retrieve material damping
  // ctrl::Vector3D dl = {dl_, dl_, dl_};

  // F_ref
  ctrl::Vector3D F_ref = {0.0, 0.0, 0.0};

  // if (x(2) < z_value + 0.0025)
  if (m_ft_sensor_wrench(2) < -0.5)
  {
    // penetrating material
    // l(2) = x(2);
    // kl = {kl_, kl_, kl_};
    // dl = {dl_, dl_, dl_};
    // F_ref(2) = -( stiffness_value * pow((z_value + 0.0025 - x(2)),1.35) - damping_value * pow((z_value + 0.0025 - x(2)),1.35) * (m_x_dot(2)-surf_vel) );
    // F_min(2) = -( stiffness_value * pow(max_pen,1.35) - damping_value * pow(max_pen,1.35) * (m_x_dot(2)-surf_vel) );
    // F_ref(2) = -9;
    // F_min(2) = -( stiffness_value * pow(max_pen,1.35) - damping_value * pow(max_pen,1.35) * (m_x_dot(2)-surf_vel) );
    F_ref(2) = -(stiffness_value * pow(max_pen, 1.35) -
                 damping_value * pow(max_pen, 1.35) * (m_x_dot(2) - surf_vel));
    F_min(2) = -9;
  }
  else
  {
    // free motion
    // l(2) = sl(2);
    // kl = {kl_, kl_, kl_};
    // dl = {dl_, dl_, dl_};
    F_ref(2) = 0.0;
    F_min(2) = -F_max(2);
  }

  if (tank_energy >= 1.0)
  {
    m_sigma = 0.0;
  }
  else
  {
    m_sigma = 1.0;
  }

  ctrl::Vector3D position_error;
  position_error = x_d - x;

  // Update energy

  real_t H[3 * 3] = {R(0) + Q(0) * pow(position_error(0), 2), 0, 0, 0,
                     R(1) + Q(1) * pow(position_error(1), 2), 0, 0, 0,
                     R(2) + Q(2) * pow(position_error(2), 2)};

  // -Kmin1 R1 - Fdx Q1 x1 + kd1 (R1 + Q1 x1^2)
  real_t g[3] = {
    -kd_min(0) * R(0) + (-F_ref(0) + m_damping(0, 0) * velocity_error(0)) * position_error(0) *
                          Q(0),  // + kd(0) * (R(0) + Q(0) * pow(x_d(0) - x(0),2)),
    -kd_min(1) * R(1) + (-F_ref(1) + m_damping(1, 1) * velocity_error(1)) * position_error(1) *
                          Q(1),  // + kd(1) * (R(1) + Q(1) * pow(x_d(1) - x(1),2)),
    -kd_min(2) * R(2) + (-F_ref(2) + m_damping(2, 2) * velocity_error(2)) * position_error(2) *
                          Q(2)  // + kd(2) * (R(2) + Q(2) * pow(x_d(2) - x(2),2))
  };
  // (R(0) + Q(0) * pow(x_d(0) - x(0), 2)),
  // (R(1) + Q(1) * pow(x_d(1) - x(1), 2)),
  // (R(2) + Q(2) * pow(x_d(2) - x(2), 2))

  // Constraints on  K2
  real_t lb[3] = {kd_min(0), kd_min(1), kd_min(2)};
  real_t ub[3] = {kd_max(0), kd_max(1), kd_max(2)};

  // Tank equation
  //  T =
  //  Xt^2/2 + x_tilde_x*x_tilde_dot_x*(kd_x - kd_x_min) + x_tilde_y*x_tilde_dot_y*(kd_y - kd_y_min) + x_tilde_z*x_tilde_dot_z*(kd_z - kd_z_min)

  // extract x depent on Kd from T -> this goes in A as {x(0) * xdot(0) , x(1) * xdot(1) , x(2) * xdot(2)}
  // real_t tank_A = x(0) * xdot(0) + x(1) * xdot(1) + x(2) * xdot(2);

  // Constraint equation is:
  // A * x = b + epsilon -> I need to compute b + epsilon

  // Rewriting it as tank_A * Kd will left out the following term which must be sum in the left part of the equation (so I changed sign)
  // should be  = - Xt^2/2 + kd_x_min*x_tilde_x*x_tilde_dot_x + kd_y_min*x_tilde_y*x_tilde_dot_y + kd_z_min*x_tilde_z*x_tilde_dot_z + epsilon

  // Xt^2/2 is for the first step -> then became the old tank value

  real_t T_constr_min, T_dot_min;

  energy_var_damping =
    m_sigma * velocity_error.transpose() * m_damping.block<3, 3>(0, 0) * velocity_error;

  if (tank_energy < tank_energy_threshold)
  {
    // empty tank
    cout << "empty tank" << endl;
    stiffness << kd_min(0), kd_min(1), kd_min(2), 50.0, 50.0, 50.0;
    tank_energy =
      tank_energy_threshold + energy_var_damping * m_deltaT;  // + (energy_var_stiff)*m_deltaT;
    // old_tank_energy = tank_energy;
    std_msgs::msg::Float64MultiArray m_data_msg;
    m_data_msg.data = {
      (current_time.nanoseconds() * 1e-9),                                      // Time
      x(0),                                                                     // x ee
      x(1),                                                                     // y ee
      x(2),                                                                     // z ee
      x_d(0),                                                                   // x_d ee
      x_d(1),                                                                   // y_d ee
      x_d(2),                                                                   // z_d ee
      kd(2) * position_error(2) + 2 * 0.707 * sqrt(kd(2)) * velocity_error(2),  // F_ext
      m_ft_sensor_wrench(0),                                                    // F_ft
      m_ft_sensor_wrench(1),                                                    // F_ft
      m_ft_sensor_wrench(2),                                                    // F_ft
      F_ref(2),                                                                 // F_ref
      tank_energy,                                                              // Tank
      (energy_var_stiff + energy_var_damping) * m_deltaT,                       // Tank_dot
      kd(0),                                                                    // Kd_x
      kd(1),                                                                    // Kd_y
      kd(2),                                                                    // Kd_z
      kd_max(2),                                                                // Kd_z max
      kd_min(2),                                                                // Kd_z min
      (z_value + 0.0025 - x(2)),                                                // penetration
      stiffness_value,                                                          // K_surf
      damping_value,                                                            // D_surf
      F_min(2),                                                                 // F_min
      (stiffness_value * pow((z_value + 0.0025 - x(2)), 1.35) -
       damping_value * pow((z_value + 0.0025 - x(2)), 1.35) * (m_x_dot(2) - surf_vel)),
      max_pen,
      tank_energy_threshold,
      power_limit,
      m_x_dot(0),
      m_x_dot(1),
      m_x_dot(2),
      surf_vel};
    m_data_publisher->publish(m_data_msg);
    return stiffness;
  }
  else
  {
    T_constr_min = -energy_var_damping +
                   position_error.transpose() * kd_min.asDiagonal() * velocity_error +
                   (tank_energy_threshold - tank_energy) / m_deltaT;
    T_dot_min = -energy_var_damping +
                position_error.transpose() * kd_min.asDiagonal() * velocity_error - power_limit;
  }

  real_t A[5 * 3] = {x_d(0) - x(0),
                     0,
                     0,
                     0,
                     x_d(1) - x(1),
                     0,
                     0,
                     0,
                     x_d(2) - x(2),
                     position_error(0) * velocity_error(0),
                     position_error(1) * velocity_error(1),
                     position_error(2) * velocity_error(2),
                     position_error(0) * velocity_error(0),
                     position_error(1) * velocity_error(1),
                     position_error(2) * velocity_error(2)};

  real_t ubA[5] = {F_max(0) - m_damping(0, 0) * velocity_error(0),
                   F_max(1) - m_damping(1, 1) * velocity_error(1),
                   F_max(2) - m_damping(2, 2) * velocity_error(2), 1e9, 1e9};

  real_t lbA[5] = {F_min(0) - m_damping(0, 0) * velocity_error(0),
                   F_min(1) - m_damping(1, 1) * velocity_error(1),
                   F_min(2) - m_damping(2, 2) * velocity_error(2), T_constr_min, T_dot_min};

  int_t ret_val;
  int_t nWSR = 10;
  Options options;
  options.printLevel = PL_NONE;
  // redeclare solver with options
  min_problem.setOptions(options);
  ret_val = getSimpleStatus(min_problem.init(H, g, A, lb, ub, lbA, ubA, nWSR));
  real_t xOpt[3];

  min_problem.getPrimalSolution(xOpt);

  if (ret_val != SUCCESSFUL_RETURN)
  {
    cout << "QP solver error: " << ret_val << endl;

    stiffness << kd_min(0), kd_min(1), kd_min(2), 50.0, 50.0, 50.0;
    tank_energy += energy_var_damping * m_deltaT;  // + (energy_var_stiff)*m_deltaT;
    // old_tank_energy = tank_energy;
    std_msgs::msg::Float64MultiArray m_data_msg;
    m_data_msg.data = {
      (current_time.nanoseconds() * 1e-9),                                      // Time
      x(0),                                                                     // x ee
      x(1),                                                                     // y ee
      x(2),                                                                     // z ee
      x_d(0),                                                                   // x_d ee
      x_d(1),                                                                   // y_d ee
      x_d(2),                                                                   // z_d ee
      kd(2) * position_error(2) + 2 * 0.707 * sqrt(kd(2)) * velocity_error(2),  // F_ext
      m_ft_sensor_wrench(0),                                                    // F_ft
      m_ft_sensor_wrench(1),                                                    // F_ft
      m_ft_sensor_wrench(2),                                                    // F_ft
      F_ref(2),                                                                 // F_ref
      tank_energy,                                                              // Tank
      (energy_var_stiff + energy_var_damping) * m_deltaT,                       // Tank_dot
      kd(0),                                                                    // Kd_x
      kd(1),                                                                    // Kd_y
      kd(2),                                                                    // Kd_z
      kd_max(2),                                                                // Kd_z max
      kd_min(2),                                                                // Kd_z min
      (z_value + 0.0025 - x(2)),                                                // penetration
      stiffness_value,                                                          // K_surf
      damping_value,                                                            // D_surf
      F_min(2),                                                                 // F_min
      (stiffness_value * pow((z_value + 0.0025 - x(2)), 1.35) -
       damping_value * pow((z_value + 0.0025 - x(2)), 1.35) * (m_x_dot(2) - surf_vel)),
      max_pen,
      tank_energy_threshold,
      power_limit,
      m_x_dot(0),
      m_x_dot(1),
      m_x_dot(2),
      surf_vel};
    m_data_publisher->publish(m_data_msg);
    return stiffness;
  }

  // cout << "ret_val: " << ret_val << endl;
  stiffness << xOpt[0], xOpt[1], xOpt[2], 50.0, 50.0, 50.0;
  kd << stiffness(0), stiffness(1), stiffness(2);

  // compute energy tank (previous + derivative of current*delta_T) -> EQUATION 16
  energy_var_stiff = position_error.transpose() * ((kd - kd_min).asDiagonal()) * velocity_error;
  tank_energy += (energy_var_stiff + energy_var_damping) * m_deltaT;

  print_index++;
  if (print_index % 21 == 0)
  {
    cout << "#########################################################" << endl;
    cout << " z_pos : " << x(2) << " | des " << x_d(2) << " | surf: " << z_value
         << " | surf vel: " << surf_vel << endl;
    cout << "EE velocity: " << velocity_error(2) << " | ik vel" << xdot(2) << endl;
    // cout<<" KD-KMIN: "<<endl<< (kd-kd_min)<<endl;
    // cout << "deltaX_ext: " << position_error(2) << "  | deltaX_dot: " << velocity_error(2) << endl;
    cout << "Kd: " << kd(0) << " " << kd(1) << " " << kd(2) << endl;
    cout << "Tank: " << tank_energy
         << " | Tank_dot: " << (energy_var_stiff + energy_var_damping) * m_deltaT
         << " |  threshold: " << T_constr_min << endl;
    cout << "F_ext: " << kd(2) * position_error(2) + 2 * 0.707 * sqrt(kd(2)) * velocity_error(2)
         << "|  F_des: " << F_ref(2) << "|  F_min: " << F_min(2)
         << " | F_ft: " << m_ft_sensor_wrench(2) << endl;
    cout << "Stiffness: " << stiffness_value << " | Damping: " << damping_value << endl;
    cout << "deltaT " << m_deltaT << endl;
    // cout<< "X: "<< x(2) <<endl;
  }

  std_msgs::msg::Float64MultiArray m_data_msg;
  m_data_msg.data = {
    (current_time.nanoseconds() * 1e-9),                                      // Time
    x(0),                                                                     // x ee
    x(1),                                                                     // y ee
    x(2),                                                                     // z ee
    x_d(0),                                                                   // x_d ee
    x_d(1),                                                                   // y_d ee
    x_d(2),                                                                   // z_d ee
    kd(2) * position_error(2) + 2 * 0.707 * sqrt(kd(2)) * velocity_error(2),  // F_ext
    m_ft_sensor_wrench(0),                                                    // F_ft
    m_ft_sensor_wrench(1),                                                    // F_ft
    m_ft_sensor_wrench(2),                                                    // F_ft
    F_ref(2),                                                                 // F_ref
    tank_energy,                                                              // Tank
    (energy_var_stiff + energy_var_damping) * m_deltaT,                       // Tank_dot
    kd(0),                                                                    // Kd_x
    kd(1),                                                                    // Kd_y
    kd(2),                                                                    // Kd_z
    kd_max(2),                                                                // Kd_z max
    kd_min(2),                                                                // Kd_z min
    (z_value + 0.0025 - x(2)),                                                // penetration
    stiffness_value,                                                          // K_surf
    damping_value,                                                            // D_surf
    F_min(2),                                                                 // F_min
    (stiffness_value * pow((z_value + 0.0025 - x(2)), 1.35) -
     damping_value * pow((z_value + 0.0025 - x(2)), 1.35) * (m_x_dot(2) - surf_vel)),
    max_pen,
    tank_energy_threshold,
    power_limit,
    m_x_dot(0),
    m_x_dot(1),
    m_x_dot(2),
    surf_vel};
  m_data_publisher->publish(m_data_msg);

  //old_tank_energy = tank_energy;
  return stiffness;
}

void CartesianAdaptiveComplianceController::getEndEffectorPoseReal()
{
  KDL::JntArray positions(Base::m_joint_state_pos_handles.size());
  KDL::JntArray velocities(Base::m_joint_state_vel_handles.size());
  for (size_t i = 0; i < Base::m_joint_state_pos_handles.size(); ++i)
  {
    positions(i) = Base::m_joint_state_pos_handles[i].get().get_value();
    velocities(i) = Base::m_joint_state_vel_handles[i].get().get_value();
  }

  KDL::JntArrayVel joint_data(positions, velocities);
  KDL::FrameVel tmp;
  m_fk_solver->JntToCart(joint_data, tmp);

  m_x(0) = tmp.p.p.x();
  m_x(1) = tmp.p.p.y();
  m_x(2) = tmp.p.p.z();

  m_x_dot(0) = tmp.p.v.x();
  m_x_dot(1) = tmp.p.v.y();
  m_x_dot(2) = tmp.p.v.z();
}
}  // namespace cartesian_adaptive_compliance_controller
// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  cartesian_adaptive_compliance_controller::CartesianAdaptiveComplianceController,
  controller_interface::ControllerInterface)