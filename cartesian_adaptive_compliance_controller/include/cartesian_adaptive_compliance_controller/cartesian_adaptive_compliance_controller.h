#ifndef CARTESIAN_COMPLIANCE_CONTROLLER_H_INCLUDED
#define CARTESIAN_COMPLIANCE_CONTROLLER_H_INCLUDED

#include <cartesian_controller_base/ROS2VersionConfig.h>
#include <cartesian_controller_base/cartesian_controller_base.h>
#include <cartesian_force_controller/cartesian_force_controller.h>
#include <cartesian_motion_controller/cartesian_motion_controller.h>
#include <controller_interface/controller_interface.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <cartesian_adaptive_compliance_controller/qpOASES.hpp>
#include <cartesian_adaptive_compliance_controller/data_reader.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <queue>

USING_NAMESPACE_QPOASES
namespace cartesian_adaptive_compliance_controller
{

class CartesianAdaptiveComplianceController
: public cartesian_motion_controller::CartesianMotionController
, public cartesian_force_controller::CartesianForceController
{
  public:
    CartesianAdaptiveComplianceController();

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE
    virtual LifecycleNodeInterface::CallbackReturn on_init() override;
#elif defined CARTESIAN_CONTROLLERS_FOXY
    virtual controller_interface::return_type init(const std::string & controller_name) override;
#endif

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
#elif defined CARTESIAN_CONTROLLERS_FOXY
    controller_interface::return_type update() override;
#endif

    using Base = cartesian_controller_base::CartesianControllerBase;
    using MotionBase = cartesian_motion_controller::CartesianMotionController;
    using ForceBase = cartesian_force_controller::CartesianForceController;

  private:
    /**
     * @brief Compute the net force of target wrench and stiffness-related pose offset
     *
     * @return The remaining error wrench, given in robot base frame
     */
    void getEndEffectorPoseReal();
    
    ctrl::Vector6D          computeStiffness();
    ctrl::Vector6D          computeComplianceError();
    std::shared_ptr<
      KDL::ChainFkSolverVel_recursive>  m_fk_solver;

    ctrl::Matrix6D          m_stiffness;
    ctrl::Matrix6D          m_damping;
    std::string             m_compliance_ref_link;

    const ctrl::Vector3D    Q = {3200,3200,3200};
    const ctrl::Vector3D    R = {0.00001, 0.00001, 0.00001};
    ctrl::Vector3D          kd = {0,0,0};
    ctrl::Vector6D          stiffness = {0,0,0,0,0,0};
    ctrl::Vector3D          kd_max = {1000, 1000, 1000};
    ctrl::Vector3D          kd_min = {300, 300, 100};
    ctrl::Vector3D          F_max = {15, 15, 15};
    ctrl::Vector3D          F_min = {-15, -15, -15};
    ctrl::Vector3D          x_d_old = {0,0,0};
    size_t                  m_window_length;
    ctrl::Vector6D          m_prev_error;
    double                  m_deltaT;
    double                  m_sigma;

    ctrl::Vector3D          m_x;
    ctrl::Vector3D          m_x_dot;

    std::vector<ctrl::Vector3D>   m_external_forces;
    std::vector<ctrl::Vector3D>   m_desired_forces;
    std::vector<ctrl::Matrix3D>   m_desired_stiffness;
    
    // Tank
    double Xt;
    double dXt;
    double kl_ = 100;
    double dl_ = 20;
    double tank_energy;
    double tank_energy_threshold;
    double d_pass_const_stiff, d_pass_damp, d_pass_en;
    double d_pass_damp_int, en_var_stiff_int;
    double energy_var_stiff, energy_var_damping;
    rclcpp::Time old_time,current_time,start_time;
    double old_z;
    std::queue<double> m_surf_vel;
    double m_surf_vel_sum;

    QProblem min_problem;
    int print_index = 0;

    // ft sensor subscriber
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_ft_sensor_wrench_subscriber;
    void ftSensorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
    ctrl::Vector3D m_ft_sensor_wrench;

    // data publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  m_data_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  m_target_pose_publisher;
    void publishTargetFrame();
    int step_seconds = 20;
    double z_step = 0.05;
    ctrl::Vector3D m_starting_pose;

    // data from the data reader
    std::vector<double> m_x_coordinates;
    std::vector<double> m_y_coordinates;
    std::vector<std::vector<double>> m_z_values;
    std::vector<std::vector<double>> m_stiffness_values;
    std::vector<std::vector<double>> m_damping_values;


};

}

#endif
