#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <realtime_tools/realtime_buffer.hpp>

// Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <std_msgs/msg/string.hpp>

using CallbackReturn = controller_interface::CallbackReturn;

namespace my_cartesian_velocity_controller {

class MyCartesianVelocityController : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Pinocchio
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd q_;      // Position articulaire  actuelle
    Eigen::VectorXd v_des_;  // Vitesse cartésienne désirée (6dimensions)
    Eigen::MatrixXd J_;      //  Jacobienne
    Eigen::VectorXd q_init_;   // Position articulaire initiale
    bool model_loaded_ = false;
    std::string urdf_string_;
    double startup_weight_ = 0.0;
    bool is_first_update_ = true;

    // Communication Temps Réel (Commandesreçues via Topic)
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_command_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> input_cmd_;
    
    int num_joints_ = 7;
};

} // namespace