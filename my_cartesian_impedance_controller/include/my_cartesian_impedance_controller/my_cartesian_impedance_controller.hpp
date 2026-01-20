#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // Changement Twist -> Pose
#include <realtime_tools/realtime_buffer.hpp>

// Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <std_msgs/msg/string.hpp>

using CallbackReturn = controller_interface::CallbackReturn;

namespace my_cartesian_impedance_controller {

class MyCartesianImpedanceController : public controller_interface::ControllerInterface {
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
    
    Eigen::VectorXd q_;      // Position articulaire
    Eigen::VectorXd v_;      // Vitesse articulaire (nécessaire pour l'amortissement)
    Eigen::VectorXd tau_J_;  // Couple calculé
    Eigen::MatrixXd J_;      // Jacobienne

    // Cible cartésienne
    pinocchio::SE3 M_des_;   // Pose désirée (Position + Orientation)
    
    // Gains d'impédance
    Eigen::VectorXd Kp_;     // Raideur (Stiffness) 6D
    Eigen::VectorXd Kd_;     // Amortissement (Damping) 6D

    bool model_loaded_ = false;
    std::string urdf_string_;
    int ee_frame_id_ = -1; // ID de la frame du bout du bras (Link 8/EE)

    // Communication Temps Réel
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::PoseStamped>> input_target_;
    
    int num_joints_ = 7;
};

} // namespace