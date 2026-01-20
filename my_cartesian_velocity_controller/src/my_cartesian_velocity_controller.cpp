#include "my_cartesian_velocity_controller/my_cartesian_velocity_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_cartesian_velocity_controller {

controller_interface::InterfaceConfiguration MyCartesianVelocityController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints_; ++i) config.names.push_back("fr3_joint" + std::to_string(i) + "/velocity");
    return config;
}

controller_interface::InterfaceConfiguration MyCartesianVelocityController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints_; ++i) config.names.push_back("fr3_joint" + std::to_string(i) + "/position");
    return config;
}


CallbackReturn MyCartesianVelocityController::on_init() {
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyCartesianVelocityController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    
    // 1. Créer un nœud temporaire, juste pour écouter le topic
    auto temp_node = std::make_shared<rclcpp::Node>("temp_urdf_loader_node");

    RCLCPP_INFO(get_node()->get_logger(), "Création d'un nœud temporaire pour récupérer l'URDF...");

    std::string temp_urdf;
    bool received = false;

    // 2. S'abonner via ce nœud temporaire
    auto qos = rclcpp::QoS(1).transient_local();
    auto sub = temp_node->create_subscription<std_msgs::msg::String>(
        "/robot_description", qos,
        [&temp_urdf, &received](const std_msgs::msg::String::SharedPtr msg) {
            temp_urdf = msg->data;
            received = true;
        });

    // 3. Boucle d'attente sécurisée
    rclcpp::Time start_time = temp_node->now();
    while (!received) {
        rclcpp::spin_some(temp_node); 
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        
        if ((temp_node->now() - start_time).seconds() > 5.0) {
            RCLCPP_ERROR(get_node()->get_logger(), "Timeout : /robot_description non reçu après 5s.");
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(get_node()->get_logger(), "URDF reçu ! Construction du modèle Pinocchio...");

    // 4. Construction Pinocchio
    try {
        pinocchio::urdf::buildModelFromXML(temp_urdf, model_);
        data_ = pinocchio::Data(model_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Erreur parsing URDF : %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // 5. Initialisation des vecteurs
    q_ = Eigen::VectorXd::Zero(model_.nq);
    v_des_ = Eigen::VectorXd::Zero(6);
    J_ = Eigen::MatrixXd::Zero(6, model_.nv);

    // 6. Setup du VRAI Subscriber pour les commandes (vrai node)
    auto node = get_node();
    sub_command_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            input_cmd_.writeFromNonRT(msg);
        });

    return controller_interface::CallbackReturn::SUCCESS;
}

CallbackReturn MyCartesianVelocityController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Reset de la commande à 0 au démarrage
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = 0.0;
    msg->linear.y = 0.0;
    msg->linear.z = 0.0;

    msg->angular.x = 0.0;
    msg->angular.y = 0.0;
    msg->angular.z = 0.0;
    input_cmd_.writeFromNonRT(msg);
    q_init_ = q_;
    return CallbackReturn::SUCCESS;
}

CallbackReturn MyCartesianVelocityController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

// LA BOUCLE TEMPS RÉEL (1000 Hz)
controller_interface::return_type MyCartesianVelocityController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

    // 1. Lire l'état des moteurs 
    for (int i = 0; i < num_joints_; ++i) {
        q_[i] = state_interfaces_[i].get_value();
    }

    // 2. Lire la dernière commande utilisateur reçue
    auto current_cmd = input_cmd_.readFromRT();
    if (current_cmd && *current_cmd) {
        v_des_ << (*current_cmd)->linear.x, (*current_cmd)->linear.y, (*current_cmd)->linear.z,
                  (*current_cmd)->angular.x, (*current_cmd)->angular.y, (*current_cmd)->angular.z;
    }

    // 3. Calcul Pinocchio (Jacobienne)
    pinocchio::computeJointJacobians(model_, data_, q_);
    pinocchio::getJointJacobian(model_, data_, 8, pinocchio::LOCAL_WORLD_ALIGNED, J_);

    // 4. Isolation partie Bras (6x7)
    Eigen::MatrixXd J_arm = J_.block(0, 0, 6, 7);

    // 5. Pseudo-Inverse 
    double lambda = 0.05; 
    Eigen::MatrixXd J_pinv = J_arm.transpose() * (J_arm * J_arm.transpose() + lambda*lambda * Eigen::MatrixXd::Identity(6,6)).inverse();

    // 5bis. Projecteur de Nullspace pour maintenir la position initiale
    Eigen::MatrixXd Identity = Eigen::MatrixXd::Identity(7, 7);
    Eigen::MatrixXd Nullspace_Projector = Identity - (J_pinv * J_arm);
    double k_null = 1.0; 
    Eigen::VectorXd q_arm = q_.head(7); 
    Eigen::VectorXd q_init_arm = q_init_.head(7);

    Eigen::VectorXd q_dot_null = -k_null * (q_arm - q_init_arm);

    // 6. Calcul q_dot 
    Eigen::VectorXd q_dot = (J_pinv * v_des_) + (Nullspace_Projector * q_dot_null);

    // 7. Écrire les commandes (Directement dans la mémoire)
    for (int i = 0; i < num_joints_; ++i) {
        //sécurité 
        double v_lim = 0.1; 
        q_dot[i] = std::max(std::min(q_dot[i], v_lim), -v_lim);
        
        command_interfaces_[i].set_value(q_dot[i]);
    }

    return controller_interface::return_type::OK;
}

} // namespace

// Export du plugin
PLUGINLIB_EXPORT_CLASS(my_cartesian_velocity_controller::MyCartesianVelocityController, controller_interface::ControllerInterface)