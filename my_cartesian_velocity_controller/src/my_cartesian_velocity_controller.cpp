#include "my_cartesian_velocity_controller/my_cartesian_velocity_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

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
    
    RCLCPP_INFO(get_node()->get_logger(), "Configuration : Chargement de l'URDF depuis le fichier...");

    try {
        // On demande à ROS où est installé ton package
        std::string share_dir = ament_index_cpp::get_package_share_directory("my_cartesian_velocity_controller");
        
        // On reconstruit le chemin complet vers le fichier fr3.urdf
        std::string urdf_file_path = share_dir + "/urdf/fr3.urdf";
        
        RCLCPP_INFO(get_node()->get_logger(), "Chemin URDF cible : %s", urdf_file_path.c_str());

        // sécurité
        if (!std::filesystem::exists(urdf_file_path)) {
            RCLCPP_ERROR(get_node()->get_logger(), "ERREUR FATALE : Fichier introuvable à : %s", urdf_file_path.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        // Construction Pinocchio DIRECTE depuis le fichier
        pinocchio::urdf::buildModel(urdf_file_path, model_);
        data_ = pinocchio::Data(model_);

        RCLCPP_INFO(get_node()->get_logger(), "Succès : Modèle chargé ! Joints: %d", model_.nq);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "EXCEPTION lors du chargement URDF : %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // 5. Initialisation des vecteurs
    q_ = Eigen::VectorXd::Zero(model_.nq);
    v_des_ = Eigen::VectorXd::Zero(6);
    J_ = Eigen::MatrixXd::Zero(6, model_.nv);

    // 6. Setup du VRAI Subscriber pour les commandes
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
    msg->linear.x = 0.0; msg->linear.y = 0.0; msg->linear.z = 0.0;
    msg->angular.x = 0.0; msg->angular.y = 0.0; msg->angular.z = 0.0;
    input_cmd_.writeFromNonRT(msg);
    
    // --- CORRECTION CRITIQUE ICI ---
    // On doit ABSOLUMENT lire la position actuelle du robot avant de l'enregistrer
    for (int i = 0; i < num_joints_; ++i) {
        if (std::isnan(state_interfaces_[i].get_value())) {
            RCLCPP_ERROR(get_node()->get_logger(), "Erreur: La position du joint %d est NaN !", i);
            return CallbackReturn::ERROR;
        }
        q_[i] = state_interfaces_[i].get_value();
    }
    
    // Maintenant q_ contient la vraie position, on peut initialiser q_init_
    q_init_ = q_;

    startup_weight_ = 0.0;
    is_first_update_ = true;
    
    RCLCPP_INFO(get_node()->get_logger(), "Contrôleur activé. Position initiale capturée. Soft start en prêt.");
    return CallbackReturn::SUCCESS;
}


CallbackReturn MyCartesianVelocityController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

// LA BOUCLE TEMPS RÉEL (1000 Hz)

controller_interface::return_type MyCartesianVelocityController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

    // ----------------------------------------------------------------
    // ETAPE 1 : "Safety Counter" (On ne fait RIEN pendant 3000 cycles)
    // ----------------------------------------------------------------
    // On utilise une variable statique pour compter les cycles sans modifier le .hpp

    static int safety_counter = 0;
    if (is_first_update_) {
        safety_counter = 0;
        is_first_update_ = false;
    }
    // Tant que les 3 premières secondes ne sont pas passées (3000ms)
    if (safety_counter < 3000) {
        safety_counter++;
        
        // ON ENVOIE STRICTEMENT ZERO
        for (int i = 0; i < num_joints_; ++i) {
            command_interfaces_[i].set_value(0.0);
        }
        return controller_interface::return_type::OK;
    } (Le Secret Anti-Reflex)

    // ----------------------------------------------------------------
    // ETAPE 2 : Calcul Normal (Pinocchio)
    // ----------------------------------------------------------------
    // 1. Lire positions
    for (int i = 0; i < num_joints_; ++i) q_[i] = state_interfaces_[i].get_value();

    // 2. Lire commande
    auto current_cmd = input_cmd_.readFromRT();
    if (current_cmd && *current_cmd) {
        v_des_ << (*current_cmd)->linear.x, (*current_cmd)->linear.y, (*current_cmd)->linear.z,
                  (*current_cmd)->angular.x, (*current_cmd)->angular.y, (*current_cmd)->angular.z;
    } else {
        v_des_.setZero(); // Sécurité si pas de commande
    }

    // 3. Pinocchio
    pinocchio::computeJointJacobians(model_, data_, q_);
    pinocchio::getJointJacobian(model_, data_, 8, pinocchio::LOCAL_WORLD_ALIGNED, J_);
    Eigen::MatrixXd J_arm = J_.block(0, 0, 6, 7);

    // 4. P-Inv Amortie
    double lambda = 0.1; 
    Eigen::MatrixXd J_pinv = J_arm.transpose() * (J_arm * J_arm.transpose() + lambda*lambda * Eigen::MatrixXd::Identity(6,6)).inverse();

    // 5. Calcul Vitesse Cible (Sans Nullspace pour l'instant)
    Eigen::VectorXd q_dot_target = J_pinv * v_des_;

    // ----------------------------------------------------------------
    // ETAPE 3 : SLEW RATE LIMITER
    // ----------------------------------------------------------------
    // On limite le changement de vitesse entre deux cycles.
    
    static Eigen::VectorXd q_dot_prev = Eigen::VectorXd::Zero(num_joints_);
    
    // Limite max de changement de vitesse par cycle (très faible !)
    // 0.0001 rad/s par ms = 0.1 rad/s² max accel
    double max_change = 0.0001; 

    Eigen::VectorXd q_dot_safe = q_dot_target;

    for (int i = 0; i < num_joints_; ++i) {
        // On calcule la différence voulue
        double diff = q_dot_target[i] - q_dot_prev[i];

        // On clippe la différence
        if (diff > max_change) diff = max_change;
        if (diff < -max_change) diff = -max_change;

        // La nouvelle commande est l'ancienne + la différence limitée
        q_dot_safe[i] = q_dot_prev[i] + diff;
        
        // Mise à jour mémoire
        q_dot_prev[i] = q_dot_safe[i];
    }

    // ----------------------------------------------------------------
    // ETAPE 4 : Envoi
    // ----------------------------------------------------------------
    for (int i = 0; i < num_joints_; ++i) {
        // Double sécurité : Vitesse Max absolue
        double v_max_abs = 0.5;
        q_dot_safe[i] = std::max(std::min(q_dot_safe[i], v_max_abs), -v_max_abs);
        
        command_interfaces_[i].set_value(q_dot_safe[i]);
    }

    return controller_interface::return_type::OK;
}

} // namespace

// Export du plugin
PLUGINLIB_EXPORT_CLASS(my_cartesian_velocity_controller::MyCartesianVelocityController, controller_interface::ControllerInterface)