#include "my_cartesian_impedance_controller/my_cartesian_impedance_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp> // Pour la gravité 
#include <rclcpp/rclcpp.hpp>

namespace my_cartesian_impedance_controller {

controller_interface::InterfaceConfiguration MyCartesianImpedanceController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // commande  en EFFORT (Couple)
    for (int i = 1; i <= num_joints_; ++i) config.names.push_back("fr3_joint" + std::to_string(i) + "/effort");
    return config;
}

controller_interface::InterfaceConfiguration MyCartesianImpedanceController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // On a besoin de POSITION et VITESSE pour l'impédance (terme d'amortissement)
    for (int i = 1; i <= num_joints_; ++i) {
        config.names.push_back("fr3_joint" + std::to_string(i) + "/position");
        config.names.push_back("fr3_joint" + std::to_string(i) + "/velocity");
    }
    return config;
}

CallbackReturn MyCartesianImpedanceController::on_init() {
    return CallbackReturn::SUCCESS;
}

CallbackReturn MyCartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto temp_node = std::make_shared<rclcpp::Node>("temp_urdf_loader_node_imp");
    RCLCPP_INFO(get_node()->get_logger(), "Waiting for robot_description...");
    
    std::string temp_urdf;
    bool received = false;
    auto qos = rclcpp::QoS(1).transient_local();
    auto sub = temp_node->create_subscription<std_msgs::msg::String>(
        "/robot_description", qos,
        [&temp_urdf, &received](const std_msgs::msg::String::SharedPtr msg) {
            temp_urdf = msg->data;
            received = true;
        });

    rclcpp::Time start_time = temp_node->now();
    while (!received) {
        rclcpp::spin_some(temp_node); 
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        if ((temp_node->now() - start_time).seconds() > 5.0) {
            RCLCPP_ERROR(get_node()->get_logger(), "Timeout URDF.");
            return controller_interface::CallbackReturn::ERROR;
        }
    }
    
    // Construction Pinocchio
    try {
        pinocchio::urdf::buildModelFromXML(temp_urdf, model_);
        data_ = pinocchio::Data(model_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "URDF Parsing error: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // Récupération de l'ID de la frame du bout du bras (ex: fr3_link8 ou flang)
    if (model_.existFrame("fr3_link8")) {
        ee_frame_id_ = model_.getFrameId("fr3_link8");
    } else {
        ee_frame_id_ = model_.nframes - 1; // Fallback au dernier
        RCLCPP_WARN(get_node()->get_logger(), "Frame 'fr3_link8' not found, using frame ID %d", ee_frame_id_);
    }

    // Init Vecteurs
    q_ = Eigen::VectorXd::Zero(model_.nq);
    v_ = Eigen::VectorXd::Zero(model_.nv);
    tau_J_ = Eigen::VectorXd::Zero(model_.nv);
    J_ = Eigen::MatrixXd::Zero(6, model_.nv);
    
    // --- TUNING IMPÉDANCE (A ajuster selon le robot) ---
    Kp_ = Eigen::VectorXd::Zero(6);
    Kd_ = Eigen::VectorXd::Zero(6);
    
    // Raideur Translation (N/m) et Rotation (Nm/rad)
    Kp_ << 500, 500, 500, 30, 30, 30; 
    // Amortissement 
    Kd_ << 40, 40, 40, 5, 5, 5;

    // Subscriber Pose cible
    auto node = get_node();
    sub_target_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            input_target_.writeFromNonRT(msg);
        });

    return CallbackReturn::SUCCESS;
}

CallbackReturn MyCartesianImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // 1. Lire état initial pour éviter un saut brutal
    for (int i = 0; i < num_joints_; ++i) {
        q_[i] = state_interfaces_[2 * i].get_value();     // Position
        // v_[i] ignoré pour l'initialisation de pose
    }

    // 2. Calculer FK
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    
    // 3. Définir la cible initiale sur la position actuelle
    M_des_ = data_.oMf[ee_frame_id_]; 
    
    // Reset buffer
    input_target_.reset();
    
    RCLCPP_INFO(get_node()->get_logger(), "Impedance Controller Activated. Holding current pose.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn MyCartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type MyCartesianImpedanceController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

    // 1. Lecture de l'état (q, v)
    // [pos_1, vel_1, pos_2, vel_2, ...] 
    for (int i = 0; i < num_joints_; ++i) {
        q_[i] = state_interfaces_[2 * i].get_value();
        v_[i] = state_interfaces_[2 * i + 1].get_value();
    }

    // 2. Mise à jour Cible (si nouveau message)
    auto current_target_msg = input_target_.readFromRT();
    if (current_target_msg && *current_target_msg) {
        auto& msg = *current_target_msg;
        // Conversion Msg -> Pinocchio SE3
        Eigen::Quaterniond quat(msg->pose.orientation.w, msg->pose.orientation.x, 
                                msg->pose.orientation.y, msg->pose.orientation.z);
        Eigen::Vector3d trans(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        
        M_des_ = pinocchio::SE3(quat, trans);
    }

    // 3. Pinocchio Calculations
    // Forward Kinematics & Jacobians
    pinocchio::forwardKinematics(model_, data_, q_, v_); 
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::computeJointJacobians(model_, data_, q_);
    
    // Gravity Compensation Vector (g(q))
    pinocchio::computeGeneralizedGravity(model_, data_, q_);

    // Récupération de la Jacobienne 
    pinocchio::getFrameJacobian(model_, data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_);

    // Pose actuelle de l'effecteur
    pinocchio::SE3 M_curr = data_.oMf[ee_frame_id_];

    // Vitesse cartésienne actuelle (Twist) en World frame
    Eigen::VectorXd v_curr_cart = J_ * v_;

    // 4. Calcul de l'erreur (Loi de contrôle)
    Eigen::Vector3d err_pos = M_des_.translation() - M_curr.translation();

    // Erreur d'Orientation (Log3 de R_curr^T * R_des)
    Eigen::Matrix3d R_err = M_des_.rotation() * M_curr.rotation().transpose(); 
    Eigen::Vector3d err_rot = pinocchio::log3(R_err);

    Eigen::VectorXd error_6d(6);
    error_6d << err_pos, err_rot;

    // 5. Calcul des Forces/Couples Cartésiens désirés
    Eigen::VectorXd F_des(6);
    F_des = Kp_.cwiseProduct(error_6d) - Kd_.cwiseProduct(v_curr_cart);

    // 6. Mapping Espace Cartésien -> Espace Joint (Couple)
    tau_J_ = J_.transpose() * F_des;

    // 7. Ajout de la Compensation de Gravité 
    tau_J_ += data_.g; 

    // 8.Envoi aux moteurs (avec sécu)
    for (int i = 0; i < num_joints_; ++i) {
        double tau_lim = 80.0; // Limite arbitraire (vérifier )
        double cmd = std::max(std::min(tau_J_[i], tau_lim), -tau_lim);
        command_interfaces_[i].set_value(cmd);
    }

    return controller_interface::return_type::OK;
}

} 

PLUGINLIB_EXPORT_CLASS(my_cartesian_impedance_controller::MyCartesianImpedanceController, controller_interface::ControllerInterface)