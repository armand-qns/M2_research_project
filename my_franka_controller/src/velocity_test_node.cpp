#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// Pinocchio headers
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

class CartesianVelocityNode : public rclcpp::Node
{
public:
    CartesianVelocityNode() : Node("cartesian_velocity_node")
    {
        const std::string urdf_filename = "/home/user/ros2_ws/src/my_franka_controller/urdf/fr3.urdf"; 
        pinocchio::urdf::buildModel(urdf_filename, model_);
        data_ = pinocchio::Data(model_);
        this->declare_parameter("target_velocity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&CartesianVelocityNode::topic_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10);
            
        RCLCPP_INFO(this->get_logger(), "Cartesian Velocity Controler initialised with Pinocchio !");
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            // 1. Récupérer la configuration articulaire actuelle
            Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq); // model_np renvoie au nombre de paramètres de configuration (9 pour fr3)

            // 2. Remplir le vecteur q avec les données du topic /joint_states en s'assurant de ne pas dépasser la taille
            int joints_to_read = std::min((int)msg->position.size(), (int)model_.nq);
            for(int i = 0; i < joints_to_read; i++) {
                q[i] = msg->position[i];
            }

            // 3. Calculer la Jacobienne complète (Taille 6x9)
            // Pinocchio va remplir J_ qui fait maintenant 6x9
            pinocchio::computeJointJacobians(model_, data_, q);
            pinocchio::getJointJacobian(model_, data_, 7, pinocchio::LOCAL_WORLD_ALIGNED, J_); 
            // Note: L'index 7 correspond normalement au link de l'effecteur dans la liste des frames Pinocchio. (si ne fonctionne pas, vérifier avec model_.getFrameId("nom_du_link")

            // 4. Extraire uniquement la partie Bras (les 7 premières colonnes)
            // On prend les 6 lignes (X,Y,Z,Rx,Ry,Rz) et les 7 premières colonnes (Joints 1 à 7)
            Eigen::MatrixXd J_arm = J_.block(0, 0, 6, 7);

            // 5. Calculer la Pseudo-Inverse sur la partie Bras uniquement
            Eigen::VectorXd v_cart_des(6);
            //v_cart_des << 0.05, 0.0, 0.0, 0.0, 0.0, 0.0; // Avancer doucement en X (5cm/s)

            std::vector<double> velocity_params;
            this->get_parameter("target_velocity", velocity_params);

            // On vérifie qu'on a bien 6 valeurs pour éviter un crash, sinon on met 0
            if (velocity_params.size() == 6) {
                for(int k=0; k<6; k++) v_cart_des[k] = velocity_params[k];
            } else {
                v_cart_des.setZero();
                RCLCPP_WARN(this->get_logger(), "Le paramètre target_velocity doit contenir exactement 6 valeurs !");
            }
            
            // Pseudo-inverse (Damped Least Squares ou SVD)
            Eigen::MatrixXd J_pinv = J_arm.completeOrthogonalDecomposition().pseudoInverse();

            // 6. Calculer la commande articulaire (taille 7)
            Eigen::VectorXd q_dot = J_pinv * v_cart_des;

            // 7. Publier la commande (Le contrôleur ROS attend 7 valeurs)
            auto command_msg = std_msgs::msg::Float64MultiArray();
            command_msg.data.resize(7);
            for(int i=0; i<7; i++) {
                // Sécurité : si on demande une vitesse trop folle, on clippe
                double v_lim = 1.0; 
                if(q_dot[i] > v_lim) q_dot[i] = v_lim;
                if(q_dot[i] < -v_lim) q_dot[i] = -v_lim;
                
                command_msg.data[i] = q_dot[i];
            }
            
            publisher_->publish(command_msg);
        }
    // Pinocchio variables
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::MatrixXd J_{6, 9}; // Jacobienne 6 lignes (cartésien), 9 colonnes (joints)
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianVelocityNode>());
    rclcpp::shutdown();
    return 0;
}