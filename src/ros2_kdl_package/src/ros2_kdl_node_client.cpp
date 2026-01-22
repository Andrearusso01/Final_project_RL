#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace std::chrono_literals;

class PickPlaceClient : public rclcpp::Node {
public:
    using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

    enum class State { SEARCHING, CANCELLING, DESCENDING, ATTACHING, LIFTING_FROM_CART, ROTATING_TO_DROP, DETACHING, LIFT_AFTER_DROP, FINISHED };

    PickPlaceClient() : Node("pick_place_client"), current_state_(State::SEARCHING) {
        
        action_client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/iiwa/ExecuteTrajectory");
        
        aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&PickPlaceClient::aruco_callback, this, std::placeholders::_1));

        attach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/attach", 10);
        detach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/detach", 10);

        RCLCPP_INFO(this->get_logger(), "--- Waiting for the rover... ---");
        
        timer_ = this->create_wall_timer(500ms, std::bind(&PickPlaceClient::start_search, this));
    }

private:
    // --- COORDINATE ---

    // 1. Cerca (Guarda giù)
    std::vector<double> joints_search_ = {-0.4, -0.5, 0.0, 1.5, 0.0, -1.0, 0.0};

    // 2. Prendi (Basso sul rover)
    std::vector<double> joints_pick_ = {-0.4, -0.9, 0.0, 1.5, 0.0, -0.5, 0.0};

    // 3. LIFT (Salita verticale dal carrello)
    //    Mantengo J1 (-0.4) uguale al pick per non ruotare, ma alzo J2 e J4
    std::vector<double> joints_lift_cart_ = {-0.4, 0.0, 0.0, 1.5, 0.0, -1.0, 0.0};

    // 4. ROTATE & DROP (La tua coordinata -3.14)
    //    Questa posizione ruota di 180 gradi e mette il robot a L per il rilascio
    std::vector<double> joints_rotate_drop_ = {-3.14, 0.0, 0.0, 1.7, 0.0, -1.4, 0.0};

    // 5. Salita di sicurezza finale
    //    Mantengo la rotazione -3.14 ma alzo il braccio
    std::vector<double> joints_safe_lift_ = {-3.14, 0.0, 0.0, 1.0, 0.0, -1.4, 0.0}; 


    void start_search() {
        timer_->cancel(); 
        RCLCPP_INFO(this->get_logger(), "Waiting for the package...");
        send_joint_goal(joints_search_);
    }

    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (current_state_ == State::SEARCHING) {
            
            if (msg->pose.position.z > 1.0) {
                return; 
            }

            RCLCPP_WARN(this->get_logger(), "ROVER DETECTED! Starting procedure...");
            current_state_ = State::CANCELLING;
            action_client_->async_cancel_all_goals();
        }
    }

    void result_callback(const GoalHandle::WrappedResult & result) {
        
        // A. Transizione INIZIALE (Start -> Pick)
        if (current_state_ == State::CANCELLING) {
            RCLCPP_INFO(this->get_logger(), "Stop confirmed. Going down to PICK...");
            current_state_ = State::DESCENDING;
            send_joint_goal(joints_pick_);
        }
        
        // B. Gestione Sequenza
        else if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            
            // FASE 2 -> 3: Presa e Salita Verticale
            if (current_state_ == State::DESCENDING) {
                RCLCPP_INFO(this->get_logger(), "On package. ATTACH.");
                attach_pub_->publish(std_msgs::msg::Empty());
                rclcpp::sleep_for(2s); 
                
                RCLCPP_INFO(this->get_logger(), "Lifting form cart (Vertical)...");
                current_state_ = State::LIFTING_FROM_CART;
                send_joint_goal(joints_lift_cart_);
            }
            
            // FASE 3 -> 4: Rotazione verso il tavolo
            else if (current_state_ == State::LIFTING_FROM_CART) {
                RCLCPP_INFO(this->get_logger(), "Clear of cart. Rotating to TABLE...");
                current_state_ = State::ROTATING_TO_DROP;
                send_joint_goal(joints_rotate_drop_);
            }

            // FASE 4 -> 5: Rilascio e Salita finale
            else if (current_state_ == State::ROTATING_TO_DROP) {
                RCLCPP_INFO(this->get_logger(), "At table position. DETACH...");
                
                // Pausa per stabilizzare l'oscillazione prima di mollare
                rclcpp::sleep_for(2s); 
                
                current_state_ = State::DETACHING;
                detach_pub_->publish(std_msgs::msg::Empty());
                rclcpp::sleep_for(1s);

                RCLCPP_INFO(this->get_logger(), "Mission done. Safe lift...");
                current_state_ = State::LIFT_AFTER_DROP;
                send_joint_goal(joints_safe_lift_);
            }

            // FINE
            else if (current_state_ == State::LIFT_AFTER_DROP) {
                RCLCPP_INFO(this->get_logger(), "--- MISSION COMPLETED ---");
                current_state_ = State::FINISHED;
            }
        }
    }

    void send_joint_goal(std::vector<double> joints) {
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Server not found!");
            return;
        }
        auto goal_msg = ExecuteTrajectory::Goal();
        goal_msg.joints_target = joints;
        
        auto opts = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
        opts.result_callback = std::bind(&PickPlaceClient::result_callback, this, std::placeholders::_1);
        action_client_->async_send_goal(goal_msg, opts);
    }

    // Variabili Membro
    State current_state_;
    rclcpp_action::Client<ExecuteTrajectory>::SharedPtr action_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlaceClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
