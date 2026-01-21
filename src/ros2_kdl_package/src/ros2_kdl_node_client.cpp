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

    // Stati della missione
    enum class State { SEARCHING, CANCELLING, DESCENDING, ATTACHING, TO_TABLE, DETACHING, LIFT_AFTER_DROP, FINISHED };

    PickPlaceClient() : Node("pick_place_client"), current_state_(State::SEARCHING) {
        
        // Action Client
        action_client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/iiwa/ExecuteTrajectory");
        
        // Vision Subscriber
        aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&PickPlaceClient::aruco_callback, this, std::placeholders::_1));

        // Gripper Publishers
        attach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/attach", 10);
        detach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/detach", 10);

        RCLCPP_INFO(this->get_logger(), "--- Waiting for the rover... ---");
        
        // Timer per avvio ritardato
        timer_ = this->create_wall_timer(500ms, std::bind(&PickPlaceClient::start_search, this));
    }

private:
    // 1. Cerca (Guarda giù)
    std::vector<double> joints_search_ = {-0.4, -0.5, 0.0, 1.5, 0.0, -1.0, 0.0};

    // 2. Prendi (Basso sul rover)
    std::vector<double> joints_pick_ = {-0.4, -0.9, 0.0, 1.5, 0.0, -0.5, 0.0};

    // 3. Tavolo (Deposito)
    std::vector<double> joints_table_ = {0.2, 0.7, 0.0, -0.7, 0.0, 1.8, 0.0};

    // 4. Salita di sicurezza
    std::vector<double> joints_safe_lift_ = {0.2, 0.0, 0.0, -1.3, 0.0, 0.7, 0.0}; 


    void start_search() {
        timer_->cancel(); 
        RCLCPP_INFO(this->get_logger(), "Waiting for the package...");
        send_joint_goal(joints_search_);
    }

    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (current_state_ == State::SEARCHING) {
            
            // Filtro distanza: attiva solo se il tag è sotto 1.0m di altezza
            if (msg->pose.position.z > 1.0) {
                return; 
            }

            RCLCPP_WARN(this->get_logger(), "ROVER DETECTED! Starting procedure...");
            
            current_state_ = State::CANCELLING;
            action_client_->async_cancel_all_goals();
        }
    }

    void result_callback(const GoalHandle::WrappedResult & result) {
        
        // CASO 1: Transizione da RICERCA a DISCESA (scatenata dal cancel)
        if (current_state_ == State::CANCELLING) {
            RCLCPP_INFO(this->get_logger(), "Stop confirmed. Going down...");
            current_state_ = State::DESCENDING;
            send_joint_goal(joints_pick_);
        }
        
        // CASO 2: Successo del movimento precedente
        else if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            
            if (current_state_ == State::DESCENDING) {
                RCLCPP_INFO(this->get_logger(), "On the package. Attach...");
                attach_pub_->publish(std_msgs::msg::Empty());
                rclcpp::sleep_for(2s); 
                
                RCLCPP_INFO(this->get_logger(), "Picked! Going to table...");
                current_state_ = State::TO_TABLE;
                send_joint_goal(joints_table_);
            }
            else if (current_state_ == State::TO_TABLE) {
                RCLCPP_INFO(this->get_logger(), "On the table. Stabilizing..");
                rclcpp::sleep_for(2s); // Pausa per smorzare oscillazioni
                
                RCLCPP_INFO(this->get_logger(), "Detach...");
                current_state_ = State::DETACHING;
                detach_pub_->publish(std_msgs::msg::Empty());
                rclcpp::sleep_for(1s);

                RCLCPP_INFO(this->get_logger(), "Safety rising...");
                current_state_ = State::LIFT_AFTER_DROP;
                send_joint_goal(joints_safe_lift_);
            }
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

}; // <--- FINE CLASSE (Importante!)

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlaceClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
