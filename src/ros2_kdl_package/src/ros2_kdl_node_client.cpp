#include <memory>
#include <string>
#include <vector>
#include <chrono>

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

    // Aggiunto stato STARTING per il posizionamento iniziale della camera
    enum class State { STARTING, SEARCHING, DESCENDING, ATTACHING, RETURNING, FINISHED };

    PickPlaceClient() : Node("pick_place_client"), current_state_(State::STARTING) {
        
        action_client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/iiwa/ExecuteTrajectory");

        aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&PickPlaceClient::aruco_callback, this, std::placeholders::_1));

        attach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/attach", 10);
        detach_pub_ = this->create_publisher<std_msgs::msg::Empty>("/gripper/detach", 10);

        RCLCPP_INFO(this->get_logger(), "--- COORDINATORE MISSIONE AVVIATO ---");
        
        // Avviamo immediatamente la posa di ricerca all'accensione del nodo
        // Usiamo un timer per dare tempo all'action server di connettersi
        timer_ = this->create_wall_timer(1s, std::bind(&PickPlaceClient::init_mission, this));
    }

private:
    void init_mission() {
        timer_->cancel(); // Eseguiamo solo una volta
        send_search_pose();
    }

    // --- 1. POSIZIONAMENTO CAMERA ---
    void send_search_pose() {
        RCLCPP_INFO(this->get_logger(), "Stato: STARTING - Posizionamento camera verso il basso...");
        auto goal_msg = ExecuteTrajectory::Goal();
        goal_msg.joints_target = {-0.4, -0.5, 0.0, 1.3, 0.0, -1.1, 0.0};
        send_goal(goal_msg);
    }

    // --- 2. CALLBACK RILEVAMENTO ARUCO ---
    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (current_state_ == State::SEARCHING) {
            RCLCPP_INFO(this->get_logger(), "Tag ArUco rilevato! Coordinate ricevute.");
            current_state_ = State::DESCENDING;

            auto goal_msg = ExecuteTrajectory::Goal();
            goal_msg.pose = msg->pose;
            goal_msg.pose.position.z += 0.03; // Offset sicurezza

            RCLCPP_INFO(this->get_logger(), "Avvio discesa verso il pacco...");
            send_goal(goal_msg);
        }
    }

    void send_goal(ExecuteTrajectory::Goal goal_msg) {
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Server KDL non disponibile!");
            return;
        }
        auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&PickPlaceClient::result_callback, this, std::placeholders::_1);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // --- GESTORE RISULTATI (MACCHINA A STATI) ---
    void result_callback(const GoalHandle::WrappedResult & result) {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "Movimento fallito.");
            return;
        }

        if (current_state_ == State::STARTING) {
            RCLCPP_INFO(this->get_logger(), "Stato: SEARCHING - Camera in posizione, cerco il tag...");
            current_state_ = State::SEARCHING;
        }
        else if (current_state_ == State::DESCENDING) {
            RCLCPP_INFO(this->get_logger(), "Stato: ATTACHING - Eseguo ATTACH.");
            attach_pub_->publish(std_msgs::msg::Empty());
            rclcpp::sleep_for(2s);

            RCLCPP_INFO(this->get_logger(), "Pacco agganciato. Ritorno al tavolo...");
            current_state_ = State::RETURNING;
            send_table_goal();
        } 
        else if (current_state_ == State::RETURNING) {
            RCLCPP_INFO(this->get_logger(), "Stato: FINISHED - Posa tavolo raggiunta. Eseguo DETACH.");
            detach_pub_->publish(std_msgs::msg::Empty());
            current_state_ = State::FINISHED;
            RCLCPP_INFO(this->get_logger(), "--- MISSIONE COMPLETATA ---");
        }
    }

    void send_table_goal() {
        auto goal_msg = ExecuteTrajectory::Goal();
        goal_msg.joints_target = {0.2, 0.9, 0.0, -1.3, 0.0, 0.7, 0.0};
        send_goal(goal_msg);
    }

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
