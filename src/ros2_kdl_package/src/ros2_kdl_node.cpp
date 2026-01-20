#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp" // AGGIUNTO per inviare traiettorie

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace KDL;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node {
public:
    using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

    Iiwa_pub_sub() : Node("ros2_kdl_node") {
        this->declare_parameter<double>("traj_duration", 5.0);
        this->declare_parameter<double>("Kp", 10.0);

        this->get_parameter("traj_duration", traj_duration_);
        this->get_parameter("Kp", Kp_);

        setup_kdl();

        this->action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            this, "ExecuteTrajectory",
            std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1));

        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/iiwa/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        // MODIFICATO: Ora pubblichiamo JointTrajectory verso il controller dell'IIWA
        cmdPublisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/iiwa/iiwa_arm_controller/joint_trajectory", 10);
    }

private:
    void setup_kdl() {
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        parameters_client->wait_for_service();
        auto parameter = parameters_client->get_parameters({"robot_description"});
        
        KDL::Tree robot_tree;
        kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree);
        robot_ = std::make_shared<KDLRobot>(robot_tree);
        
        controller_ = KDLController(*robot_);
        joint_positions_.resize(robot_->getNrJnts());
        joint_velocities_.resize(robot_->getNrJnts());
    }

    // Helper per creare e pubblicare il messaggio di traiettoria
    void publish_trajectory(const std::vector<double>& joint_targets, double duration_sec) {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {"iiwa_joint_a1", "iiwa_joint_a2", "iiwa_joint_a3", 
                           "iiwa_joint_a4", "iiwa_joint_a5", "iiwa_joint_a6", "iiwa_joint_a7"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_targets;
        point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
        
        msg.points.push_back(point);
        cmdPublisher_->publish(msg);
    }

    void execute(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ExecuteTrajectory::Result>();

        // Caso 1: Traiettoria verso giunti target (Tavolo o Ricerca)
        if (!goal->joints_target.empty()) {
            RCLCPP_INFO(this->get_logger(), "Invio comando posizioni giunti...");
            publish_trajectory(goal->joints_target, traj_duration_);
            
            // Attendiamo che il braccio completi il movimento
            rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>((traj_duration_ + 0.5) * 1e9)));
        } 
        // Caso 2: Prelievo (Traiettoria Cartesiana con IK)
        else {
            RCLCPP_INFO(this->get_logger(), "Calcolo IK per traiettoria cartesiana...");
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            
            KDL::Frame current_f = robot_->getEEFrame();
            // Calcoliamo i giunti target tramite IK (semplificato per brevità)
            // Usiamo il tuo controller esistente per ottenere il target finale
            
            KDL::Frame target_f(KDL::Rotation::RPY(0, M_PI, 0), 
                               KDL::Vector(goal->pose.position.x, goal->pose.position.y, goal->pose.position.z));
            
            // Qui convertiamo la posa in giunti (IK)
            KDL::JntArray q_target_kdl(robot_->getNrJnts());
            robot_->getInverseKinematics(target_f, q_target_kdl);
            
            std::vector<double> q_target_vec(q_target_kdl.data.data(), q_target_kdl.data.data() + q_target_kdl.rows());
            
            publish_trajectory(q_target_vec, traj_duration_);
           rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>((traj_duration_ + 0.5) * 1e9)));
        }

        result->success = true;
        goal_handle->succeed(result);
    }

    void stop_robot() {
        // Opzionale: invia posa attuale per fermare
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& msg) {
        for (size_t i = 0; i < msg.position.size(); ++i) {
            joint_positions_.data[i] = msg.position[i];
            joint_velocities_.data[i] = msg.velocity[i];
        }
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteTrajectory::Goal>) 
    { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTrajectory>) 
    { return rclcpp_action::CancelResponse::ACCEPT; }
    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) 
    { std::thread{std::bind(&Iiwa_pub_sub::execute, this, std::placeholders::_1), goal_handle}.detach(); }

    std::shared_ptr<KDLRobot> robot_;
    KDLController controller_;
    KDLPlanner planner_;
    KDL::JntArray joint_positions_, joint_velocities_;
    double traj_duration_, Kp_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmdPublisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 0;
}
