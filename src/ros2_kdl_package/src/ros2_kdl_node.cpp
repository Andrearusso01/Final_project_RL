#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex> 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

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

    Iiwa_pub_sub() : Node("ros2_kdl_node"), state_received_(false) {
        this->declare_parameter<double>("traj_duration", 5.0);
        this->declare_parameter<double>("Kp", 10.0);

        this->get_parameter("traj_duration", traj_duration_);
        this->get_parameter("Kp", Kp_);

        robot_ = nullptr;
        controller_ = nullptr;

        // 1. TENTIAMO IL SETUP KDL
        if (!setup_kdl()) {
            RCLCPP_ERROR(this->get_logger(), "CRITICO: Setup KDL fallito. Il nodo non funzionerà.");
            // Non creiamo né subscriber né server se il robot non c'è.
            return; 
        }

        // 2. SOLO SE KDL È OK, AVVIAMO IL RESTO
        this->action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            this, "ExecuteTrajectory",
            std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1));

        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/iiwa/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        cmdPublisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/iiwa/iiwa_arm_controller/joint_trajectory", 10);
            
        RCLCPP_INFO(this->get_logger(), "Server KDL avviato correttamente.");
    }

private:
    // Restituisce bool per dire se ha avuto successo
    bool setup_kdl() {
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        int retries = 0;
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) return false;
            RCLCPP_INFO(this->get_logger(), "In attesa di robot_state_publisher...");
            retries++;
            if (retries > 5) {
                RCLCPP_ERROR(this->get_logger(), "Timeout: robot_state_publisher non trovato!");
                return false;
            }
        }
        
        auto parameter = parameters_client->get_parameters({"robot_description"});
        if (parameter.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parametro robot_description vuoto!");
            return false;
        }
        
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Errore parsing URDF!");
            return false;
        }
        
        robot_ = std::make_shared<KDLRobot>(robot_tree);
        
        unsigned int nj = robot_->getNrJnts();
        if (nj == 0) {
             RCLCPP_ERROR(this->get_logger(), "Errore: Il robot ha 0 giunti!");
             return false;
        }

        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_.data.setZero();
        joint_velocities_.data.setZero();
        
        controller_ = std::make_shared<KDLController>(*robot_);
        RCLCPP_INFO(this->get_logger(), "KDL inizializzato con %d giunti.", nj);
        return true;
    }

    void publish_trajectory(const std::vector<double>& joint_targets, double duration_sec) {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.header.stamp = this->get_clock()->now();
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

        // 1. Check preliminare
        if (robot_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Robot non inizializzato! Impossibile eseguire.");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // 2. Copia dati con Mutex
        KDL::JntArray q_current(robot_->getNrJnts());
        KDL::JntArray qd_current(robot_->getNrJnts());
        
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Se non abbiamo mai ricevuto dati dal topic, è inutile calcolare
            if (!state_received_) {
                 RCLCPP_WARN(this->get_logger(), "Nessun dato giunti ricevuto! Controlla i topic.");
                 result->success = false;
                 goal_handle->abort(result);
                 return;
            }
            q_current = joint_positions_;
            qd_current = joint_velocities_;
        } 

        // 3. Esecuzione
        if (!goal->joints_target.empty()) {
            RCLCPP_INFO(this->get_logger(), "Executing Joint Trajectory...");
            publish_trajectory(goal->joints_target, traj_duration_);
        } 
        else {
            RCLCPP_INFO(this->get_logger(), "Executing Cartesian IK...");
            
            // Conversione manuale sicura
            std::vector<double> q_std(q_current.rows());
            std::vector<double> qd_std(qd_current.rows());
            for(unsigned int i=0; i<q_current.rows(); ++i) {
                q_std[i] = q_current(i);
                qd_std[i] = qd_current(i);
            }

            robot_->update(q_std, qd_std);
            
            KDL::Frame target_f(KDL::Rotation::RPY(0, M_PI, 0), 
                               KDL::Vector(goal->pose.position.x, goal->pose.position.y, goal->pose.position.z));
            
            KDL::JntArray q_target_kdl(robot_->getNrJnts());
            robot_->getInverseKinematics(target_f, q_target_kdl);
            
            std::vector<double> q_vec;
            for(unsigned int i=0; i<q_target_kdl.rows(); i++) q_vec.push_back(q_target_kdl(i));
            
            publish_trajectory(q_vec, traj_duration_);
        }

        // 4. Attesa attiva
        double time_slept = 0.0;
        double total_wait = traj_duration_ + 0.5;
        rclcpp::Rate rate(10); 

        while (rclcpp::ok() && time_slept < total_wait) {
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
            rate.sleep();
            time_slept += 0.1;
        }

        result->success = true;
        goal_handle->succeed(result);
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& msg) {
        // SAFETY FIRST: Se il robot non è pronto, non fare nulla!
        if (robot_ == nullptr) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Verifica corrispondenza giunti (Evita crash se Gazebo manda vettori diversi)
        if (msg.position.size() < joint_positions_.rows()) {
            return; 
        }

        for (size_t i = 0; i < joint_positions_.rows(); ++i) {
            joint_positions_.data[i] = msg.position[i];
            joint_velocities_.data[i] = msg.velocity[i];
        }
        
        state_received_ = true; // Segnala che abbiamo dati validi
    }

    // Boilerplate Action
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteTrajectory::Goal>) 
    { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTrajectory>) 
    { return rclcpp_action::CancelResponse::ACCEPT; }
    
    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) 
    { std::thread{std::bind(&Iiwa_pub_sub::execute, this, std::placeholders::_1), goal_handle}.detach(); }

    std::shared_ptr<KDLRobot> robot_;
    std::shared_ptr<KDLController> controller_;
    KDL::JntArray joint_positions_, joint_velocities_;
    double traj_duration_, Kp_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmdPublisher_;
    
    std::mutex mutex_;
    bool state_received_; // <--- Variabile fondamentale
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Iiwa_pub_sub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
