// Copyright (C) 2007 Francois Cauwe <francois at cauwe dot org>
// Modified for ROS 2 Action Server Integration

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

    Iiwa_pub_sub() : Node("ros2_kdl_node")
    {
        using namespace std::placeholders;

        // --- DICHIARAZIONE PARAMETRI ---
        this->declare_parameter<double>("traj_duration", 5.0);
        this->declare_parameter<double>("total_time", 10.0);
        this->declare_parameter<int>("trajectory_len", 100);
        this->declare_parameter<double>("Kp", 1.0);
        this->declare_parameter<std::vector<double>>("end_position", {0.1, 0.1, 0.1});
        this->declare_parameter<double>("acc_duration", 1.5);
        this->declare_parameter<std::string>("cmd_interface", "velocity");
        this->declare_parameter<std::string>("ctrl", "velocity_ctrl_null");
        this->declare_parameter<std::string>("traj_type", "linear");
        this->declare_parameter<std::string>("s_type", "trapezoidal");

        // Recupero parametri
        this->get_parameter("traj_duration", traj_duration_);
        this->get_parameter("total_time", total_time_);
        this->get_parameter("trajectory_len", trajectory_len_);
        this->get_parameter("Kp", Kp_);
        this->get_parameter("end_position", end_position_vec_);
        this->get_parameter("acc_duration", acc_duration_);
        this->get_parameter("cmd_interface", cmd_interface_);
        this->get_parameter("ctrl", ctrl_);
        this->get_parameter("traj_type", traj_type_);
        this->get_parameter("s_type", s_type_);

        // --- INIZIALIZZAZIONE KDL ---
        setup_kdl();

        // --- TOPIC E ACTION SERVER ---
        this->action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            this,
            "ExecuteTrajectory",
            std::bind(&Iiwa_pub_sub::handle_goal, this, _1, _2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, _1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, _1));

        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, _1));

        MarkerPoseSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, _1));

        std::string cmd_topic = (cmd_interface_ == "position") ? "iiwa_arm_controller/commands" : 
                                (cmd_interface_ == "velocity") ? "velocity_controller/commands" : 
                                "effort_controller/commands";
        
        cmdPublisher_ = this->create_publisher<FloatArray>(cmd_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Node Ready. Interface: %s, Ctrl: %s", cmd_interface_.c_str(), ctrl_.c_str());
    }

private:
    // --- KDL SETUP ---
    void setup_kdl() {
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "Waiting for robot_description...");
        }
        
        auto parameter = parameters_client->get_parameters({"robot_description"});
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
            return;
        }

        robot_ = std::make_shared<KDLRobot>(robot_tree);
        unsigned int nj = robot_->getNrJnts();
        
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96;
        q_max.data <<  2.96,  2.09,  2.96,  2.09,  2.96,  2.09,  2.96;
        robot_->setJntLimits(q_min, q_max);

        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        desired_commands_.resize(nj, 0.0);

        controller_ = KDLController(*robot_);
        robot_->addEE(KDL::Frame::Identity());
    }

    // --- ACTION CALLBACKS ---
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteTrajectory::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid; (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal cancel requested");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle) {
        std::thread{std::bind(&Iiwa_pub_sub::execute, this, std::placeholders::_1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting trajectory execution...");

    // 1. RECUPERO DATI DAL CLIENT
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
    auto result = std::make_shared<ExecuteTrajectory::Result>();

    rclcpp::Rate rate(50);  // 50 Hz
    double dt = 1.0 / 50.0;
    double t = 0.0;

    // 2. ATTESA JOINT STATES
    while (rclcpp::ok() && joint_positions_.data.norm() < 0.0001) {
        RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
        rclcpp::sleep_for(100ms);
    }

    // 3. UPDATE ROBOT E POSA INIZIALE
    robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
    KDL::Frame init_frame = robot_->getEEFrame();
    Eigen::Vector3d start_p(init_frame.p.data);

    // Selezione Target di posizione
    Eigen::Vector3d end_p;
    if (goal->order == 0) {
        RCLCPP_INFO(this->get_logger(), "Received Target: [X: %.2f, Y: %.2f, Z: %.2f]",
                    goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
        end_p << goal->pose.position.x, goal->pose.position.y, goal->pose.position.z;
    } else {
        end_p << end_position_vec_[0], end_position_vec_[1], end_position_vec_[2];
    }

    // 4. INIZIALIZZAZIONE PLANNER
    if (traj_type_ == "linear") {
        planner_ = KDLPlanner(traj_duration_, acc_duration_, start_p, end_p);
    } else {
        planner_ = KDLPlanner(traj_duration_, start_p, 0.15, acc_duration_);
    }

    // --- LOOP DI CONTROLLO TRAIETTORIA ---
    while (rclcpp::ok() && t < total_time_ && ctrl_ != "vision") {
        
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            stop_robot();
            return;
        }

        // Calcolo traiettoria desiderata (posizione e velocità lineare)
        if (traj_type_ == "linear") {
            p_ = (s_type_ == "trapezoidal") ? planner_.linear_traj_trapezoidal(t) : planner_.linear_traj_cubic(t);
        } else {
            p_ = (s_type_ == "trapezoidal") ? planner_.circular_traj_trapezoidal(t) : planner_.circular_traj_cubic(t);
        }

        // Aggiorna stato robot
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame current_f = robot_->getEEFrame();

        // 5. CALCOLO ERRORE DI POSIZIONE
        Eigen::Vector3d pos_error = p_.pos - Eigen::Vector3d(current_f.p.data);

        // 6. CALCOLO ERRORE DI ORIENTAMENTO (CORRETTO)
        // Creiamo la rotazione desiderata dal quaternion ricevuto dal Client
        KDL::Rotation target_rot = KDL::Rotation::Quaternion(
            goal->pose.orientation.x, goal->pose.orientation.y, 
            goal->pose.orientation.z, goal->pose.orientation.w);

        // Calcoliamo la rotazione relativa: R_err = R_curr^-1 * R_target
        // .GetRot() restituisce il vettore di rotazione la cui direzione è l'asse e il modulo l'angolo
        KDL::Vector ori_error_kdl = (current_f.M.Inverse() * target_rot).GetRot();
        Eigen::Vector3d ori_error(ori_error_kdl.x(), ori_error_kdl.y(), ori_error_kdl.z());

        // Componiamo l'errore a 6 gradi di libertà
        Eigen::Matrix<double, 6, 1> error_6d;
        error_6d << pos_error, ori_error;

        // Feedback al client
        feedback->position_error = {pos_error(0), pos_error(1), pos_error(2)};
        goal_handle->publish_feedback(feedback);

        // 7. CALCOLO COMANDO DI VELOCITÀ AI GIUNTI
        if (ctrl_ == "velocity_ctrl") {
            Vector6d cartvel;
            // Velocità desiderata = Velocità feedforward (p_.vel) + Guadagno (Kp_) * Errore
            cartvel << p_.vel + Kp_ * pos_error, Kp_ * ori_error; 
            joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
        } 
        else if (ctrl_ == "velocity_ctrl_null") {
            // Utilizzo del controller specifico per la gestione del nullo (se implementato in kdl_control.cpp)
            joint_velocities_cmd_ = controller_.velocity_ctrl_null(error_6d, Kp_);
        }

        // 8. PUBBLICAZIONE COMANDI
        if (!std::isnan(joint_velocities_cmd_.data(0))) {
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data.assign(joint_velocities_cmd_.data.data(), 
                                joint_velocities_cmd_.data.data() + joint_velocities_cmd_.rows());
            cmdPublisher_->publish(cmd_msg);
        }

        t += dt;
        rate.sleep();
    }

    // --- LOOP VISIONE (Invariato per logica, ma con feedback) ---
    while (rclcpp::ok() && ctrl_ == "vision") {
        double norm = cPo_.norm();
        if (norm < 0.001) norm = 1.0; 
        Eigen::Vector3d s = cPo_ / norm;
        Eigen::Vector3d sd(0, 0, 1);
        Eigen::Vector3d dir_error = s - sd;

        joint_velocities_cmd_ = controller_.vision_ctrl(Kp_, cPo_, sd);

        feedback->position_error = {dir_error(0), dir_error(1), dir_error(2)};
        goal_handle->publish_feedback(feedback);

        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data.assign(joint_velocities_cmd_.data.data(),
                            joint_velocities_cmd_.data.data() + joint_velocities_cmd_.rows());
        cmdPublisher_->publish(cmd_msg);

        rate.sleep(); 
    }

    result->success = true;
    goal_handle->succeed(result);
    stop_robot();
    RCLCPP_INFO(this->get_logger(), "Execution completed.");
}
    

    

    void stop_robot() {
        FloatArray stop_msg;
        stop_msg.data.resize(desired_commands_.size(), 0.0);
        cmdPublisher_->publish(stop_msg);
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState& msg) {
        for (size_t i = 0; i < msg.position.size(); ++i) {
            joint_positions_.data[i] = msg.position[i];
            joint_velocities_.data[i] = msg.velocity[i];
        }
    }

    void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped& msg) {
        cPo_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    }

    // --- MEMBRI ---
    std::shared_ptr<KDLRobot> robot_;
    KDLController controller_;
    KDLPlanner planner_;
    trajectory_point p_;

    double traj_duration_, total_time_, Kp_, acc_duration_;
    int trajectory_len_;
    std::vector<double> end_position_vec_;
    std::string cmd_interface_, ctrl_, traj_type_, s_type_;

    KDL::JntArray joint_positions_, joint_velocities_, joint_positions_cmd_, joint_velocities_cmd_;
    std::vector<double> desired_commands_;
    Eigen::Vector3d cPo_;

    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr MarkerPoseSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Iiwa_pub_sub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
