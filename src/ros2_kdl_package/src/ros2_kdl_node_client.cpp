#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Header per i messaggi
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

// Header per le trasformate (TF2)
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Importante per trasformare le pose

#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace std::chrono_literals;

class PickAndPlaceClient : public rclcpp::Node
{
public:
  using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
  using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

  PickAndPlaceClient()
  : Node("pick_place_client")
  {
    // 1. Action Client per muovere il robot
    client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/iiwa/ExecuteTrajectory");

    // 2. Publisher per attivare la ventosa (tramite Bridge o topic diretto)
    gripper_pub_ = this->create_publisher<std_msgs::msg::Bool>("/gripper/toggle", 10);

    // 3. Setup TF2 per le trasformate
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 4. Subscriber per la visione (Aruco)
    vision_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aruco_single/pose", 10, 
      std::bind(&PickAndPlaceClient::vision_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Client pronto. In attesa di vedere un Aruco...");
  }

private:
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_sub_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  bool goal_in_progress_ = false;

  // Questa funzione viene chiamata appena la telecamera vede l'Aruco
  void vision_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (goal_in_progress_) return; // Non accettare nuovi comandi se stiamo già lavorando

    RCLCPP_INFO(this->get_logger(), "Aruco rilevato! Calcolo traiettoria...");

    geometry_msgs::msg::PoseStamped target_pose_base;

    try {
      // TRASFORMATA: Da 'camera_optical_frame' a 'iiwa_base' (o base_link)
      // Cerca la trasformata più recente disponibile
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        "iiwa_link_0",        // Frame di destinazione (Base robot)
        "iiwa_camera_optical_frame", // Frame sorgente (Camera)
        tf2::TimePointZero
      );

      // Applica la trasformata alla posa dell'Aruco
      tf2::doTransform(*msg, target_pose_base, transform);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Fallimento TF: %s", ex.what());
      return;
    }

    // Ora abbiamo la posa dell'Aruco rispetto alla base del robot.
    // Avviamo la sequenza di Pick
    start_pick_sequence(target_pose_base.pose);
  }

  void start_pick_sequence(geometry_msgs::msg::Pose object_pose)
  {
    goal_in_progress_ = true;

    // --- FASE 1: APPROACH (Sopra il pacco) ---
    auto goal_msg = ExecuteTrajectory::Goal();
    
    // Copiamo la posa dell'oggetto
    goal_msg.pose = object_pose;
    
    // MODIFICA Z: Stiamo 20cm sopra
    goal_msg.pose.position.z += 0.20; 
    
    goal_msg.pose.orientation.x = 1.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = 0.0;
    goal_msg.pose.orientation.w = 0.0;

    goal_msg.order = 0; // 0 significa "Usa la posa cartesiana" (modifica il server per gestire questo!)

    RCLCPP_INFO(this->get_logger(), "Invio Goal: APPROACH (z+0.20)");
    
    auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = 
      std::bind(&PickAndPlaceClient::approach_completed, this, std::placeholders::_1, object_pose);
    
    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void approach_completed(
    const GoalHandleExecuteTrajectory::WrappedResult & result, 
    geometry_msgs::msg::Pose object_pose)
  {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Approach fallito!");
      goal_in_progress_ = false;
      return;
    }

    // --- FASE 2: DESCENT (Presa) ---
    auto goal_msg = ExecuteTrajectory::Goal();
    goal_msg.pose = object_pose;
    
    // MODIFICA Z: Scendiamo leggermente DENTRO il pacco per la ventosa
    // Esempio: altezza pacco + margine di schiacciamento
    // Se object_pose è il centro del tag (superficie), scendiamo di 5mm
    goal_msg.pose.position.z -= 0.05; 
    goal_msg.order = 0;

    RCLCPP_INFO(this->get_logger(), "Invio Goal: GRASP (contatto)");

    auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = 
      std::bind(&PickAndPlaceClient::grasp_completed, this, std::placeholders::_1, object_pose);
    
    client_->async_send_goal(goal_msg, send_goal_options);
  }

 // Nota: ho aggiunto 'object_pose' agli argomenti
  void grasp_completed(
    const GoalHandleExecuteTrajectory::WrappedResult & result,
    geometry_msgs::msg::Pose object_pose)
  {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Discesa fallita!");
      goal_in_progress_ = false;
      return;
    }

    // --- FASE 3: ATTIVA VENTOSA (GRASP) ---
    // Con il plugin DetachableJoint su topic "toggle":
    // TRUE = Attacca
    RCLCPP_INFO(this->get_logger(), "ATTIVAZIONE PRESA (Topic: /gripper/toggle -> True)");
    auto msg = std_msgs::msg::Bool();
    msg.data = true; 
    gripper_pub_->publish(msg);

    // Aspettiamo un attimo che il vincolo fisico si attivi
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // --- FASE 4: LIFT (Salita) ---
    RCLCPP_INFO(this->get_logger(), "Invio Goal: LIFT (Salita z+0.20)");

    auto goal_msg = ExecuteTrajectory::Goal();
    
    // Partiamo dalla posizione di presa e saliamo di 20cm
    goal_msg.pose = object_pose;
    goal_msg.pose.position.z += 0.20; 
    
    goal_msg.order = 0; // Pick & Place mode

    auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
    
    // Callback finale quando ha finito di salire
    send_goal_options.result_callback = 
      std::bind(&PickAndPlaceClient::lift_completed, this, std::placeholders::_1);
    
    client_->async_send_goal(goal_msg, send_goal_options);
  }

  // --- NUOVA FUNZIONE: FINE OPERAZIONE ---
  void lift_completed(const GoalHandleExecuteTrajectory::WrappedResult & result)
  {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "PICK COMPLETATO CON SUCCESSO!");
      } else {
          RCLCPP_ERROR(this->get_logger(), "Errore durante la risalita.");
      }
      
      // SOLO ORA siamo liberi per il prossimo pacco
      goal_in_progress_ = false; 
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndPlaceClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
