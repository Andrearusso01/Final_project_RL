#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import yaml
import time

waypoints_yaml = '''
waypoints:
  - {x: -6.0, y: 3.1, z: 0.0, qz: 0.0, qw: 1.0}    # WP 1
  - {x: -4.8, y: 0.0, z: 0.0, qz: 0.0, qw: 1.0}    # WP 2
  - {x: -2.3, y: -0.5, z: 0.0, qz: 0.0, qw: 1.0}   # WP 3
  - {x: -1.8, y: -2.6, z: 0.0, qz: 0.0, qw: 1.0}   # WP 4 (Ingresso corridoio)
  - {x: 3.0, y: -2.6, z: 0.0, qz: 0.0, qw: 1.0}    # WP 4.5 (Metà corridoio)
  - {x: 6.5, y: -3.2, z: 0.0, qz: 0.99994, qw: 0.01079} # WP 5 (Uscita - PIU LARGO)
  - {x: 7.20, y: -4.10, z: 0.0, qz: -0.7643, qw: 0.6448} # WP 6 (Target finale)
'''

def create_pose(navigator, wp_data):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = wp_data['x']
    pose.pose.position.y = wp_data['y']
    pose.pose.orientation.z = wp_data['qz']
    pose.pose.orientation.w = wp_data['qw']
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()
    data = yaml.safe_load(waypoints_yaml)
    waypoints = data['waypoints']

    # --- INITIAL POSE ---
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -9.2
    initial_pose.pose.position.y = 3.1
    initial_pose.pose.orientation.w = 1.0
    
    print("Inizializzazione Nav2...")
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    for i, wp in enumerate(waypoints):
        wp_idx = i + 1
        
        # Al punto 5 e 6, proviamo a resettare SOLO se Nav2 è in crisi
        if wp_idx >= 5:
            print(f"--- RESET DI SICUREZZA PER WAYPOINT {wp_idx} ---")
            # Usiamo la posa del punto PRECEDENTE per dare stabilità
            reset_pose = create_pose(navigator, waypoints[i-1]) 
            navigator.setInitialPose(reset_pose)
            time.sleep(1.0)

        goal_pose = create_pose(navigator, wp)
        print(f"Andando al Waypoint {wp_idx}...")
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            # Se il robot ci mette troppo (bloccato), proviamo a cancellare e ridare il comando
            feedback = navigator.getFeedback()
            if feedback:
                # Se la distanza al goal non scende per 5 secondi, Nav2 potrebbe essere incastrato
                pass
        
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Waypoint {wp_idx} raggiunto!")
        else:
            print(f"Waypoint {wp_idx} fallito. Provo a forzare il passaggio al prossimo...")
            # Non usciamo, proviamo a passare al prossimo waypoint comunque
            continue

    print("MISSIONE TERMINATA")
    exit(0)

if __name__ == '__main__':
    main()
