#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math

def main():
    rospy.init_node('ur5_trajectory_publisher')
  
    # Publisher to the joint trajectory controller
    trajectory_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    
    # Publisher for joint states
    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
  
    # JointTrajectory message
    trajectory = JointTrajectory()
    trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
  
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.get_time()

    position = 0
    velocity = 0
  
    while not rospy.is_shutdown():
        position += 1.0
        velocity += 0.0
        elapsed = rospy.get_time() - start_time
      
        # Create a new point in the trajectory
        point = JointTrajectoryPoint()
        point.positions = [0.0, math.sin(elapsed), 0.0, 0.0, 0.0, 0.0]
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = rospy.Duration(elapsed)
        
        trajectory.header.stamp = rospy.Time.now()
        trajectory.points = [point]
      
        # Publish the trajectory
        trajectory_pub.publish(trajectory)
      
        # Publish joint states for visualization
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = trajectory.joint_names
        joint_state.position = point.positions
        joint_state.velocity = point.velocities
        joint_state.effort = []  # You can leave this empty if not relevant
      
        joint_state_pub.publish(joint_state)
      
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
