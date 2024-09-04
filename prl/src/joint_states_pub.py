#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def publish_joint_states():
    rospy.init_node('joint_state_publisher')
    
    # Create a publisher for joint states
    joint_pub = rospy.Publisher('/move_group/goal', JointState, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    joint_state = JointState()
    
    # Fill in the joint state header
    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()
    
    # Joint names as per URDF
    joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    # Initial positions and velocities (example values)
    joint_state.position = [10.0, 30.0, 100.0, 270.0, 330.0, 5.0]
    joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    while not rospy.is_shutdown():
        # Update the timestamp
        joint_state.header.stamp = rospy.Time.now()
        
        # Publish the joint state
        joint_pub.publish(joint_state)

        # joint_state.position[0] += 0.1
        # joint_state.position[1] -= 0.1
        # joint_state.position[2] += 0.1
        # joint_state.position[3] -= 0.1
        # joint_state.position[4] += 0.1
        # joint_state.position[5] -= 0.1

        # joint_state.velocity[0] += 10.0
        # joint_state.velocity[1] += 10.0
        # joint_state.velocity[2] += 10.0
        # joint_state.velocity[3] += 10.0
        # joint_state.velocity[4] += 10.0
        # joint_state.velocity[5] += 10.0
        
        # Log the published joint states
        # rospy.loginfo("joint positions: %s, joint velocities: %s", joint_state.position, joint_state.velocity)
        
        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
