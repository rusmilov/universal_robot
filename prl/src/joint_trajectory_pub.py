#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

def load_trajectory(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def main():
    rospy.init_node('joint_trajectory_publisher')

    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    traj_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    
    rate = rospy.Rate(10)

    trajectory = load_trajectory('/prl/config/joint_trajectory.yaml')

    traj_msg = JointTrajectory()
    traj_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    for point in trajectory['joint_trajectory']:
        traj_point = JointTrajectoryPoint()
        traj_point.positions = point['positions']
        traj_point.velocities = point['velocities']
        traj_point.time_from_start = rospy.Duration(point['time_from_start'])
        traj_msg.points.append(traj_point)
    
    while not rospy.is_shutdown():
        traj_pub.publish(traj_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

