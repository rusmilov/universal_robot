import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_python_interface', anonymous=True)

# Instantiate a `RobotCommander` object
robot = moveit_commander.RobotCommander()

# Instantiate a `PlanningSceneInterface` object
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a `MoveGroupCommander` object for the UR5's arm group
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

# For displaying trajectories in RViz:
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# Get the current joint values
joint_goal = group.get_current_joint_values()

# Modify the joint values
joint_goal[0] = 2.0  # shoulder_pan_joint
joint_goal[1] = -2.0  # shoulder_lift_joint
joint_goal[2] = 2.0  # elbow_joint
joint_goal[3] = -2.0  # wrist_1_joint
joint_goal[4] = 2.0  # wrist_2_joint
joint_goal[5] = -2.0  # wrist_3_joint

# Set the joint target, plan, and execute
plan = group.plan()
group.go(joint_goal, wait=True)

# Ensure that there is no residual movement
group.stop()

# import sys
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from math import pi

# def main():
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('moveit_python_interface', anonymous=True)

#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     group = moveit_commander.MoveGroupCommander("manipulator")

#     # Set planning time
#     # group.set_planning_time(100)  # Increase planning time limit

#     # Set joint goal
#     joint_goal = group.get_current_joint_values()
#     joint_goal[0] = 3.0  # shoulder_pan_joint
#     joint_goal[1] = 1.0  # shoulder_lift_joint
#     joint_goal[2] = -2.5  # elbow_joint
#     joint_goal[3] = 2.0  # wrist_1_joint
#     joint_goal[4] = 3.0  # wrist_2_joint
#     joint_goal[5] = -3.0  # wrist_3_joint

#     group.set_joint_value_target(joint_goal)

#     # Plan and execute
#     # plan = true # group.plan()
#     # if plan:
#     #     group.go(wait=True)
#     # else:
#     #     rospy.loginfo("Plan failed")

#     group.go(wait=True)

#     # moveit_commander.roscpp_shutdown()

# if __name__ == '__main__':
#     main()
