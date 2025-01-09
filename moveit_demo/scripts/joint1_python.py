#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotTrajectory, Grasp, PlaceLocation, Constraints
from sensor_msgs.msg import JointState
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states

    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:

    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:

    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_planner_id('RRTkConfigDefault')

    move_group.set_max_velocity_scaling_factor(0.1)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:

    planning_frame = move_group.get_planning_frame()
    print ("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    # move_group.getEndEffectorLink
    eef_link = move_group.get_end_effector_link()
    print ("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print ("============ Available Planning Groups:", robot.get_group_names())

    # 获取机械臂状态
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print ("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_back(self):
    # 运动学正解
    move_group = self.move_group

    # 获取当前关节角度值
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 2.495798577584196e-06
    joint_goal[1] = 0.0010494629354047547
    joint_goal[2] = -3.4866916641007606e-05
    joint_goal[3] = -1.572429767637109
    joint_goal[4] = 3.877825585174577e-05
    joint_goal[5] = 1.5709280766224047
    joint_goal[6] = 0.7849653674564285

    # 规划执行到目标点（阻塞）(提高实时性)
    move_group.go(joint_goal, wait=True)
    # rospy.sleep(1)

    # joint_goal[0] += 1.57
    # move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_joint_state(self):
    # 运动学正解
    # 输入：机械臂各个关节的目标值
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:

    # 获取当前关节角度值
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -0.09710210646870741
    joint_goal[1] = 0.14022892195670564
    joint_goal[2] = -0.9363551844333426
    joint_goal[3] = -2.057766297941847
    joint_goal[4] = 0.1336904613169746
    joint_goal[5] = 2.1374147936139565
    joint_goal[6] = -0.3160219769148445

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group

    # 规划执行到目标点（阻塞）(提高实时性)
    move_group.go(joint_goal, wait=True)
    # rospy.sleep(1)

    # joint_goal[0] += 1.57
    # move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def get_trajectory2(self):
    # get_traj = []

    # 运动学逆解
    # 输入：机械臂末端的姿态（xyz, ox oy oz ow）
    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -0.13562810641781553
    joint_goal[1] = 0.08474102485773383
    joint_goal[2] = -0.1125179864129624
    joint_goal[3] = -2.3113127900137567
    joint_goal[4] = 0.012481000332082104
    joint_goal[5] = 2.3907465314546013
    joint_goal[6] = 0.52817738464607

    get_traj = move_group.plan(joint_goal)
    # print move_group.get_remembered_joint_values()
    move_group.stop()

    # out = get_traj.joint_trajectory.points[0].velocities
    # print(' '.join(str(out)))

    # a = []
    # length = range(len(get_traj.joint_trajectory.points))
    
    # with open("/home/shibao/clutter_ws2/velocities.txt", "a") as f:
    #   for i in length:
    #     out = get_traj.joint_trajectory.points[i].velocities
    #     out2 = out.replace('(', '').replace(')', '')
    #     f.write(str(get_traj.joint_trajectory.points[i].velocities))
    #     f.write('\r\n')

    # a = []
    # length = range(len(get_traj.joint_trajectory.points))
    # for i in length:
    #   a.append(get_traj.joint_trajectory.points[i].accelerations)
    # with open("/home/shibao/clutter_ws2/accelerations.txt", "a") as f:
    #   f.write(str(a))

    # with open("/home/shibao/clutter_ws2/jointStates.txt", "a") as f:
    #   f.write(str(get_traj.joint_trajectory.points))
    # print(get_traj.joint_trajectory.points[3].velocities)
    # print(len(get_traj.joint_trajectory.points))

    # current_joints = move_group.get_current_joint_values()
    # return all_close(joint_goal, current_joints, 0.01)
    return get_traj

  def get_trajectory3(self):
    # get_traj = []

    # 运动学逆解
    # 输入：机械臂末端的姿态（xyz, ox oy oz ow）
    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 2.7321094697247794
    joint_goal[1] = -0.8134774146054458
    joint_goal[2] = -2.2543700029175255
    joint_goal[3] = -1.4649609209994292
    joint_goal[4] = -0.6533762114157122
    joint_goal[5] = 1.960005461111244
    joint_goal[6] = 1.3621184287196026

    get_traj = move_group.plan(joint_goal)
    # print move_group.get_remembered_joint_values()
    move_group.stop()

    # a = []
    # length = range(len(get_traj.joint_trajectory.points))
    # with open("/home/shibao/clutter_ws2/accelerations.txt", "a") as f:
    #   for i in length:
    #     f.write(str(get_traj.joint_trajectory.points[i].accelerations))
    #     f.write('\r\n')

    # with open("/home/shibao/clutter_ws2/jointStates.txt", "a") as f:
    #   f.write(str(get_traj.joint_trajectory.points))
    # print(get_traj.joint_trajectory.points[0])

    # current_joints = move_group.get_current_joint_values()
    # return all_close(joint_goal, current_joints, 0.01)
    return get_traj

  def get_trajectory4(self):
    # get_traj = []

    # 运动学逆解
    # 输入：机械臂末端的姿态（xyz, ox oy oz ow）
    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -0.9544360186845265
    joint_goal[1] = -0.6783486854020465
    joint_goal[2] = 1.8400115567547608
    joint_goal[3] = -1.222076793010144
    joint_goal[4] = 0.6540416493345782
    joint_goal[5] = 1.46118179819315
    joint_goal[6] = 1.8253284750560974

    get_traj = move_group.plan(joint_goal)
    # print move_group.get_remembered_joint_values()
    move_group.stop()
    print(get_traj)

    # current_joints = move_group.get_current_joint_values()
    # return all_close(joint_goal, current_joints, 0.01)
    return get_traj

  def get_trajectory5(self):
    # get_traj = []

    # 运动学逆解
    # 输入：机械臂末端的姿态（xyz, ox oy oz ow）
    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -0.09710210646870741
    joint_goal[1] = 0.14022892195670564
    joint_goal[2] = -0.9363551844333426
    joint_goal[3] = -2.057766297941847
    joint_goal[4] = 0.1336904613169746
    joint_goal[5] = 2.1374147936139565
    joint_goal[6] = -0.3160219769148445

    get_traj = move_group.plan(joint_goal)
    # print move_group.get_remembered_joint_values()
    move_group.stop()
    print(get_traj)

    # current_joints = move_group.get_current_joint_values()
    # return all_close(joint_goal, current_joints, 0.01)
    return get_traj


    

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):

    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL



def main():
  try:
    print ("")
    print ("----------------------------------------------------------")
    print ("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print ("----------------------------------------------------------")
    print ("Press Ctrl-D to exit at any time")
    print ("")
    print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    # 阻塞
    raw_input()
    # 初始化MoveGroupPythonIntefaceTutorial（init）
    tutorial = MoveGroupPythonIntefaceTutorial()

    # arm_group = moveit_commander.move_group.MoveGroupCommander("panda_arm")
    # arm_group.set_planner_id('geometric::RRT')

    print ("============ Press `Enter` to a joint trajectory2 ...")
    raw_input()
    joint_plan2 = tutorial.get_trajectory2()

    # print ("============ Press `Enter` to display a saved trajectory")
    # raw_input()
    # tutorial.display_trajectory(joint_plan2)

    print ("============ Press `Enter` to execute a saved path2 ...")
    raw_input()
    tutorial.execute_plan(joint_plan2)

    print ("============ Press `Enter` to a joint trajectory3 ...")
    raw_input()
    joint_plan3 = tutorial.get_trajectory3()

    print ("============ Press `Enter` to execute a saved path3 ...")
    raw_input()
    tutorial.execute_plan(joint_plan3)

    # print ("============ Press `Enter` to a joint trajectory4 ...")
    # raw_input()
    # joint_plan4 = tutorial.get_trajectory4()

    # print ("============ Press `Enter` to execute a saved path4 ...")
    # raw_input()
    # tutorial.execute_plan(joint_plan4)

    # print ("============ Press `Enter` to a joint trajectory5 ...")
    # raw_input()
    # joint_plan5 = tutorial.get_trajectory5()

    # print ("============ Press `Enter` to execute a saved path5 ...")
    # raw_input()
    # tutorial.execute_plan(joint_plan5)

    print ("============ Press `Enter` to go back ...")
    raw_input()
    tutorial.go_back()

    # print ("============ Press `Enter` to execute a movement using a joint state goal ...")
    # raw_input()
    # tutorial.go_to_joint_state()

    # print ("============ Press `Enter` to execute a movement using a pose goal ...")
    # raw_input()
    # tutorial.go_to_pose_goal()

    # print ("============ Press `Enter` to plan and display a Cartesian path ...")
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path()

    # print ("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
    # raw_input()
    # tutorial.display_trajectory(cartesian_plan)

    # print ("============ Press `Enter` to execute a saved path ...")
    # raw_input()
    # tutorial.execute_plan(cartesian_plan)

    print ("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
