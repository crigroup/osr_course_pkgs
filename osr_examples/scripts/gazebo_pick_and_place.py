#!/usr/bin/env python
import rospy
import criros
import IPython
import numpy as np
import openravepy as orpy
import tf.transformations as tr
from osr_openrave import kinematics, planning
from osr_control.controllers import GripperController, JointTrajectoryController


if __name__ == '__main__':
  # Initialize a ROS node
  rospy.init_node('gazebo_pick_and_place')
  
  # Load the OpenRAVE environment
  env = orpy.Environment()
  if not env.Load('worlds/cubes_task.env.xml'):
    rospy.logerr('Failed to load the world. Did you run: catkin_make install?')
    exit(1)
  env.SetDefaultViewer()
  Tcamera = tr.euler_matrix(*np.deg2rad([-120, 13, 135]))
  Tcamera[:3,3] = [1, 1, 2]
  env.GetViewer().SetCamera(Tcamera)
  # Setup robot and manipulator
  robot = env.GetRobot('robot')
  manipulator = robot.SetActiveManipulator('gripper')
  robot.SetActiveDOFs(manipulator.GetArmIndices())
  taskmanip = orpy.interfaces.TaskManipulation(robot)
  # Scale down the velocity and acceleration limits
  robot.SetDOFVelocityLimits(robot.GetDOFVelocityLimits()*0.4)
  robot.SetDOFAccelerationLimits(robot.GetDOFAccelerationLimits()*0.2)
  # Connect to the hardware interfaces
  trajectory_controller = JointTrajectoryController()
  gripper_controller = GripperController()
  
  # Load IKFast. Generate it if needed
  iktype = orpy.IkParameterization.Type.Transform6D
  success = kinematics.load_ikfast(robot, iktype)
  if not success:
    rospy.logerr('Failed to load IKFast for {0}, manipulator: {1}'.format(robot.GetName(), manipulator.GetName()))
    IPython.embed()
    exit(1)
  # Load links stats for finding closest IK solutions
  statsmodel = orpy.databases.linkstatistics.LinkStatisticsModel(robot)
  if not statsmodel.load():
    rospy.loginfo('Generating LinkStatistics database. It will take around 1 minute...')
    statsmodel.autogenerate()
  statsmodel.setRobotWeights()
  statsmodel.setRobotResolutions(xyzdelta=0.01)
  
  # Find a valid IK solution for grasping the cube
  cube = env.GetKinBody('cube01')
  cube_centroid = cube.ComputeAABB().pos()
  Tgrasp = tr.euler_matrix(0, np.pi, 0)
  Tgrasp[:3,3] = cube_centroid
  qgrasp = kinematics.find_closest_iksolution(robot, Tgrasp, iktype)
  axes = []
  axes.append( orpy.misc.DrawAxes(env, Tgrasp, dist=0.05) )
  
  # Move to the grasping pose
  traj = planning.plan_to_joint_configuration(robot, qgrasp)
  ros_traj = criros.conversions.ros_trajectory_from_openrave(robot.GetName(), traj)
  trajectory_controller.set_trajectory(ros_traj)
  trajectory_controller.start()
  robot.GetController().SetPath(traj)
  trajectory_controller.wait()
  # Grasp the cube
  gripper_controller.command(0.05)
  taskmanip.CloseFingers()
  gripper_controller.wait()
  robot.WaitForController(0)
  robot.Grab(cube)
  gripper_controller.grab('{0}::link'.format(cube.GetName()))
  
  # Find a valid IK solution for the retreat pose
  Tretreat = np.array(Tgrasp)
  Tretreat[2,3] += 0.1
  axes.append( orpy.misc.DrawAxes(env, Tretreat, dist=0.05) )
  qretreat = kinematics.find_closest_iksolution(robot, Tretreat, iktype)
  
  # Move to the retreat pose
  traj = planning.plan_to_joint_configuration(robot, qretreat)
  ros_traj = criros.conversions.ros_trajectory_from_openrave(robot.GetName(), traj)
  trajectory_controller.set_trajectory(ros_traj)
  trajectory_controller.start()
  robot.GetController().SetPath(traj)
  trajectory_controller.wait()
  
  # Find a valid IK solution for the placing pose
  base_cube = env.GetKinBody('cube02')
  aabb = base_cube.ComputeAABB()
  Tplace = np.array(Tgrasp)
  Tplace[:3,3] = aabb.pos()
  Tplace[2,3] += 2*aabb.extents()[2]
  axes.append( orpy.misc.DrawAxes(env, Tplace, dist=0.05) )
  qplace = kinematics.find_closest_iksolution(robot, Tplace, iktype)
  
  # Move to the placing pose
  traj = planning.plan_to_joint_configuration(robot, qplace)
  ros_traj = criros.conversions.ros_trajectory_from_openrave(robot.GetName(), traj)
  trajectory_controller.set_trajectory(ros_traj)
  trajectory_controller.start()
  robot.GetController().SetPath(traj)
  trajectory_controller.wait()
  
  # Release the cube
  gripper_controller.open()
  taskmanip.ReleaseFingers()
  gripper_controller.wait()
  robot.WaitForController(0)
  robot.Release(cube)
  gripper_controller.release('{0}::link'.format(cube.GetName()))
  
  # Move back home
  traj = planning.plan_to_joint_configuration(robot, np.zeros(6))
  ros_traj = criros.conversions.ros_trajectory_from_openrave(robot.GetName(), traj)
  trajectory_controller.set_trajectory(ros_traj)
  trajectory_controller.start()
  robot.GetController().SetPath(traj)
  trajectory_controller.wait()
  
  # Use for debugging
  IPython.embed()
  env.Reset()
  env.Destroy()
