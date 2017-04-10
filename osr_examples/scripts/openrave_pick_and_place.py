#!/usr/bin/env python
import IPython
import numpy as np
import openravepy as orpy
import tf.transformations as tr


if __name__ == '__main__':
  # Load the environment
  env = orpy.Environment()
  if not env.Load('worlds/cubes_task.env.xml'):
    raise Exception('Failed to load the world. Did you run: catkin_make install?')
  env.SetDefaultViewer()
  Tcamera = tr.euler_matrix(*np.deg2rad([-120, 13, 135]))
  Tcamera[:3,3] = [1, 1, 2]
  env.GetViewer().SetCamera(Tcamera)
  
  # Setup robot and manipulator
  robot = env.GetRobot('robot')
  manipulator = robot.SetActiveManipulator('gripper')
  robot.SetActiveDOFs(manipulator.GetArmIndices())
  
  # Generate IKFast if needed
  iktype = orpy.IkParameterization.Type.Transform6D
  ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=iktype)
  if not ikmodel.load():
    print 'Generating IKFast {0}. It will take few minutes...'.format(iktype.name)
    ikmodel.autogenerate()
    print 'IKFast {0} has been successfully generated'.format(iktype.name)
  
  # Find a valid IK solution for grasping the cube
  cube = env.GetKinBody('cube01')
  cube_centroid = cube.ComputeAABB().pos()
  Tgrasp = tr.euler_matrix(0, np.pi, 0)
  Tgrasp[:3,3] = cube_centroid
  qgrasp = None
  ikparam = orpy.IkParameterization(Tgrasp, iktype)
  solutions = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
  if len(solutions) == 0:
    raise Exception('Failed to find a valid IK for grasping the cube')
  qgrasp = solutions[0]
  axes = []
  axes.append( orpy.misc.DrawAxes(env, Tgrasp, dist=0.05) )
  
  # Move to the grasping pose
  def plan_to_joint_values(qgoal):
    # Create the planner
    planner = orpy.RaveCreatePlanner(env,'birrt') # Using bidirectional RRT
    params = orpy.Planner.PlannerParameters()
    params.SetRobotActiveJoints(robot)
    params.SetGoalConfig(qgoal)
    params.SetExtraParameters('<_postprocessing planner="ParabolicSmoother"><_nmaxiterations>40</_nmaxiterations></_postprocessing>')
    planner.InitPlan(robot, params)
    # Plan a trajectory
    traj = orpy.RaveCreateTrajectory(env, '')
    planner.PlanPath(traj)
    return traj
  traj = plan_to_joint_values(qgrasp)
  controller = robot.GetController()
  controller.SetPath(traj)
  robot.WaitForController(0)
  # Grasp the cube
  taskmanip = orpy.interfaces.TaskManipulation(robot)
  taskmanip.CloseFingers()
  robot.WaitForController(0)
  robot.Grab(cube)
  
  # Move to the retreat pose
  Tretreat = np.array(Tgrasp)
  Tretreat[2,3] += 0.1
  axes.append( orpy.misc.DrawAxes(env, Tretreat, dist=0.05) )
  qretreat = None
  ikparam = orpy.IkParameterization(Tretreat, iktype)
  solutions = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
  if len(solutions) == 0:
    raise Exception('Failed to find a valid IK for the retreat pose')
  qretreat = solutions[0]
  traj = plan_to_joint_values(qretreat)
  controller = robot.GetController()
  controller.SetPath(traj)
  robot.WaitForController(0)
  
  # Move to the placing pose
  base_cube = env.GetKinBody('cube02')
  aabb = base_cube.ComputeAABB()
  Tplace = np.array(Tgrasp)
  Tplace[:3,3] = aabb.pos()
  Tplace[2,3] += 2*aabb.extents()[2]
  axes.append( orpy.misc.DrawAxes(env, Tplace, dist=0.05) )
  qplace = None
  ikparam = orpy.IkParameterization(Tplace, iktype)
  solutions = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
  if len(solutions) == 0:
    raise Exception('Failed to find a valid IK for the placing pose')
  qplace = solutions[0]
  traj = plan_to_joint_values(qplace)
  controller = robot.GetController()
  controller.SetPath(traj)
  robot.WaitForController(0)
  # Release the cube
  taskmanip.ReleaseFingers()
  robot.WaitForController(0)
  robot.Release(cube)
  
  # Move back home
  traj = plan_to_joint_values(np.zeros(6))
  controller = robot.GetController()
  controller.SetPath(traj)
  robot.WaitForController(0)
  
  # Use for debugging
  IPython.embed()
  env.Reset()
  env.Destroy()
