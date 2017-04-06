#!/usr/bin/env python
import openravepy as orpy


def find_closest_iksolution(T):
  pass

def plan_to_joint_configuration(robot, qgoal, planner='birrt', max_planner_iterations=20,
                                  max_postprocessing_iterations=40):
  env = robot.GetEnv()
  rave_planner = orpy.RaveCreatePlanner(env, planner)
  params = orpy.Planner.PlannerParameters()
  params.SetRobotActiveJoints(robot)
  params.SetGoalConfig(qgoal)
  extra_parameters = """
  <_postprocessing planner="ParabolicSmoother">
    <_nmaxiterations>{0}</_nmaxiterations>
  </_postprocessing> 
  """.format(max_postprocessing_iterations)
  params.SetExtraParameters(extra_parameters)
  success = rave_planner.InitPlan(robot, params)
  if not success:
    return None
  # Plan a trajectory
  traj = orpy.RaveCreateTrajectory(env, '')
  status = rave_planner.PlanPath(traj)
  if status != orpy.PlannerStatus.HasSolution:
    return None
  return traj
