#!/usr/bin/env python
import criros
import numpy as np
import openravepy as orpy


def find_closest_iksolution(robot, T, iktype):
  ikparam = get_ik_parameterization(T, iktype)
  if ikparam is None:
    return None
  manipulator = robot.GetActiveManipulator()
  qseed = robot.GetActiveDOFValues()
  solutions = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
  weights = robot.GetDOFWeights(manipulator.GetArmIndices())
  solutions = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
  if len(solutions) > 0:
    distances = [sum(weights*(qseed-qsol)**2) for qsol in solutions]
    closest = np.argmin(distances)
    return solutions[closest]
  else:
    return None

def get_ik_parameterization(goal, iktype):
  target = goal
  if iktype == orpy.IkParameterizationType.TranslationDirection5D:
    if type(goal) is not orpy.Ray:
      target = criros.conversions.to_ray(goal)
  elif iktype == orpy.IkParameterizationType.Transform6D:
    if type(goal) is orpy.Ray:
      target = criros.conversions.from_ray(goal)
  else:
    return None
  return orpy.IkParameterization(target, iktype)

def load_ikfast(robot, iktype, autogenerate=True):
  ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=iktype)
  if not ikmodel.load() and autogenerate:
    print 'Generating IKFast {0}. It will take few minutes...'.format(iktype.name)
    ikmodel.autogenerate()
    print 'IKFast {0} has been successfully generated'.format(iktype.name)
  return ikmodel.load()
