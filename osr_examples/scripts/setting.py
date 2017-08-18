#!/usr/bin/env python
import rospy
import criros
import collections
import copy
import time
import numpy as np
import matplotlib.pyplot as plt
import openravepy as orpy
import tf.transformations as tr
from osr_openrave import kinematics, planning
from geometry_msgs.msg import WrenchStamped
from osr_control.controllers import GripperController, JointTrajectoryController, JointPositionController

class FTsensor(object):
  queue_len = 10
  
  def __init__(self, namespace='', timeout=3.0):
    ns = criros.utils.solve_namespace(namespace)
    self.raw_msg = None
    self.rate = 250
    self.wrench_rate = 250
    self.wrench_filter = criros.filters.ButterLowPass(2.5, self.rate, 2)
    self.wrench_window = int(self.wrench_rate)
    assert( self.wrench_window >= 5)
    self.wrench_queue = collections.deque(maxlen=self.wrench_window)
    rospy.Subscriber('%sft_sensor/raw' % ns, WrenchStamped, self.cb_raw)
    initime = rospy.get_time()
    if not criros.utils.wait_for(lambda : self.raw_msg is not None, timeout=timeout):
      rospy.logerr('Timed out waiting for {0}ft_sensor/raw topic'.format(ns))
      return
    rospy.loginfo('FTSensor successfully initialized')
    
  def add_wrench_observation(self,wrench):
    self.wrench_queue.append(np.array(wrench))
    
  def cb_raw(self, msg):
    self.raw_msg = copy.deepcopy(msg)
    self.add_wrench_observation(criros.conversions.from_wrench(self.raw_msg.wrench))
  
  #function to filter out high frequency signal  
  def get_filtered_wrench(self):
    if len(self.wrench_queue) < self.wrench_window:
      return None    
    wrench_filtered = self.wrench_filter(np.array(self.wrench_queue))
    return wrench_filtered[-1,:]
    
class Hybrid_Controller(object):
  def __init__(self):
    #Load the environment
    self.env = orpy.Environment()
    if not self.env.Load('worlds/cubes_task.env.xml'):
      rospy.logerr('Failed to load the world. Did you run: catkin_make install?')
      exit(1)
    self.env.SetDefaultViewer()
    Tcamera = tr.euler_matrix(*np.deg2rad([-120, 13, 135]))
    Tcamera[:3,3] = [1, 1, 2]
    self.env.GetViewer().SetCamera(Tcamera)
    self.robot = self.env.GetRobot('robot')
    self.manipulator = self.robot.SetActiveManipulator('gripper')
    self.robot.SetActiveDOFs(self.manipulator.GetArmIndices())
    self.taskmanip = orpy.interfaces.TaskManipulation(self.robot)
    self.js_rate = criros.utils.read_parameter('/joint_state_controller/publish_rate', 250.0) #read publish rate if it does exist, otherwise set publish rate
    self.T = 1. / self.js_rate
    self.rate = rospy.Rate(self.js_rate)
    # Scale down the velocity and acceleration limits
    self.robot.SetDOFVelocityLimits(self.robot.GetDOFVelocityLimits()*0.4)
    self.robot.SetDOFAccelerationLimits(self.robot.GetDOFAccelerationLimits()*0.2)
    # Connect to the hardware interfaces
    self.joint_controller = JointPositionController()
    self.gripper_controller = GripperController()
    self.ft_sensor = FTsensor()
    
     # Load IKFast. Generate it if needed
    self.iktype = orpy.IkParameterization.Type.Transform6D
    success = kinematics.load_ikfast(self.robot, self.iktype)
    if not success:
      rospy.logerr('Failed to load IKFast for {0}, manipulator: {1}'.format(self.robot.GetName(), self.manipulator.GetName()))
      IPython.embed()
      exit(1)
    # Load links stats for finding closest IK solutions
    statsmodel = orpy.databases.linkstatistics.LinkStatisticsModel(self.robot)
    if not statsmodel.load():
      rospy.loginfo('Generating LinkStatistics database. It will take around 1 minute...')
      statsmodel.autogenerate()
    statsmodel.setRobotWeights()
    statsmodel.setRobotResolutions(xyzdelta=0.01)
  
  
  def grasp_box_and_retreat(self):
    # Find a valid IK solution for grasping the cube
    cube = self.env.GetKinBody('cube02')
    cube_centroid = cube.ComputeAABB().pos()
    Tgrasp = tr.euler_matrix(0, np.pi, 0)
    Tgrasp[:3,3] = cube_centroid
    qgrasp = kinematics.find_closest_iksolution(self.robot, Tgrasp, self.iktype)
    axes = []
    axes.append( orpy.misc.DrawAxes(self.env, Tgrasp, dist=0.05) )
    
    # Move to the grasping pose
    traj = planning.plan_to_joint_configuration(self.robot, qgrasp)
    #ros_traj = criros.conversions.ros_trajectory_from_openrave(self.robot.GetName(), traj)
    self.robot.WaitForController(0)
    traj_spec = traj.GetConfigurationSpecification()
    traj_duration = traj.GetDuration()
    step_num = traj_duration // self.T
    for t in np.append(np.arange(0,traj_duration,self.T),traj_duration):
      self.joint_controller.set_joint_positions(list(traj_spec.ExtractJointValues(traj.Sample(t),self.robot, self.manipulator.GetArmIndices())))
      self.robot.SetDOFValues(list(self.joint_controller.get_joint_positions()),self.manipulator.GetArmIndices())
      self.rate.sleep()
    # Grasp the cube
    self.gripper_controller.command(0.05)
    self.taskmanip.CloseFingers()
    self.gripper_controller.wait()
    self.robot.WaitForController(0)
    self.robot.Grab(cube)
    self.gripper_controller.grab('{0}::link'.format(cube.GetName()))
    
    # Find a valid IK solution for the retreat pose
    Tretreat = np.array(Tgrasp)
    Tretreat[2,3] += 0.05
    Tretreat[0,3] -= 0.1
    axes.append( orpy.misc.DrawAxes(self.env, Tretreat, dist=0.05) )
    qretreat = kinematics.find_closest_iksolution(self.robot, Tretreat, self.iktype)
    traj = planning.plan_to_joint_configuration(self.robot, qretreat)
    self.robot.WaitForController(0)
    traj_spec = traj.GetConfigurationSpecification()
    traj_duration = traj.GetDuration()
    step_num = traj_duration // self.T
    for t in np.append(np.arange(0,traj_duration,self.T),traj_duration):
      self.joint_controller.set_joint_positions(list(traj_spec.ExtractJointValues(traj.Sample(t),self.robot, self.manipulator.GetArmIndices())))
      self.robot.SetDOFValues(list(self.joint_controller.get_joint_positions()),self.manipulator.GetArmIndices())
      self.rate.sleep()
    time.sleep(2)
  def start_hybrid_controller(self):
    
    #moving the box down to the table surface until contact
    dt = 1. / self.js_rate
    Kf = 5000.
    Kp = np.array([1., 1., 1.]) * 1. 
    Kv = np.array([1., 1., 1.]) * 40     
    Fr = np.array([0., 0., -15])
    qc = self.joint_controller.get_joint_positions()
    Fe_prev = np.zeros(3)
    xf = np.zeros(3)
    dxf = np.zeros(3)
    rospy.loginfo('Going down')
    self.wrench_offset = self.ft_sensor.get_filtered_wrench()
    link_idx = [l.GetName() for l in self.robot.GetLinks()].index('robotiq_85_base_link')
    link_origin = self.robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]
    J = np.zeros((6,6))
    twist = np.zeros(6)
    while not rospy.is_shutdown():
      q_actual = self.joint_controller.get_joint_positions()
      self.robot.SetDOFValues(q_actual, self.manipulator.GetArmIndices())
      We = self.ft_sensor.get_filtered_wrench() - self.wrench_offset
      bTe = self.manipulator.GetEndEffectorTransform()
      bXeF = criros.spalg.force_frame_transform(bTe)
      Wb = np.dot(bXeF, We)
      Fb = -Wb[:3]
      Fr[0:2] = Fb[0:2]
      if np.linalg.norm(Fb) >= np.linalg.norm(Fr):
        rospy.loginfo('Surface contacted, preparing for sliding')
        break
      # Force PD compensator
      Fe = (Fr - Fb) / Kf
      dFe = (Fe - Fe_prev)
      Fe_prev = Fe
      dxf = (Kp*Fe + Kv*dFe) * dt
      xf += dxf
      twist[:3] = dxf
      # Velocity-based operational space controller
      J[:3,:] = self.robot.ComputeJacobianTranslation(link_idx, link_origin)[:,:6]
      J[3:,:] = self.robot.ComputeJacobianAxisAngle(link_idx)[:,:6]
      dqc = np.linalg.solve(J, twist)
      qc += dqc
      self.joint_controller.set_joint_positions(qc)
      self.rate.sleep()
      # Safety limits: displacement and max force
      if np.linalg.norm(xf) > 0.5:
        rospy.loginfo('Maximum displacement exceeded')
        break
      if np.linalg.norm(Fb) >= 20.:
        rospy.loginfo('Maximum force exceeded')
        break
    time.sleep(2)
    
    #start sliding
    
    vlin = 0.006         #linear velocity
    dr_slide = [1.,0.]   # sliding direction on x-y plane
    dt = 1./ self.js_rate  # time step  
    x0,y0,z0 = self.manipulator.GetEndEffectorTransform()[:3,3] #position of gripper at the starting point
    xr = [x0,y0,z0]
    timeout = 17         
    Kp_pos = np.array([1, 1, 0])          #proportion controller of position
    Kv_pos = np.array([0.05, 0.05, 0])    #derivative controller of position
    Kp_force = np.array([0, 0, 3.4e-5])   #proportion controller of force
    Kv_force = np.array([0, 0, 5e-6])     #derivative controller of force
    bTe = self.manipulator.GetEndEffectorTransform()
    bXeF = criros.spalg.force_frame_transform(bTe) # transformation matrix that convert wrench wrt body frame to  wrench in space frame
    We = self.ft_sensor.get_filtered_wrench() - self.wrench_offset
    Wb = np.dot(bXeF, We)                 #wrench (the environment inserts on the end effector) is represented in space frame
    Fe_prev = -Wb[:3]                     #extract force vector only, negative sign indicates this is the force inserted on the environment by the robot
    xe_prev = [x0, y0, z0]
    qc = self.joint_controller.get_joint_positions()
    force_data = list()     #data for graph
    time_data = list()
    initime = rospy.get_time()
    link_idx = [l.GetName() for l in self.robot.GetLinks()].index('robotiq_85_base_link')
    link_origin = self.robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]
    J = np.zeros((6,6))
    twist = np.zeros(6)
    rospy.loginfo('Sliding')
    while not rospy.is_shutdown() and (rospy.get_time() - initime) < timeout:
      xr[0] += dr_slide[0] * vlin * dt
      xr[1] += dr_slide[1] * vlin * dt
      q_actual = self.joint_controller.get_joint_positions()
      self.robot.SetDOFValues(q_actual, self.manipulator.GetArmIndices())
      # Transform wrench to the base_link frame
      We = self.ft_sensor.get_filtered_wrench() - self.wrench_offset
      bTe = self.manipulator.GetEndEffectorTransform()
      bXeF = criros.spalg.force_frame_transform(bTe)
      Wb = np.dot(bXeF, We)
      Fb = -Wb[:3]
      Fr = np.array([0., 0., -15])
      Fr[:2] = Fb[:2]
      Fe = Fr - Fb
      dFe = (Fe - Fe_prev) / dt
      Fe_prev = Fe
      dxf_force = (Kp_force*Fe + Kv_force*dFe) * dt
      xb = self.manipulator.GetEndEffectorTransform()[:3, 3]
      xe = xr - xb
      dxe = (xe - xe_prev)
      xe_prev = xe
      dxf_pos = (Kp_pos*xe + Kv_pos*dxe) * dt
      dxf = dxf_force + dxf_pos
      twist[:3] = dxf
      # velocity-space IK
      J[:3,:] = self.robot.ComputeJacobianTranslation(link_idx, link_origin)[:,:6]
      J[3:,:] = self.robot.ComputeJacobianAxisAngle(link_idx)[:,:6]
      dqc = np.linalg.solve(J, twist)
      qc += dqc
      force_data.append(Fb[2])
      time_data.append(rospy.get_time() - initime)
      self.joint_controller.set_joint_positions(qc)
      self.rate.sleep()
    rospy.loginfo("Complete hybrid force control demo")
    plt.plot(time_data,force_data, label = "Controlled force in Z direction")
    plt.axhline(y=-15.0, xmin=0, xmax=15., linewidth=2, color = 'r',label = "reference line")
    plt.axis([0,15,-20,0])
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
    plt.show() 
    
  
