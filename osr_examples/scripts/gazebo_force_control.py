#!/usr/bin/env python
import setting
import rospy
if __name__ == '__main__':
  rospy.init_node('gazebo_force_control')
  hybrid_controller = setting.Hybrid_Controller()
  hybrid_controller.grasp_box_and_retreat()
  hybrid_controller.start_hybrid_controller()
  
