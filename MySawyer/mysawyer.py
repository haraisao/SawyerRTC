#
#
#  Copyright (C) 2018 Isao Hara,AIST,JP
#  All rights reserved.
#  License: the MIT License
#

from __future__ import print_function
import os
import sys
import traceback
import rospy
import intera_interface
import numpy as np
import threading
import cv2
import cv_bridge

import tf
import JARA_ARM


#
#
from intera_core_msgs.msg import InteractionControlCommand
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
    InteractionOptions
)

from intera_motion_msgs.msg import TrajectoryOptions
from std_msgs.msg import String

from sensor_msgs.msg import Image

#
#
#
class MySawyer(object):
  #
  #  Init class
  def __init__(self, name='MySawyer', limb='right', anonymous=True, disable_signals=True, light=True, gripper_reverse=False):
    rospy.init_node(name, anonymous=anonymous, disable_signals=disable_signals)
   # rospy.sleep(1)
    #
    #
    self._limb=None
    self._head=None
    self._light=None
    self._head_display=None
    self._display=None
    self._cuff=None
    self._limits=None
    self._navigator=None

    self._init_nodes(limb,light)
    self._get_gripper(gripper_reverse)

    #
    # Default Variables
    self._home_pos=[0.0, -1.178, 0.0, 2.178, 0.0, 0.567, 3.313]
    self._init_pos=[0.0, -1.178, 0.0, 2.178, 0.0, 0.567, 3.313]
    self._default_pos=[0.0, -0.9, 0.0, 1.8, 0.0, -0.9, 0.0]
    self._motion_trajectory=None

    #self._joint_names=self._limb.joint_names()
    #self._velocity_limits=self._limits.joint_velocity_limits()

    #
    #  for motion controller 
    self._motions={}
    self._joint_positions={'home':self._home_pos,'init':self._init_pos, 'default':self._default_pos}
    self._index=0
    self._p_index=0
    self._is_recording=False
    self.max_record_time=30
    self._accuracy=0.01  # 0.05 this value use velocity control mode and position control mode
    self._recording_intval=0.5
    #
    # for velicity control mode
    self._running=True
    self._target=[0.0, -1.178, 0.0, 2.178, 0.0, 0.567, 3.313] ### initial position 
    self._target_motion=[]
    self._vmax=0.4
    self._vrate=2.0
    self._is_moving=False

    #
    # for interaction mode
    self._speed_ratio=0.1 # 0.001 -- 1.0
    self._max_speed_ratio=0.5 # 0.001 -- 1.0
    self._max_accel_ratio=0.5 # 0.001 -- 1.0
    self._trajType='JOINT' # 'JOINT' ot 'CARTESIAN'
    self._interaction_active=True 
    self._K_impedance=[1300.0,1300.0, 1300.0, 30.0, 30.0, 30.0]
    self._max_impedance=[1,1,1,1,1,1]
    self._interaction_control_mode=[1,1,1,1,1,1]
    self._interaction_frame=[0,0,0,1,0,0,0]
    self._in_endpoint_frame=False
    self._endpoint_name='right_hand'
    self._force_command=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self._K_nullspace=[5.0, 10.0, 5.0, 10.0, 5.0, 10.0, 5.0]
    self._disable_damping_in_force_control=False
    self._disable_reference_resetting=False
    self._rotations_for_constrained_zeroG=False
    self._timeout=None

    # for Cartesian Pose base motion
    self._in_tip_frame=False
    self._tip_name='right_hand'
    self._linear_speed=0.6     # m/s
    self._linear_accel=0.6     # m/s/s
    self._rotational_speed=1.57  # rad/s
    self._rotational_accel=1.57  # rad/s/s

    ## for event handlers
    self.ok_id=None
    self.show_id=None
    self.back_id=None

    #
    # for RTC
    self._is_pause=False


  #
  #
  def _init_nodes(self, limb, light):
    try:
      self._limb=intera_interface.Limb(limb)

      self._head=intera_interface.Head()

      self._light=SawyerLight(light)
    
      #self._head_display=intera_interface.HeadDisplay()
      self._display=SawyerDisplay()
      self._cuff=intera_interface.Cuff()

      self._limits=intera_interface.JointLimits()
      self._navigator=intera_interface.Navigator()

      self._joint_names=self._limb.joint_names()
      self._velocity_limits=self._limits.joint_velocity_limits()

      self._stop_cmd={}
      for i,name in enumerate(self._joint_names):
        self._stop_cmd[name]=0.0
    except:
      print("Warning caught exception...")
      traceback.print_exc()
      pass

  #
  3
  def _get_gripper(self, gripper_reverse):
    try:
      self._gripper=intera_interface.get_current_gripper_interface()
      self._is_clicksmart = isinstance(self._gripper, intera_interface.SimpleClickSmartGripper)
      self._gripper_reverse=gripper_reverse
      if self._is_clicksmart:
        if self._gripper.needs_init():
          self._gripper.initialize()

        _signals=self._gripper.get_ee_signals()

        if 'grip' in _signals:
          self._gripper_type='grip'
        elif 'vacuumOn' in _signals:
          self._gripper_type='vacuum'
        else:
          self._gripper_type='unknown'
      else:
        if not (self._gripper.is_calibrated() or
                self._gripper.calibrate() == True):
          raise
    except:
      self._gripper=None
      self._is_clicksmart=False
      self._gripper_type=None
      self._gripper_reverse=None
      
  #
  #
  def activate(self):
    #
    #  Enable Robot
    self._rs=intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    self._init_state=self._rs.state().enabled
    self._rs.enable()

    #
    #  current positions
    self._angles=self._limb.joint_angles()
    self._pose=self._limb.endpoint_pose()
 
    #
    #
    self._limb.set_joint_position_speed(self._speed_ratio)

    #
    # LED white ON
    self._light.head_on()
    self.mkRosPorts()
    self.set_record()

  #
  #
  def mkRosPorts(self):
    self._sub=dict()
    self._pub=dict()

    self._sub['target_joint_pos']=rospy.Subscriber('target_joint_pos', String,self.set_target_joint_pos)

    self._pub['current_joint_pos']=rospy.Publisher('current_joint_pos', String,queue_size=1)
    self._pub['target_joint_pos']=rospy.Publisher('target_joint_pos', String,queue_size=1)
    self._pub['image']=rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)

  #
  #
  def set_motion_sequencer(self):
    self.set_subscriber('current_joint_pos', self.set_next_target, String)
  
  #
  #
  def set_subscriber(self, name, func, arg_type=String):
    if name in self._sub and self._sub[name]: self._sub[name].unregister()
    self._sub[name]=rospy.Subscriber(name, arg_type,func)

  def unset_subscriber(self, name):
    if name in self._sub and self._sub[name]: self._sub[name].unregister()
    self._sub[name]=None
  #
  #
  def enable(self):
    self._rs.enable()
  #
  #
  def state(self):
    print(self._rs.state())
  #
  #
  def reset(self):
    self._rs.reset()
  #
  #
  def disable(self):
    self._rs.disable()
  #
  #
  def stop(self):
    self._rs.stop()

  #
  #
  def exit_control_mode(self):
    self._limb.exit_control_mode()

  #
  #
  def update_pose(self):
    self._angles=self._limb.joint_angles()
    self._pose=self._limb.endpoint_pose()
 
  #
  #
  def init_pos(self, use_motion_ctrl=True):
    if use_motion_ctrl:
      self.move_to([self._init_pos])
    else:
      self._light.head_green()
      self._limb.move_to_neutral(speed=self._speed_ratio)
      self.update_pose()
      self._light.head_on()
  #
  #
  def set_speed(self, rate=0.3):
    self._speed_ratio=rate
    self._limb.set_joint_position_speed(rate)

  #
  #
  def print_joiint_pos(self, dtime=5.0, intval=0.1):
    end_time = rospy.Time.now() + rospy.Duration(dtime) 
    while rospy.Time.now() < end_time:
      if rospy.is_shutdown() : break
      print(self._limb.endpoint_pose())
      rospy.sleep(intval)

  ##############################################
  # Joint Position Control (Depreciated for Intera 5.2 and beyond)
  def move_joints(self, pos):
    self._limb.set_joint_position_speed(self._speed_ratio)
    self._light.head_green()
    self._limb.move_to_joint_positions(pos)
    self.update_pose()
    self._light.head_on()
  #
  #
  def move_cart(self, x_dist, y_dist, z_dist):
    self._limb.set_joint_position_speed(self._speed_ratio)
    self._pose=self.endpoint_pose()
    self._pose.position.x += x_dist
    self._pose.position.y += y_dist
    self._pose.position.z += z_dist
    self.move_joints(self._limb.ik_request(self._pose))
  #
  #
  def record_motion(self, name=None, dtime=0, intval=1.0):
    if not name :
      name=self.mk_motion_name()
      self._index += 1

    if dtime <= 0:
      dtime=self.max_record_time

    print ("Start Recording:", name)
    self._light.head_blue()

    self._motions[name]=[]
    self._is_recording=True
    end_time = rospy.Time.now() + rospy.Duration(dtime) 

    while (rospy.Time.now() < end_time) and self._is_recording :
      if rospy.is_shutdown() : break
      self._motions[name].append(self._limb.joint_angles())
      rospy.sleep(intval)

    print ("End Recording: record ", len(self._motions[name]), " points")
    self._is_recording=False
    self._light.head_on()

  #
  #
  def mk_motion_name(self):
    name = 'Motion_' + str(self._index)
    while name in self._motions:
      self._index += 1 
      name = 'Motion_' + str(self._index)
    return name

  #
  #  Record positions
  def record_pos(self,val):
    if val :
      self._light.head_yellow()
      name = 'P_' + str(self._p_index)
      while name in self._joint_positions:
        self._p_index += 1 
        name = 'P_' + str(self._p_index)
      self._joint_positions[name]=self._limb.joint_ordered_angles()
      self._light.head_on()

  #
  #  Motion Recorder Event Handleer(Start)
  def start_record(self, value):
     if value:
         print('Start..')
         self.record_motion(None, 0, self._recording_intval)
  #
  #  Motion Recorder Event Handler(Stop)
  def stop_record(self, value):
     if value:
         print('Stop..')
         self._is_recording=False
  #
  #  set Event Handlers
  def set_record(self):
     print ("Register callbacks")
     self.ok_id=self._navigator.register_callback(self.start_record, 'right_button_ok')
     self.back_id=self._navigator.register_callback(self.stop_record, 'right_button_back')
     self.square_id=self._navigator.register_callback(self.record_pos, 'right_button_square')
     self.show_id=self._navigator.register_callback(self.unset_record, 'right_button_show')
  #
  # unset Event Handlers
  def unset_record(self, value=0):
    if value and self.ok_id :
      print ("Unregister all callbacks")
      if self._navigator.deregister_callback(self.ok_id) : self.ok_id=None
      #if self._navigator.deregister_callback(self.show_id) : self.show_id=None
      if self._navigator.deregister_callback(self.back_id) : self.back_id=None
      if self._navigator.deregister_callback(self.square_id) : self.square_id=None
  
  def gripper_state(self):
    if self.is_gripping :
      return 1
    else:
      return 0
  #######################################################
  #
  # For Joint Position mode (before SDK-5.2)
  def play_motion(self, name, intval=0.0):
    self._limb.set_joint_position_speed(self._speed_ratio)
    self._light.head_green()
    for pos in self._motions[name]:
      if rospy.is_shutdown() :
        self._light.head_red()
        return
      # 
      self._limb.move_to_joint_positions(pos, threshold=self._accuracy)
      if intval > 0: rospy.sleep(intval)
    self._light.head_on()
  #
  #
  def play_motion_seq(self, names):
    self._limb.set_joint_position_speed(self._speed_ratio)
    self._light.head_green()
    for name in names:
      for pos in self._motions[name]:
        if rospy.is_shutdown() :
          self._light.head_red()
          return
        self._limb.move_to_joint_positions(pos)
    self._light.head_on()
  ###############################################  
  #
  #
  def list_motions(self):
    print(self._motions.keys())
  #
  #
  def joint_pos_d2l(self, pos):
    return map(lambda x: pos[x], self._joint_names)

  def convert_motion(self, name):
    return map(lambda x: self.joint_pos_d2l(x), self._motions[name])
  #
  #
  def save_motion(self, name):
    with open("motions/"+name+".jpos", mode="w") as f:
      for pos in self._motions[name]:
        f.write(str(pos))
        f.write("\n")
  #
  #
  def load_motion(self, name):
    self._motions[name]=[]
    with open("motions/"+name+".jpos") as f:
      motion=f.readlines()
    for p in motion:
      self._motions[name].append( eval(p) )
  #
  #
  def get_joint_positions(self, name):
    if type(name) == str:
      if name in self._joint_positions:
        target_joints=self._joint_positions[name]
      else:
        print("Invalid position name")
        target_joints=None
    elif len(name) == 7:
      target_joints=name
    return  target_joints
  ####################################
  #
  #  Move Motion
  def move_to(self, name=None, tout=None, with_in_contact=False, wait_for_result=True):
    #
    # for Motion Controller Interface
    if type(name) == str and name in self._motions:
      waypoints=self.convert_motion(name)
    elif type(name) == list:
      waypoints=name
    else:
      print("Invalid motion name")
      return None

    self._motion_trajectory=MotionTrajectory(limb=self._limb)

    _wpt_opts=MotionWaypointOptions(max_joint_speed_ratio=self._max_speed_ratio,
                                       max_joint_accel=self._max_accel_ratio)
    _waypoint=MotionWaypoint(options=_wpt_opts, limb=self._limb)
    #
    # set current joint position...
    _waypoint.set_joint_angles(joint_angles=self._limb.joint_ordered_angles())
    self._motion_trajectory.append_waypoint(_waypoint.to_msg())
    #
    # set target joint position...
    for pos in waypoints:
      if type(pos) == str:
        if pos in self._joint_positions:
          pos=self._joint_positions[pos]
          _waypoint.set_joint_angles(joint_angles=pos)
          self._motion_trajectory.append_waypoint(_waypoint.to_msg())
      else:
        _waypoint.set_joint_angles(joint_angles=pos)
        self._motion_trajectory.append_waypoint(_waypoint.to_msg())
    #
    #
    if with_in_contact :
      opts=self.get_in_contact_opts()
      if opts :
        self._motion_trajectory.set_trajectory_options(opts)
    #
    # run motion...
    self._light.head_green()
    result=self._motion_trajectory.send_trajectory(wait_for_result=wait_for_result,timeout=tout)
    #
    #
    if result is None:
      self._light.head_yellow()
      print("Trajectory FAILED to send")
      return None
    #
    #
    if not wait_for_result : return True
    #
    if result.result: self._light.head_on()
    else: self._light.head_red()
    #
    #
    self._motion_trajectory=None
    return result.result
  #
  #  Move in Certecian Mode
  def cart_move_to(self, target_pos, tout=None, relative_mode=False,  wait_for_result=True):
    #
    # for Motion Controller Interface
    _trajectory_opts=TrajectoryOptions()
    _trajectory_opts.interpolation_type=TrajectoryOptions.CARTESIAN

    #
    self._motion_trajectory=MotionTrajectory(trajectory_options=_trajectory_opts, limb=self._limb)
    #
    # set Waypoint Options
    _wpt_opts=MotionWaypointOptions(max_linear_speed=self._linear_speed,
                                       max_linear_accel=self._linear_accel,
                                       max_rotational_speed=self._rotational_speed,
                                       max_rotational_accel=self._rotational_accel,
                                       max_joint_speed_ratio=1.0)
    _waypoint=MotionWaypoint(options=_wpt_opts, limb=self._limb)

    #
    endpoint_state=self._limb.tip_state(self._tip_name)
    pose=endpoint_state.pose

    ########################################
    #  set target position

    if relative_mode:
      # relative position : target_pos -> x, y, z, roll, pitch, yew
      trans = PyKDL.Vector(target_pos[0],target_pos[1],target_pos[2])
      rot = PyKDL.Rotation.RPY(target_pos[3], target_pos[4],target_pos[5])
      f2 = PyKDL.Frame(rot, trans)

      if self._in_tip_frame:
        # endpoint's cartesian systems
        pose=posemath.toMsg(posemath.fromMsg(pose) * f2)
      else:
        # base's cartesian systems
        pose=posemath.toMsg(f2 * posemath.fromMsg(pose))
    else:
      #  global position : x, y, z, rx, ry, rz, rw
      pose.position.x=target_pos[0]
      pose.position.y=target_pos[1]
      pose.position.z=target_pos[2]
      pose.orientation.x=target_pos[3]
      pose.orientation.y=target_pos[4]
      pose.orientation.z=target_pos[5]
      pose.orientation.w=target_pos[6]
    #
    ###########################################
    # set target position.
    poseStamped=PoseStamped()
    poseStamped.pose=pose
    _waypoint.set_cartesian_pose(poseStamped, self._tip_name, [])
    self._motion_trajectory.append_waypoint(_waypoint.to_msg())
    #
    # run motion...
    self._light.head_green()
    result=self._motion_trajectory.send_trajectory( wait_for_result=wait_for_result,timeout=tout)

    #
    if result is None:
      self._light.head_yellow()
      print("Trajectory FAILED to send")
      return None
    #
    if not wait_for_result : return True
    #
    if result.result: self._light.head_on()
    else: self._light.head_red()
    #
    self._motion_trajectory=None
    return result.result

  #
  # stop motion...
  def stop_trajectory(self):
    if self._motion_trajectory :
      self._motion_trajectory.stop_trajectory()
  #
  #  set Interaction control
  def set_interaction_params(self):
    interaction_options = InteractionOptions()
    interaction_options.set_interaction_control_active(self._interaction_active)
    interaction_options.set_K_impedance(self._K_impedance)
    interaction_options.set_max_impedance(self._max_impedance)
    interaction_options.set_interaction_control_mode(self._interaction_control_mode)
    interaction_options.set_in_endpoint_frame(self._in_endpoint_frame)
    interaction_options.set_force_command(self._force_command)
    interaction_options.set_K_nullspace(self._K_nullspace)
    interaction_options.set_endpoint_name(self._endpoint_name)

    if len(self._interaction_frame) == 7:
      quat_sum_square = self._interaction_frame[3]*self._interaction_frame[3] + self._interaction_frame[4]*self._interaction_frame[4] + self._interaction_frame[5]*self._interaction_frame[5] + self._interaction_frame[6]*self._interaction_frame[6]

      if quat_sum_square  < 1.0 + 1e-7 and quat_sum_square > 1.0 - 1e-7:
        interaction_frame = Pose()
        interaction_frame.position.x = self._interaction_frame[0]
        interaction_frame.position.y = self._interaction_frame[1]
        interaction_frame.position.z = self._interaction_frame[2]
        interaction_frame.orientation.w = self._interaction_frame[3]
        interaction_frame.orientation.x = self._interaction_frame[4]
        interaction_frame.orientation.y = self._interaction_frame[5]
        interaction_frame.orientation.z = self._interaction_frame[6]
        interaction_options.set_interaction_frame(interaction_frame)
      else:
        print('Invalid input to quaternion! The quaternion must be a unit quaternion!')
        return None

    else:
        print('Invalid input to interaction_frame!')
        return None

    interaction_options.set_disable_damping_in_force_control(self._disable_damping_in_force_control)
    interaction_options.set_disable_reference_resetting(self._disable_reference_resetting)
    interaction_options.set_rotations_for_constrained_zeroG(self._rotations_for_constrained_zeroG)
    return interaction_options

  #
  #
  def set_interaction_mode(self):
    pub = rospy.Publisher('/robot/limb/right/interaction_control_command', InteractionControlCommand, queue_size = 1)
    interaction_options = self.set_interaction_params()
    if interaction_options:
      msg=interaction_options.to_msg()
      pub.publish(msg)

  #
  # 
  def get_in_contact_opts(self):
    interaction_options = self.set_interaction_params()
    if interaction_options:
      trajectory_options = TrajectoryOptions()
      trajectory_options.interaction_control = True
      trajectory_options.interpolation_type = self._trajType
      trajectory_options.interaction_params = interaction_options.to_msg()
      return trajectory_options
    else:
      return None
  ##############################################################
  #
  #  Open the gripper
  def gripper_open(self):
    if self._gripper and self._gripper.is_ready():
      if self._is_clicksmart:
        if 'grip' in self._gripper.get_ee_signals() :
          self._gripper.set_ee_signal_value('grip', self._gripper_reverse)
        else:
          return None
      else:
        self._gripper.open()
    return True
  #
  #  Close the gripper
  def gripper_close(self):
    if self._gripper and self._gripper.is_ready():
      if self._is_clicksmart:
        if 'grip' in self._gripper.get_ee_signals() :
          print(self._gripper_reverse)
          self._gripper.set_ee_signal_value('grip', not self._gripper_reverse)
        else:
          return None
      else:
        self._gripper.close()
    return True

  #  Grippper vacuum: True:off, False:on
  def gripper_vacuum(self, stat=True):
    if self._gripper and self._gripper.is_ready():
      if self._is_clicksmart:
        if 'vacuumOn' in self._gripper.get_ee_signals() :
          self._gripper.set_ee_signal_value('vacuumOn', stat)
          return self._gripper.get_ee_signal_value('vacuumOn')
    return None

  def is_gripping(self):
    if self._gripper and self._gripper.is_ready():
      if self._is_clicksmart:
        if self._gripper.get_ee_signal_value('grip') is None:
          return not self._gripper.get_ee_signal_value('vacuumOn')
        else:
          return self._gripper.get_ee_signal_value('grip')
      else:
        return self._gripper.is_gripping()
    return None

  ###############################################################
  #
  #   stop the thread of velocity control loop 
  def stop_vctrl(self):
    self._running=False
    self._vctrl_th.join()
    self._vctrl_th=None

  #
  # start vctrl_loop with Thread
  def start_vctrl(self, hz=100):
    self._vctrl_th=threading.Thread(target=self.vctrl_loop, args=(hz,self.report,))
    self._vctrl_th.start()
  #
  # velocity control mode, one cycle
  def _vctrl_one_cycle(self, func=None):
    cmd={}
    cur=self._limb.joint_ordered_angles()
    dv=np.array(self._target) - np.array(cur)

    if np.linalg.norm(dv) < self._accuracy:
      if func:
        func(self)
      self._is_moving=False
    else:
      self._is_moving=True
      vels = map(lambda x: x*self._vrate, dv)  
      for i,name in enumerate(self._joint_names):
        cmd[name]=maxmin(vels[i] , self._velocity_limits[name]*self._vmax, -self._velocity_limits[name]*self._vmax)
      self._limb.set_joint_velocities(cmd)
  #
  #  velocity control loop
  def vctrl_loop(self, hz, func=None):
    rate=rospy.Rate(hz)
    self._running=True

    while self._running and (not rospy.is_shutdown()) :
      cuff_state=self._cuff.cuff_button()
      if cuff_state :
        self.set_target()
      elif self._limb.has_collided() :
        self.set_target()
      else:
        self._vctrl_one_cycle(func)
      rate.sleep()
    self._limb.exit_control_mode()
    print("Terminated")

  #
  # callback function for Subscriber
  def set_target_joint_pos(self, data):
    self._target=eval(data.data)

  #
  # 
  def set_next_target(self, data):
    try:
      self.unset_subscriber('current_joint_pos')
      next_target=self._target_motion.pop(0)
      self.set_target(next_target)
      self.set_motion_sequencer()
    except:
      self.unset_subscriber('current_joint_pos')
      pass
  #
  #  Publish current position
  def report(self,val):
    cur=self._limb.joint_ordered_angles()
    self._pub['current_joint_pos'].publish(str(cur)) 

  #
  # Set target joint positions (Publish target joint positions)
  def set_target(self, val=None, relative=False):
    if val is None:
      val=self._limb.joint_ordered_angles()
    if type(val) is str:
      val=self._joint_positions[val]
    elif relative:
      if len(self._target) != len(val) :
        print("Dimension mismatch")
        return
      for i,v in enumerate(self._target):
        val[i]=v + val[i]
    
    self._pub['target_joint_pos'].publish(str(val)) 

  def set_target_seq(self, targets):
    self._target_motion=targets
    self.set_next_target('star')

  def show_positions(self):
    print(self._joint_positions.keys())

  #
  def set_cart_target(self, x,y,z,roll=9,pitch=0,yew=0, in_tip_frame=True):
    #pose = self.convert_Cart2Joint(x,y,z, relativa, end_point)
    pos=self.calc_cart_move2joints(x,y,z,roll, pitch,yew, in_tip_frame=in_tip_frame)
    val=self.joint_pos_d2l(pos)
    self._pub['target_joint_pos'].publish(str(val)) 


  #
  #  Set movement of target joint posistions
  def move_joint(self, idxs, vals):
    for i,v in enumerate(idxs) :
      self._target[v] += vals[i]
    self._pub['target_joint_pos'].publish(str(self._target)) 

  #
  # end_point should be 'right_hand' or 'right_arm_base_link'.
  def convert_Cart2Joint(self, x,y,z, relative=False, end_point='right_hand'):
    _pose=self.endpoint_pose()
    if relative:
      _pose.position.x += x
      _pose.position.y += y
      _pose.position.z += z
    else:
      _pose.position.x = x
      _pose.position.y = y
      _pose.position.z = z
    return self._limb.ik_request(_pose, end_point=end_point)
  #
  #
  def convert_Joint2Cart(self, pos):
    _pos=self._limb.joint_angles()
    for i,name in enumerate(self._joint_names):
      _pos[name]=pos[i]
    return self._limb.joint_angles_to_cartesian_pose(_pos)

  def endpoint_pose(self):
    return self._limb.tip_state('right_hand').pose
    #return self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles())

  def calc_relative_pose(self, x,y,z,roll=0,pitch=0,yew=0, in_tip_frame=True):
    _pose=self.endpoint_pose()
    ########################################
    #  set target position
    trans = PyKDL.Vector(x,y,z)
    rot = PyKDL.Rotation.RPY(roll,pitch,yew)
    f2 = PyKDL.Frame(rot, trans)

    if in_tip_frame:
      # endpoint's cartesian systems
      _pose=posemath.toMsg(posemath.fromMsg(_pose) * f2)
    else:
      # base's cartesian systems
      _pose=posemath.toMsg(f2 * posemath.fromMsg(_pose))

    return _pose

  def calc_cart_move2joints(self, x,y,z,roll=0,pitch=0,yew=0, in_tip_frame=True):
    _pose=self.calc_relative_pose(x,y,z,roll,pitch,yew, in_tip_frame)
    return self._limb.ik_request(_pose)

  ########################################################################
  #
  #  onExecuted
  #
  def onExecute(self):
    cuff_state=self._cuff.cuff_button()
    if self._is_pause :
      self._limb.set_joint_velocities(self._stop_cmd)
    elif cuff_state :
      self.set_target()
    elif self._limb.has_collided() :
      self.set_target()
    else:
      self._vctrl_one_cycle(self.report)
  #
  # for RTC(Common)
  #
  def clearAlarms(self):
    print('No alerm..')
    return True

  def getActiveAlarm(self):
    res=[]
    return res

  def getFeedbackPosJoint(self):
    res=map(lambda x: np.rad2deg(x), self._limb.joint_ordered_angles())
    res.append(self.gripper_state())
    return res

#    manifactur: string
#    type: string
#    axisNum: ULONG
#    cmdCycle: ULONG
#    isGripper: boolea
  def getManipInfo(self):
    if self._gripper :
      res=['RethinkRobotics', 'Sawyer', 7, 100, True]
    else:
      res=['RethinkRobotics', 'Sawyer', 7, 100, False]
    return res

  def getSoftLimitJoint(self):
    return None

  def getState(self):
    stat=self._rs.state()
    if stat.enabled :
      res=0x01
      if self._is_moveing :
        res=0x01 | 0x02
    else:
      res=0x00
    return res

  def servoOFF(self):
    return None

  def servoON(self):
    return None

  def setSoftLimitJoint(self, softLimit):
    return None
  #
  # for RTC(Middle)
  #
  def openGripper(self):
    if self._gripper_type == 'vacuum':
      self.gripper_vacuum(False)
    else:
      self.gripper_open()
    return True

  def closeGripper(self):
    if self._gripper_type == 'vacuum':
      self.gripper_vacuum(True)
    else:
      self.gripper_close()
    return True

  def moveGripper(self, angleRatio):
    print('Move gripper is not supported')
    return None

  def getBaseOffset(self):
    return None

  def getFeedbackPosCartesian(self):
    return None

  def getMaxSpeedCartesian(self):
    return None

  def getMaxSpeedJoint(self):
    return None

  def getMinAccelTimeCartesian(self):
    return None

  def getMinAccelTimeJoint(self):
    return None

  def getSoftLimitCartesian(self):
    return None

  def moveLinearCartesianAbs(self, carPos, elbow, flag):
    return None

  def moveLinearCartesianRel(self, carPos, elbow, flag):
    return None

  def movePTPCartesianAbs(self, carPos, elbow, flag):
    mx=np.array(carPos)
    pos=mx[:,3]
    mx=np.vstack((mx, [0,0,0,1]))
    qtn=tf.transformations.quaternion_from_matrix(mx)

    pose=newPose(pos, qtn)
    pos=self._limb.ik_request(pose)
    self._target=self.joint_pos_d2l(pos)

    return None

  def movePTPCartesianRel(self, carPos, elbow, flag):
    ep=self.endpoint_pose()
    mx=np.array(carPos)
    pos=mx[:,3]
    mx=np.vstack((mx, [0,0,0,1]))
    qtn=tf.transformations.quaternion_from_matrix(mx)

    dp=newPose(pos, qtn)
    pose=addPose(ep, dp)
    pos=self._limb.ik_request(pose)
    self._target=self.joint_pos_d2l(pos)

    return None

  def movePTPJointAbs(self, jointPoints):
    if len(jointPoints) >= 7:
      pos=map(lambda x: np.deg2rad(x), jointPoints[:7])
      self._target=pos
      return True
    else:
      return False

  def movePTPJointRel(self, jointPoints):
    if len(jointPoints) >= 7:
      pos=map(lambda x: np.deg2rad(x), jointPoints[:7])
      self._target=np.array(self._target) + np.array(pos)
      return True
    else:
      return False

  def pause_motion(self):
    self._is_pause=True
    self._limb.set_joint_velocities(self._stop_cmd)
    return True

  def resume_motion(self):
    self._is_pause=False
    return True

  def stop_motion(self):
    self.set_target()
    return True

  def setAccelTimeCartesian(self, aclTime):
    return None

  def setAccelTimeJoint(self, aclTime):
    return None

  def setBaseOffset(self, offset):
    return None

  def setControlPointOffset(self, offset):
    return None

  def setMaxSpeedCartesian(self, sp_trans, sp_rot):
    return None

  def setMaxSpeedJoint(self, speed):
    return None

  def setMinAccelTimeCartesian(self, aclTime):
    return None

  def setMinAccelTimeJoint(self, aclTime):
    return None

  def setSoftLimitCartesian(self, xLimit, yLimit, zLimit):
    return None

  def setSpeedCartesian(self, spdRatio):
    return None

  def setSpeedJoint(self, spdRatio):
    self.set_speed(spdRatio)
    return True

  def moveCircularCartesianAbs(self, carPointR, carPointT):
    return None

  def moveCircularCartesianRel(self, carPointR, carPointT):
    return None

  def setHome(self, jointPoint):
    self._home_pos=jointPoint
    return True

  def getHome(self):
    return self._home_pos

  def goHome(self):
    self._target=self._home_pos
    return True


########################################################################
#
#  LED Light of the Head
#
class SawyerLight(object):
  def __init__(self, enabled=True):
    self._light=intera_interface.Lights()
    self._enabled=enabled
  #
  # if you use with Gazebo simulator, you would be disable light...
  def enabled(self, val=True):
    self._enabled=val 
  #
  #  Right
  ###########################
  def head_yellow(self):
    if self._enabled:
      self._light.set_light_state('head_blue_light',False)
      self._light.set_light_state('head_red_light',True)
      self._light.set_light_state('head_green_light',True)
  #
  #
  def head_blue(self):
    if self._enabled:
      self._light.set_light_state('head_red_light',False)
      self._light.set_light_state('head_green_light',False)
      self._light.set_light_state('head_blue_light',True)
  #
  #
  def head_green(self):
    if self._enabled:
      self._light.set_light_state('head_red_light',False)
      self._light.set_light_state('head_blue_light',False)
      self._light.set_light_state('head_green_light',True)
  #
  #
  def head_red(self):
    if self._enabled:
      self._light.set_light_state('head_green_light',False)
      self._light.set_light_state('head_blue_light',False)
      self._light.set_light_state('head_red_light',True)
  #
  #  White
  def head_on(self):
    if self._enabled:
      self._light.set_light_state('head_red_light',True)
      self._light.set_light_state('head_green_light',True)
      self._light.set_light_state('head_blue_light',True)
  #
  # Turn off
  def head_off(self):
    if self._enabled:
      self._light.set_light_state('head_red_light',False)
      self._light.set_light_state('head_green_light',False)
      self._light.set_light_state('head_blue_light',False)

  ###########################

#########
import PIL.Image,PIL.ImageFont, PIL.ImageDraw

class SawyerDisplay(object):
  def __init__(self):
    self._image_pub=rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)

    self._fontname='/usr/share/fonts/truetype/takao-gothic/TakaoGothic.ttf'
    if not os.path.exists(self._fontname):
      self._fontname='/usr/share/fonts/opentype/NotoSansCJK-Regular.ttc'

    self._font24=PIL.ImageFont.truetype(self._fontname, 24, encoding='unic')
    self._font32=PIL.ImageFont.truetype(self._fontname, 32, encoding='unic')
    self._font42=PIL.ImageFont.truetype(self._fontname, 42, encoding='unic')
    self._font48=PIL.ImageFont.truetype(self._fontname, 48, encoding='unic')
    self._font64=PIL.ImageFont.truetype(self._fontname, 64, encoding='unic')
    self._font=self._font42

    self._image=self.mkImage()
    self._draw=None
    self._sdk_img='sawyer_sdk_research.png'

  def mkImage(self, val=128):
    img=np.full((600,1024,3), val, dtype=np.uint8)
    return img

  def mkPILImage(self, val=128):
    img=PIL.Image.new('RGB',(1024,600), color=(val,val,val))
    self._draw=PIL.ImageDraw.Draw(img)
    return img

  def clear(self):
    self._image=self.mkImage()
    self.update()

  def update(self):
    self.showImage()

  def convert2cv(self):
    if isinstance(self._image, PIL.Image.Image) :
      self._image=np.asarray(self._image)
      self._draw=None

  def convert2PIL(self):
    if self._draw is None:
      self._image=PIL.Image.fromarray(self._image)
      self._draw=PIL.ImageDraw.Draw(self._image)

  def drawText(self, txt, x, l, clear=False):
    if clear: self._image=self.mkPILImage()
    self.convert2PIL()
    pos=(20+x, l*50)
    self._draw.text(pos, txt, font=self._font, fill='#ffffff')
    self.update()

  def drawEllipse(self, param, fill=(255,0,0), outline=(0,0,0), clear=False):
    if clear: self._image=self.mkPILImage()
    self.convert2PIL()
    self._draw.ellipse(param, fill=fill, outline=outline)
    self.update()

  def putText(self, txt, x, l, clear=False):
    if clear: self._image=self.mkImage()
    self.convert2cv()

    pos=(20+x, l*50)
    cv2.putText(self._image, txt, pos, cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255,255,255), 2, cv2.LINE_AA)
    self.update()

  def showDefault(self):
    self.showImage(self._sdk_img)

  def showImage(self, img=None):
    if img is None:
      img=self._image
    elif type(img) is str:
      img=cv2.imread('images/'+img)

    if isinstance(img, PIL.Image.Image):
      img=np.asarray(img)

    if not rospy.is_shutdown():
      cv_img=cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
      self._image_pub.publish(cv_img)
      
####################################
#  Function....
def maxmin(v, mx, mn):
  return np.max([np.min([v,mx]), mn])

def newPose(p, qtn):
  res=Pose()
  Pose.position.x=p[0]
  Pose.position.y=p[1]
  Pose.position.z=p[2]
  Pose.orientation.x=qtn[0]
  Pose.orientation.y=qtn[1]
  Pose.orientation.z=qtn[2]
  Pose.orientation.w=qtn[3]
  return res

def addPose(p1, p2):
  res=Pose()
  Pose.position.x=p1.position.x+p2.position.x
  Pose.position.y=p1.position.y+p2.position.y
  Pose.position.z=p1.position.z+p2.position.z
  Pose.orientation.x=p1.orientation.x+p2.orientation.x
  Pose.orientation.y=p1.orientation.y+p2.orientation.y
  Pose.orientation.z=p1.orientation.z+p2.orientation.z
  Pose.orientation.w=p1.orientation.w+p2.orientation.w
  return res

