#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file StdManipulator.py
 @brief Manipulator controller with JARA Standard Interfaces
 @date $Date$

 @license the MIT License

 Copyright(C) 2018 Isao Hara,AIST,JP
 All rights reserved.

"""
from DataFlowRTC_Base import *
import numpy as np

#from MySawyer import *
import MySawyer


##
# @class SawyerRTC
# @brief Manipulator controller with JARA Standard Interfaces
# 
# 
class SawyerRTC(DataFlowRTC_Base):
  ##
  # @brief constructor
  # @param manager Maneger Object
  # 
  def __init__(self, manager):
    DataFlowRTC_Base.__init__(self, manager)

  ##
  #
  # The initialize action (on CREATED->ALIVE transition)
  # formaer rtc_init_entry() 
  # 
  # @return RTC::ReturnCode_t
  # 
  #
  def onInitialize(self):
    DataFlowRTC_Base.onInitialize(self)

    self.bindDataListener('joints')
    self.bindDataListener('grip')

    self._robot=None
    
    return RTC.RTC_OK
  ##
  #
  # The activated action (Active state)
  #
  # @param ec_id target ExecutionContext Id
  #
  # @return RTC::ReturnCode_t
  #
  #  
  def onActivated(self, ec_id):
    print (self.getInstanceName())
    self._robot=MySawyer.MySawyer(self.getInstanceName(), anonymous=False)
    self._robot.activate()
    self._robot._is_running=True
    self._manipCommon_service._robot=self._robot
    self._manipMiddle_service._robot=self._robot
    self._sawyerMiddle_service._robot=self._robot
  
    return RTC.RTC_OK
  
  ##
  #
  # The deactivated action (Active state exit action)
  # former rtc_active_exit()
  #
  # @param ec_id target ExecutionContext Id
  #
  # @return RTC::ReturnCode_t
  #
  #
  def onDeactivated(self, ec_id):
    if self._robot:
      self._robot._is_running=False
      self._robot.disable()
  
    return RTC.RTC_OK

  ##
  #
  # The execution action that is invoked periodically
  # former rtc_active_do()
  #
  # @param ec_id target ExecutionContext Id
  #
  # @return RTC::ReturnCode_t
  #
  #
  def onExecute(self, ec_id):
    if self._robot:
       
      self._robot._vmax=self._vmax[0]
      self._robot._vrate=self._vrate[0]
      self._robot._gripper_reverse=(self._gripper_reverse[0] == 1)

      self._robot.onExecute()

      data=RTC.TimedFloatSeq(new_Time(),[])

      data.data=self._robot.get_joint_angles()
      self._out_jointsOut.write(data)
  
      data.data=self._robot.get_joint_velocities()
      self._out_velocityOut.write(data)

    return RTC.RTC_OK
  
        #
        # Callback method from RtcDataListenr
        # 
  def onData(self, name, data):
    if name == 'joints':
      joints=map(lambda x: np.deg2rad(x), data.data)
      self._robot.set_target(joints)

    elif name == 'grip':
      if data.data == 0:
        self._robot.openGripper()
      elif data.data == 1:
        self._robot.closeGripper()

    return RTC.RTC_OK



#########################################
#  Initializers
#

def main():
  mgr = rtc_init("SawyerRTC", SawyerRTC, 'rtc.yaml')
  mgr.runManager()

if __name__ == "__main__":
  main()

