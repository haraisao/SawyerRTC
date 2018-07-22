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

# Import Service implementation class
from ManipulatorCommonInterface_Common_idl_impl import *
from ManipulatorCommonInterface_Middle_idl_impl import *
from Sawyer_Middle_idl_impl import *

from MySawyer import *

# This module's spesification
sawyer_manipulator_spec = ["implementation_id", "SawyerRTC", 
     "type_name",         "SawyerRTC", 
     "description",       "Manipulator controller with JARA Standard Interfaces", 
     "version",           "1.0.0", 
     "vendor",            "AIST", 
     "category",          "Manipulator", 
     "activity_type",     "STATIC", 
     "max_instance",      "1", 
     "language",          "Python", 
     "lang_type",         "SCRIPT",
     ""]

############################
#  Data Ports
rtc_dataports={}
rtc_dataports['joints']={'data_type':'RTC.TimedFloatSeq', 'direction':'in'}
rtc_dataports['grip']={'data_type':'RTC.TimedOctet', 'direction':'in'}
rtc_dataports['out_joints']={'data_type':'RTC.TimedFloatSeq', 'direction':'out'}
rtc_dataports['out_velocity']={'data_type':'RTC.TimedFloatSeq', 'direction':'out'}
#rtc_dataports['out_torque']={'data_type':'RTC.TimedFloatSeq', 'direction':'out'}

# Service Ports
rtc_services={}
rtc_services['manipCommon']={'impl': ManipulatorCommonInterface_Common_i,
        'direction':'provider',
      'if_name': "JARA_ARM_ManipulatorCommonInterface_Common",
      'if_type_name' :"JARA_ARM::ManipulatorCommonInterface_Common"}

rtc_services['manipMiddle']={'impl': ManipulatorCommonInterface_Middle_i,
       'direction':'provider',
      'if_name': "JARA_ARM_ManipulatorCommonInterface_Middle",
      'if_type_name' :"JARA_ARM::ManipulatorCommonInterface_Middle"}

rtc_services['sawyerMiddle']={'impl': Sawyer_Middle_i,
       'direction':'provider',
      'if_name': "JARA_ARM_Sawyer_Middle",
      'if_type_name' :"JARA_ARM::Sawyer_Middle"}

#  Parameters
rtc_params={}
rtc_params['vmax']={'__type__':'float', 'default':'0.3', '__widget__':'text'}
rtc_params['vrate']={'__type__':'float', 'default':'2.0', '__widget__':'text'}
rtc_params['accuracy']={'__type__':'float', 'default':'0.01',
       '__widget__': 'text'}
rtc_params['gripper_reverse']={'__type__':'int', 'default':'1', 
      '__widget__':'radio', '__constraints__':'(0,1)'}


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
    DataFlowRTC_Base.__init__(self, manager,rtc_dataports, rtc_services, rtc_params)

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
    self._robot=MySawyer(self.getInstanceName(), anonymous=False)
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
    self._robot._vmax=self._vmax[0]
    self._robot._vrate=self._vrate[0]
    self._robot._gripper_reverse=(self._gripper_reverse[0] == 1)

    self._robot.onExecute()

    data=RTC.TimedFloatSeq(RTC.Time(0,0),[])

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
      self._robot.set_target(data.data)

    if name == 'grip':
      if data.data == 0:
        self._robot.openGripper()
      elif data.data == 1:
        self._robot.closeGripper()
    return RTC.RTC_OK



#########################################
#  Initializers
#
def SawyerRTCInit(manager):
  init_params_spec(sawyer_manipulator_spec, rtc_params)
  profile = OpenRTM_aist.Properties(defaults_str=sawyer_manipulator_spec)
  manager.registerFactory(profile,
                            SawyerRTC,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
  SawyerRTCInit(manager)

  # Create a component
  comp = manager.createComponent("SawyerRTC")

def main():
  mgr = OpenRTM_aist.Manager.init(sys.argv)
  mgr.setModuleInitProc(MyModuleInit)
  mgr.activateManager()
  mgr.runManager()

if __name__ == "__main__":
  main()

