#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file ManipulatorCommonInterface_Common_idl_examplefile.py
 @brief Python example implementations generated from ManipulatorCommonInterface_Common.idl
 @date $Date$

 @license the MIT License
 
 Copyright (C) 2018 Isao Hara,AIST,JP
 All rights reserved.

"""
#from __future__ import print_function
import omniORB
from omniORB import CORBA, PortableServer
import JARA_ARM, JARA_ARM__POA

from MySawyer import *

#
# ManipInfo:
#    manifactur: string
#    type: string
#    axisNum: ULONG
#    cmdCycle: ULONG
#    isGripper: boolea
def mk_manipInfo(info):
  if type(info) is list and len(info) == 5:
    return JARA_ARM.ManipInfo(*info)
  else:
    return None

  

#
#
class ManipulatorCommonInterface_Common_i (JARA_ARM__POA.ManipulatorCommonInterface_Common):
    """
    @class ManipulatorCommonInterface_Common_i
    Example class implementing IDL interface JARA_ARM.ManipulatorCommonInterface_Common
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        #self._robot=MySawyer()
        self._robot=None

    # RETURN_ID clearAlarms()
    def clearAlarms(self):
        # Must return: result
        try:
          res=self._robot.clearAlarms()
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')

    # RETURN_ID getActiveAlarm(out AlarmSeq alarms)
    def getActiveAlarm(self):
        # Must return: result, alarms
        try:
          res=self._robot.getActiveAlarm()
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''

          return JARA_ARM.RETURN_ID(code, msg), res

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')

    # RETURN_ID getFeedbackPosJoint(out JointPos pos)
    def getFeedbackPosJoint(self):
        # Must return: result, pos
        try:
          pos=self._robot.getFeedbackPosJoint()
          if pos is None :
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), pos

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')

    # RETURN_ID getManipInfo(out ManipInfo mInfo)
    def getManipInfo(self):
        # Must return: result, mInfo
        try:
          info=self._robot.getManipInfo()
          mInfo=mk_manipInfo(info)
          if mInfo is None :
            code=JARA_ARM.NG
            msg=str(info)
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), mInfo

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')

    # RETURN_ID getSoftLimitJoint(out LimitSeq softLimit)
    def getSoftLimitJoint(self):
        # Must return: result, softLimit
        try:
          softLimit=self._robot.getSoftLimitJoint()
          if softLimit is None :
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), softLimit

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')

    # RETURN_ID getState(out ULONG state)
    def getState(self):
        # Must return: result, state
        try:
          state=self._robot.getState()
          if state is None :
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg),state

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')

    # RETURN_ID servoOFF()
    def servoOFF(self):
        # Must return: result
        try:
          res=self._robot.servoOFF()
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')

    # RETURN_ID servoON()
    def servoON(self):
        # Must return: result
        try:
          res=self._robot.servoON()
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')

    # RETURN_ID setSoftLimitJoint(in LimitSeq softLimit)
    def setSoftLimitJoint(self, softLimit):
        # Must return: result
        try:
          res=self._robot.setSoftLimitJoint(softLimit)
          if res is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg)

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')



if __name__ == "__main__":
    import sys
    
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv)
    
    # As an example, we activate an object in the Root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Create an instance of a servant class
    servant = ManipulatorCommonInterface_Common_i()

    # Activate it in the Root POA
    poa.activate_object(servant)

    # Get the object reference to the object
    objref = servant._this()
    
    # Print a stringified IOR for it
    print(orb.object_to_string(objref))

    # Activate the Root POA's manager
    poa._get_the_POAManager().activate()

    # Run the ORB, blocking this thread
    orb.run()

