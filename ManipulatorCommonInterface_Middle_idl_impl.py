#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file ManipulatorCommonInterface_Middle_idl_examplefile.py
 @brief Python example implementations generated from ManipulatorCommonInterface_Middle.idl
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
#
class ManipulatorCommonInterface_Middle_i (JARA_ARM__POA.ManipulatorCommonInterface_Middle):
    """
    @class ManipulatorCommonInterface_Middle_i
    Example class implementing IDL interface JARA_ARM.ManipulatorCommonInterface_Middle
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        #self._robot=MySawyer()
        self._robot=None

    # RETURN_ID closeGripper()
    def closeGripper(self):
        # Must return: result
        try:
          res=self._robot.closeGripper()
          if res:
            code=JARA_ARM.OK
            msg=''
          else:
            code=JARA_ARM.NG
            msg='Fail to close'
          return JARA_ARM.RETURN_ID(code, msg)
        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')

    # RETURN_ID getBaseOffset(out HgMatrix offset)
    def getBaseOffset(self):
        # Must return: result, offset
        try:
          offset=self._robot.getBaseOffset()
          if offset is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), offset

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')

    # RETURN_ID getFeedbackPosCartesian(out CarPosWithElbow pos)
    def getFeedbackPosCartesian(self):
        # Must return: result, pos
        try:
          pos=self._robot.getFeedbackPosCartesian()
          if pos is None:
            code=JARA_ARM.NG
            msg='Not supported'
            pos=JARA_ARM.CarPosWithElbow(None, None, None) 
          else:
            code=JARA_ARM.OK
            msg=''
            pos=JARA_ARM.CarPosWithElbow(pos[0], pos[1], pos[2]) 
          return JARA_ARM.RETURN_ID(code, msg), pos

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error'),None

    # RETURN_ID getMaxSpeedCartesian(out CartesianSpeed speed)
    def getMaxSpeedCartesian(self):
        # Must return: result, speed
        try:
          speed=self._robot.getMxSpeedCartesian()
          if speed is None:
            code=JARA_ARM.NG
            msg='Not supported'
            speed=JARA_ARM.CartesianSpeed(0,0) 
          else:
            code=JARA_ARM.OK
            msg=''
            speed=JARA_ARM.CartesianSpeed(speed[0],speed[1]) 
          return JARA_ARM.RETURN_ID(code, msg), speed

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error'), None

    # RETURN_ID getMaxSpeedJoint(out DoubleSeq speed)
    def getMaxSpeedJoint(self):
        # Must return: result, speed
        try:
          speed=self._robot.getMxSpeedJoint()
          if speed is None:
            code=JARA_ARM.NG
            msg='Not supported'
            speed=[]
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), speed

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error'),None

    # RETURN_ID getMinAccelTimeCartesian(out double aclTime)
    def getMinAccelTimeCartesian(self):
        # Must return: result, aclTime
        try:
          aclTime=self._robot.getMinAccelTimeCartesian()
          if aclTime is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), aclTime

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error'),None

    # RETURN_ID getMinAccelTimeJoint(out double aclTime)
    def getMinAccelTimeJoint(self):
        # Must return: result, aclTime
        try:
          aclTime=self._robot.getMinAccelTimeJoint()
          if aclTime is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), aclTime

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error'), None

    # RETURN_ID getSoftLimitCartesian(out LimitValue xLimit, out LimitValue yLimit, out LimitValue zLimit)
    def getSoftLimitCartesian(self):
        # Must return: result, xLimit, yLimit, zLimit
        try:
          lmits=self._robot.getSoftLimitCartesian()
          if limits is None:
            code=JARA_ARM.NG
            msg='Not supported'
            limitx=JARA_ARM.LimitValue(0,0)
            limity=JARA_ARM.LimitValue(0,0)
            limitz=JARA_ARM.LimitValue(0,0)
          else:
            code=JARA_ARM.OK
            msg=''
            limitx=JARA_ARM.LimitValue(limits[0][0], limits[0][1])
            limity=JARA_ARM.LimitValue(limits[1][0], limits[1][1])
            limitz=JARA_ARM.LimitValue(limits[2][0], limits[2][1])
          return JARA_ARM.RETURN_ID(code, msg), limitx,limity,limitz

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error'),None,None,None

    # RETURN_ID moveGripper(in ULONG angleRatio)
    def moveGripper(self, angleRatio):
        # Must return: result
        try:
          res=self._robot.moveGripper()
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

    # RETURN_ID moveLinearCartesianAbs(in CarPosWithElbow carPoint)
    def moveLinearCartesianAbs(self, carPoint):
        # Must return: result
        try:
          res=self._robot.moveLinearCartesianAbs(carPoint.carPos, carPoint.elbow, carPoint.structFlag)
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

    # RETURN_ID moveLinearCartesianRel(in CarPosWithElbow carPoint)
    def moveLinearCartesianRel(self, carPoint):
        # Must return: result
        try:
          res=self._robot.moveLinearCartesianRel(carPoint.carPos, carPoint.elbow, carPoint.structFlag)
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


    # RETURN_ID movePTPCartesianAbs(in CarPosWithElbow carPoint)
    def movePTPCartesianAbs(self, carPoint):
        # Must return: result
        try:
          res=self._robot.movePTPCartesianAbs(carPoint.carPos, carPoint.elbow, carPoint.structFlag)
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

    # RETURN_ID movePTPCartesianRel(in CarPosWithElbow carPoint)
    def movePTPCartesianRel(self, carPoint):
        # Must return: result
        try:
          res=self._robot.movePTPCartesianRel(carPoint.carPos, carPoint.elbow, carPoint.structFlag)
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

    # RETURN_ID movePTPJointAbs(in JointPos jointPoints)
    def movePTPJointAbs(self, jointPoints):
        # Must return: result
        try:
          res=self._robot.movePTPJointAbs(jointPoints)
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

    # RETURN_ID movePTPJointRel(in JointPos jointPoints)
    def movePTPJointRel(self, jointPoints):
        # Must return: result
        try:
          res=self._robot.movePTPJointRel(jointPoints)
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

    # RETURN_ID openGripper()
    def openGripper(self):
        # Must return: result
        try:
          res=self._robot.openGripper()
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

    # RETURN_ID pause()
    def pause(self):
        # Must return: result
        try:
          res=self._robot.pause_motion()
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

    # RETURN_ID resume()
    def resume(self):
        # Must return: result
        try:
          res=self._robot.resume_motion()
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

    # RETURN_ID stop()
    def stop(self):
        # Must return: result
        try:
          res=self._robot.stop_motion()
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

    # RETURN_ID setAccelTimeCartesian(in double aclTime)
    def setAccelTimeCartesian(self, aclTime):
        # Must return: result
        try:
          res=self._robot.setAccelTimeCartesian(aclTime)
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

    # RETURN_ID setAccelTimeJoint(in double aclTime)
    def setAccelTimeJoint(self, aclTime):
        # Must return: result
        try:
          res=self._robot.setAccelTimeJoint(aclTime)
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

    # RETURN_ID setBaseOffset(in HgMatrix offset)
    def setBaseOffset(self, offset):
        # Must return: result
        try:
          res=self._robot.setBaseOffset(offset)
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

    # RETURN_ID setControlPointOffset(in HgMatrix offset)
    def setControlPointOffset(self, offset):
        # Must return: result
        try:
          res=self._robot.setControlPointOffset(offset)
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

    # RETURN_ID setMaxSpeedCartesian(in CartesianSpeed speed)
    def setMaxSpeedCartesian(self, speed):
        # Must return: result
        try:
          res=self._robot.setMaxSpeedCartesian(speed.translation, speed.rotation)
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

    # RETURN_ID setMaxSpeedJoint(in DoubleSeq speed)
    def setMaxSpeedJoint(self, speed):
        # Must return: result
        try:
          res=self._robot.setMaxSpeedJoint(speed)
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

    # RETURN_ID setMinAccelTimeCartesian(in double aclTime)
    def setMinAccelTimeCartesian(self, aclTime):
        # Must return: result
        try:
          res=self._robot.setMinAccelTimeCartesian(aclTime)
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

    # RETURN_ID setMinAccelTimeJoint(in double aclTime)
    def setMinAccelTimeJoint(self, aclTime):
        # Must return: result
        try:
          res=self._robot.setMinAccelTimeJoint(aclTime)
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

    # RETURN_ID setSoftLimitCartesian(in LimitValue xLimit, in LimitValue yLimit, in LimitValue zLimit)
    def setSoftLimitCartesian(self, xLimit, yLimit, zLimit):
        # Must return: result
        try:
          res=self._robot.setSoftLimitCartesian([xLimt.upper, xLimit.lower], [yLimit.upper, xLimit.lower], [zLimit.upper, zLimit.lower])
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

    # RETURN_ID setSpeedCartesian(in ULONG spdRatio)
    def setSpeedCartesian(self, spdRatio):
        # Must return: result
        try:
          res=self._robot.setSpeedCartesian(spdRatio)
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

    # RETURN_ID setSpeedJoint(in ULONG spdRatio)
    def setSpeedJoint(self, spdRatio):
        # Must return: result
        try:
          res=self._robot.setSpeedJoint(spdRatio)
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

    # RETURN_ID moveCircularCartesianAbs(in CarPosWithElbow carPointR, in CarPosWithElbow carPointT)
    def moveCircularCartesianAbs(self, carPointR, carPointT):
        # Must return: result
        try:
          res=self._robot.moveCircularCartesianAbs([carPointR.catPos, carPointR.elbow, carPointR.structFlag], [carPointT.catPos, carPointT.elbow, carPointT.structFlag])
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

    # RETURN_ID moveCircularCartesianRel(in CarPosWithElbow carPointR, in CarPosWithElbow carPointT)
    def moveCircularCartesianRel(self, carPointR, carPointT):
        # Must return: result
        try:
          res=self._robot.moveCircularCartesianRel([carPointR.catPos, carPointR.elbow, carPointR.structFlag], [carPointT.catPos, carPointT.elbow, carPointT.structFlag])
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

    # RETURN_ID setHome(in JointPos jointPoint)
    def setHome(self, jointPoint):
        # Must return: result
        try:
          res=self._robot.setHome(jointPoint)
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

    # RETURN_ID getHome(out JointPos jointPoint)
    def getHome(self):
        # Must return: result, jointPoint
        try:
          jointPoint=self._robot.getHome()
          if jointPoint is None:
            code=JARA_ARM.NG
            msg='Not supported'
          else:
            code=JARA_ARM.OK
            msg=''
          return JARA_ARM.RETURN_ID(code, msg), jointPoint

        except AttributeError:
          raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)

        except:
          return JARA_ARM.RETURN_ID(JARA_ARM.NG, 'Unknown Error')

    # RETURN_ID goHome()
    def goHome(self):
        # Must return: result
        try:
          res=self._robot.goHome()
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
    servant = ManipulatorCommonInterface_Middle_i()

    # Activate it in the Root POA
    poa.activate_object(servant)

    # Get the object reference to the object
    objref = servant._this()
    
    # Print a stringified IOR for it
    print ( orb.object_to_string(objref) )

    # Activate the Root POA's manager
    poa._get_the_POAManager().activate()

    # Run the ORB, blocking this thread
    orb.run()

