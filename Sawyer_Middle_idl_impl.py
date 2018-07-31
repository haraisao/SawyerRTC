#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 Copyright(C) 2018 Isao Hara,AIST,JP
 All rights reserved

"""
from __future__ import print_function
import omniORB
from omniORB import CORBA, PortableServer
import JARA_ARM, JARA_ARM__POA

#from MySawyer import *

#
#  Sawter_Middle.idl
#
class Sawyer_Middle_i (JARA_ARM__POA.Sawyer_Middle):

    def __init__(self):
        self._robot=None
    #
    # RETURN_ID moveCertesianRel(pos, rot, flag)
    #  pos; DoubleSeq, rot: DoubleSeq, flag: octet
    def moveCartesianRel(self, pos, rot, flag):
        # Must return: result
        try:
          res=self._robot.moveCartesianRel(pos, rot, flag)
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

