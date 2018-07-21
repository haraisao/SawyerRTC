#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

#  Copyright(C) 2018 Isao Hara,AIST,JP
#  All rights reserved
#  License: the MIT License

import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist


##
# @class StdManipulator
# @brief Manipulator controller with JARA Standard Interfaces
# 
# 
class DataFlowRTC_Base(OpenRTM_aist.DataFlowComponentBase):
  
  ##
  # @brief constructor
  # @param manager Maneger Object
  # 
  def __init__(self, manager, rtc_dataports, rtc_services, rtc_params):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    self._dataports=rtc_dataports
    self._services=rtc_services
    self._params=rtc_params

    #
    # set dataport
    for k in self._dataports.keys():
      _d, _p = init_dataport(k, self._dataports)

      if self._dataports[k]['direction'] == 'in':
        self.__dict__['_d_'+k] = _d
        self.__dict__['_'+k+'In'] = _p
      elif self._dataports[k]['direction'] == 'out':
        self.__dict__['_d_'+k] = _d
        self.__dict__['_'+k+'Out'] = _p
      else:
        pass

    # set service port
    for k in self._services.keys():
      if self._services[k]['direction'] == 'provider':
        self.__dict__['_'+k+'Port'] = OpenRTM_aist.CorbaPort(k)
        self.__dict__['_'+k+'_service'] = self._services[k]['impl']()
      elif self._services[k]['direction'] == 'consumer':
        self.__dict__['_'+k+'Port'] = OpenRTM_aist.CorbaPort(k)
        self.__dict__['_'+k+'_service'] = OpenRTM_aist.CorbaConsumer(interfaceType=self._services[k]['if_type'])

    # initialize of configuration-data.
    for x in init_params(self._params):
      self.__dict__['_'+x[0]] = [x[1]]
    
  ##
  #
  # The initialize action (on CREATED->ALIVE transition)
  # formaer rtc_init_entry() 
  # 
  # @return RTC::ReturnCode_t
  # 
  #
  def onInitialize(self):

    # Bind variables and configuration variable
    for k in self._params.keys():
      self.bindParameter(k, self.__dict__['_'+k], self._params[k]['default'])
                  
    # Set DataPort buffers
    for k in self._dataports.keys():
      if self._dataports[k]['direction'] == 'in':
        self.addOutPort(k, self.__dict__['_'+k+'In'])
      elif self._dataports[k]['direction'] == 'out':
        self.addOutPort(k, self.__dict__['_'+k+'Out'])
      else:
        pass

    
    # Set service ports
    service_keys = self._services.keys()
    service_keys.sort()
    for k in service_keys:
      s_port=self.__dict__['_'+k+'Port']
      service=self.__dict__['_'+k+'_service']

      if self._services[k]['direction'] == 'provider':
        s_port.registerProvider(self._services[k]['if_name'], self._services[k]['if_type_name'], service)
        self.addPort(s_port)

      elif self._services[k]['direction'] == 'consumer':
        s_port.registerConsumer(self._services[k]['if_name'], self._services[k]['if_type_name'], service)
        self.addPort(s_port)
      else:
        pass
    
    return RTC.RTC_OK
  

#########################################
#
def init_params_spec(spec, param):
  for k1 in param.keys():
    for k2 in param[k1].keys():
      e="conf." + k2 + "." + k1
      v=param[k1][k2]
      spec.insert(-1, e)
      spec.insert(-1, v)

def init_params(param):
  res=[]
  for k1 in param.keys():
    if param[k1]['__type__'] == 'string':
      val=param[k1]['default']
    else:
      val=eval(param[k1]['default'])
    res.append([k1, val])
  return res

def init_dataport(name, dataport):
  dname=dataport[name]['data_type']
  typ=dataport[name]['direction']
  m, d=dname.split('.')
  d_type = eval(m+"._d_"+d)
  v_arg = [None] * ((len(d_type) - 4) / 2)
  _d = eval(dname)(*v_arg)
  if typ == 'in':
    _p = OpenRTM_aist.InPort(name, _d)
  elif typ == 'out':
    _p = OpenRTM_aist.OutPort(name, _d)
  else:
    _p = None

  return _d,_p

#########################################
