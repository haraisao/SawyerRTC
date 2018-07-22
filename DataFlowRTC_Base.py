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
import omniORB
import RTC
import OpenRTM_aist


#########################################################################
#
# DataListener 
#   This class connected with DataInPort
#
class RtcDataListener(OpenRTM_aist.ConnectorDataListenerT):
    def __init__(self, name, typ, obj, func=None):
        self._name = name
        self._type = typ
        self._obj = obj
        self._func = func
    
    def __call__(self, info, cdrdata):
        data = OpenRTM_aist.ConnectorDataListenerT.__call__(self,
                        info, cdrdata, self._type)

        if self._func :
          self._func(self._name, data)

        else:
          self._obj.onData(self._name, data)

    def _set_callback(self, func):
        self._func = func

##
# @class DataFlowRTC_Base
# @brief Base class for DataFlowComponent
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
  
  #
  #
  def onData(self, name, data):

    return RTC.RTC_OK

  #
  #
  def bindDataListener(self, portname, func=None):
    try:
      port = eval('self._'+portname+'In')
      if isinstance(port, OpenRTM_aist.InPort) :
        port.addConnectorDataListener(
          OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE,
          RtcDataListener(port._name, port._value, self, func))
        return True
      else:
        print("Invalid PortType")
        return False
    except:
      traceback.print_exc()
      return False

#########################################
#
#  Funcrions
#
def init_params_spec(spec, param):
  for k1 in param.keys():
    for k2 in param[k1].keys():
      e="conf." + k2 + "." + k1
      v=param[k1][k2]
      spec.insert(-1, e)
      spec.insert(-1, v)

#
#
def init_params(param):
  res=[]
  for k1 in param.keys():
    if param[k1]['__type__'] == 'string':
      val=param[k1]['default']
    else:
      val=eval(param[k1]['default'])
    res.append([k1, val])
  return res

#
#
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

#
#  DataType for CORBA
#
def instantiateDataType(dtype):
    if isinstance(dtype, int) : desc = [dtype]
    elif isinstance(dtype, tuple) : desc = dtype
    else :
        desc=omniORB.findType(dtype._NP_RepositoryId)

    if desc[0] in [omniORB.tcInternal.tv_alias ]: return instantiateDataType(desc[2])

    if desc[0] in [omniORB.tcInternal.tv_short,
                   omniORB.tcInternal.tv_long,
                   omniORB.tcInternal.tv_ushort,
                   omniORB.tcInternal.tv_ulong,
                   omniORB.tcInternal.tv_boolean,
                   omniORB.tcInternal.tv_char,
                   omniORB.tcInternal.tv_octet,
                   omniORB.tcInternal.tv_longlong,
                   omniORB.tcInternal.tv_enum
                  ]: return 0

    if desc[0] in [omniORB.tcInternal.tv_float,
                   omniORB.tcInternal.tv_double,
                   omniORB.tcInternal.tv_longdouble
                  ]: return 0.0

    if desc[0] in [omniORB.tcInternal.tv_sequence,
                   omniORB.tcInternal.tv_array,
                  ]: return []


    if desc[0] in [omniORB.tcInternal.tv_string ]: return ""
    if desc[0] in [omniORB.tcInternal.tv_wstring,
                   omniORB.tcInternal.tv_wchar
                  ]: return u""
    if desc[0] == omniORB.tcInternal.tv_struct:
        arg = []
        for i in  range(4, len(desc), 2):
            attr = desc[i]
            attr_type = desc[i+1]
            arg.append(instantiateDataType(attr_type))
        return desc[1](*arg)
    return None

#########################################
