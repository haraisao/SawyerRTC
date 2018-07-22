#!/bin/sh
#export RTM_IDL_ROOT=/usr/include/openrtm-1.1/rtm/idl
export RTM_IDL_ROOT=/usr/lib/python2.7/dist-packages/OpenRTM_aist/RTM_IDL
omniidl -bpython -I"$RTM_IDL_ROOT" -I"./idl" idl/*.idl
