#!/bin/sh
export RTM_ROOT=/usr/include/openrtm-1.1
omniidl -bpython -I"$RTM_ROOT/rtm/idl" -I"./idl" idl/*.idl
