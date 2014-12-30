#!/usr/bin/env luajit
dofile'../include.lua'
local JOINTS_DATE = '12.19.2014.14.10.48'
local SENSOR_DATE = '12.19.2014.14.08.13'
local SENSOR_PREFIX = 'k2_depth'

local util = require'util'
local mp = require'msgpack.MessagePack'
local libLog = require'libLog'
local replay_sensor = libLog.open(HOME..'/Data/', SENSOR_DATE, SENSOR_PREFIX)
local replay_joints = libLog.open(HOME..'/Data/', JOINTS_DATE, 'joint')
local meta_sensor = replay_sensor:unroll_meta()
local logged_sensor = replay_sensor:log_iter()
local meta_joints = replay_joints:unroll_meta()
--local logged_joints = replay_joints:log_iter()

print(#meta_sensor,#meta_joints)

local get_time = unix.time
local ijoints = 1
local t0

print('Initial times', meta_sensor[1].t, meta_joints[1].t)

for i, sensor in logged_sensor do
  local joints = meta_joints[ijoints]
  print(joints.t, sensor.t)
  while sensor.t < joints.t do
    ijoints = ijoints + 1
    joints = meta_joints[ijoints]
  end
end
