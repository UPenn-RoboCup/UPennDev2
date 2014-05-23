--------------------------------
-- Look at Wire
-- (c) 2014 Stephen McGill
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local t_entry, t_update, t_finish
local timeout = 10.0
local get_time = Body.get_time

-- State specific
require'vcm'
local lost_timeout = Config.fsm.armWireLook.lost_timeout
local thresh_yaw = Config.fsm.armWireLook.thresh_yaw
local thresh_roll = Config.fsm.armWireLook.thresh_roll
local roll_rate = Config.fsm.armWireLook.roll_rate
local yaw_rate = Config.fsm.armWireLook.yaw_rate
-- Need the camera information
local cam_metadata = Config.camera[1]
local hfov, cam_width = cam_metadata.hfov, cam_metadata.width

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = get_time()
  t_update = t_entry
  t_finish = t
end

function state.update()
  local t  = get_time()
  local dt = t - t_update
  local dt_entry = t - t_entry
  if dt_entry>timeout then return'timeout' end
  -- Save this at the last update time
  t_update = t
  -- Have we seen the wire recently?
  local wire_dt = t - vcm.get_wire_t()
  if wire_dt>lost_timeout then return'lost' end
  -- Is the wire within threshold in the camera frame?
  local cam_roll, cam_pitch, cam_yaw = unpack(vcm.get_wire_cam_rpy())
  local done_yaw = math.abs(cam_yaw)<thresh_yaw
  local done_roll = math.abs(cam_roll)<thresh_roll
  -- If aligned, then we are done, and ready to approach the wire
  if done_roll and done_yaw then return'done' end
  --if done_roll then return'done' end
  -- Where are we now, in camera roll/yaw?
  -- TODO: Assume that the camera timestamp is not too far from the motor ts
  local qLArm = Body.get_larm_position()
  -- Use a simple P controller. TODO: Is PID worth it?
  local roll_corr = (cam_roll - 0) * roll_rate * dt
  local yaw_corr = (cam_yaw - 0) * yaw_rate * dt
  qLArm[5] = qLArm[5] + roll_corr
  qLArm[1] = qLArm[1] + yaw_corr
  -- Move the bounding box attention
  --[[
  local bbox = vcm.get_wire_bbox()
  -- Move the yaw a bit
  local i_corr = cam_width * yaw_corr / hfov / 4
  bbox[1], bbox[2] = bbox[1] + i_corr, bbox[2] + i_corr
  vcm.set_wire_bbox(bbox)
  --]]
  -- Set the command
  Body.set_larm_command_position(qLArm)
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
