--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi, Jinwook Huh
--------------------------------

local state = {}
state._NAME = ...

local USE_SAFE_YAW = false

local Body   = require'Body'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'




dofile'../include.lua'
local lW = require'libWorld'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local munpack = require'msgpack.MessagePack'.unpack   --jwhuh
local ptable = require'util'.ptable

-- Cache some functions
local get_time = Body.get_time
-- Subscribe to important messages
local vision_ch = si.new_subscriber'vision0'
local rrt_ch = si.new_subscriber'rrt1'

-- Who to send to
local operator
--operator = Config.net.operator.wireless
operator = Config.net.operator.wired

local stream = Config.net.streams.world
local udp_ch = ENABLE_NET and stream and stream.udp and si.new_sender(operator, stream.udp)
local udp_wireless_ch = ENABLE_NET and stream and stream.udp and si.new_sender(Config.net.operator.wireless, stream.udp)
local world_ch = stream and stream.sub and si.new_publisher(stream.sub)







local t_entry, t_update, t_finish
local timeout = 30.0

local last_error


-- left and right will both request waist positions potentially
local lco, rco
local okL, qLWaypoint, qLWaist
local okR, qRWaypoint, qRWaist
local sequence, s, stage


-- Goal position is arm Init, with hands in front, ready to manipulate

local qLArmTarget, qRArmTarget
local tree_f, tree_b;

-----RRT parameters -----------
local Rrtjinwook = require'RRT_functions'
local Rrt_Run = 0;
local RRT_Path = {}
local RRT_Path_size = 0;
local RRT_idx = 0;
local transit = 0;


local function setArmJoints(qLArmTarget,qRArmTarget, dt,dqArmLim, absolute)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  local qL_approach, doneL2 = util.approachTolRad( qLArm, qLArmTarget, dqArmLim, dt ,nil,absolute)
  local qR_approach, doneR2 = util.approachTolRad( qRArm, qRArmTarget, dqArmLim, dt ,nil,absolute)

  if not absolute then
    for i=1,7 do
      local qL_increment = util.mod_angle(qL_approach[i]-qLArm[i])
      local qR_increment = util.mod_angle(qR_approach[i]-qRArm[i])
      qL_approach[i] = qLArm[i] + qL_increment
      qR_approach[i] = qRArm[i] + qR_increment
    end
  end

  Body.set_larm_command_position( qL_approach )
  Body.set_rarm_command_position( qR_approach )
  if doneL2 and doneR2 then return 1 end
end





function state.entry()
io.write(state._NAME, ' Entry\n')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  --sequence = {unpack(Config.arm.ready)}

  stage = 1
  Rrt_Run = 0
  --Slowly close all fingers
  --[[
  Body.move_lgrip1(Config.arm.torque.movement)
  Body.move_lgrip2(Config.arm.torque.movement)
  Body.move_rgrip1(Config.arm.torque.movement)
  Body.move_rgrip2(Config.arm.torque.movement)
  --]]

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()


print(string.format("qLArmr: %.2f %.2f %.2f %.2f %.2f %.2f %.2f" ,
unpack(qLArm*RAD_TO_DEG)
))

  --qLArmTarget = Body.get_inverse_arm_given_wrist(qLWrist, Config.arm.lrpy0)
  --qRArmTarget = Body.get_inverse_arm_given_wrist(qRWrist, Config.arm.rrpy0)

--[[  qLArmTarget = Body.get_inverse_larm(
    vector.zeros(7),
    Config.arm.trLArm0,
    Config.arm.ShoulderYaw0[1],
    mcm.get_stance_bodyTilt(),{0,0},true)

  qRArmTarget = Body.get_inverse_rarm(
    vector.zeros(7),
    Config.arm.trRArm0,
    Config.arm.ShoulderYaw0[2],
    mcm.get_stance_bodyTilt(),{0,0},true)
]]--

qLArmTarget = vector.new({120,0,0,0,0,0,0})*DEG_TO_RAD

  --qRArmTarget = vector.new({110,0,-10,-160,-90,-40,76})*DEG_TO_RAD
  qRArmTarget = vector.new({90,0,0,0,0,0, 0})*DEG_TO_RAD


--print(string.format("QLArmTarget: %.2f %.2f %.2f %.2f",
--unpack( vector.new(qLArmTarget)*RAD_TO_DEG)
--))
--[[
  local qLWrist = Body.get_inverse_lwrist(
    qLArm,
    Config.arm.pLWristTarget0,
    Config.arm.lShoulderYaw0)
  local qRWrist = Body.get_inverse_rwrist(
    qRArm,
    Config.arm.pRWristTarget0,
    Config.arm.rShoulderYaw0)

  qLArmTarget = Body.get_inverse_arm_given_wrist(qLWrist, Config.arm.lrpy0)
  qRArmTarget = Body.get_inverse_arm_given_wrist(qRWrist, Config.arm.rrpy0)


print(string.format("QLArmTarget: %.2f %.2f %.2f %.2f",
unpack( vector.new(qLArmTarget)*RAD_TO_DEG)
))






  mcm.set_stance_enable_torso_track(0)

  mcm.set_arm_dqVelLeft(Config.arm.vel_angular_limit)
  mcm.set_arm_dqVelRight(Config.arm.vel_angular_limit)
--]]
--[[
  if not IS_WEBOTS then
    for i=1,10 do
      Body.set_larm_command_velocity({500,500,500,500,500,500,500})
      unix.usleep(1e6*0.01);
      Body.set_rarm_command_velocity({500,500,500,500,500,500,500})
      unix.usleep(1e6*0.01);
      Body.set_larm_command_acceleration({50,50,50,50,50,50,50})
      unix.usleep(1e6*0.01);
      Body.set_rarm_command_acceleration({50,50,50,50,50,50,50})
      unix.usleep(1e6*0.01);
    end
  end
--]]
end


-- function state.update()
--   local t  = Body.get_time()
--   local dt = t - t_update
--   -- Save this at the last update time
--   t_update = t
-- --  print(state._NAME..' Update' )

--   local qLArm = Body.get_larm_command_position()
--   local qRArm = Body.get_rarm_command_position()

-- --[[
-- print(string.format("Cur: %.2f %.2f %.2f %.2f" ,
-- qLArm[1]*Body.RAD_TO_DEG,
-- qLArm[2]*Body.RAD_TO_DEG,
-- qLArm[3]*Body.RAD_TO_DEG,
-- qLArm[4]*Body.RAD_TO_DEG
-- ))

-- print(string.format("Nxt: %.2f %.2f %.2f %.2f",
-- qLArmTarget[1]*Body.RAD_TO_DEG,
-- qLArmTarget[2]*Body.RAD_TO_DEG,
-- qLArmTarget[3]*Body.RAD_TO_DEG,
-- qLArmTarget[4]*Body.RAD_TO_DEG

-- ))
-- --]]
--   local ret = movearm.setArmJoints(qLArmTarget,qRArmTarget,dt)
--   if ret==1 then

--     print("jinwook nice")
--     return "done"
--   end
-- end



function state.update()
  print("jinwook jinwook")
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
--  print(state._NAME..' Update' )

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local finished = 0;   --RRT

    print("Jinwook Jinwook Jinwook")

--[[
print(string.format("Cur: %.2f %.2f %.2f %.2f" ,
qLArm[1]*Body.RAD_TO_DEG,
qLArm[2]*Body.RAD_TO_DEG,
qLArm[3]*Body.RAD_TO_DEG,
qLArm[4]*Body.RAD_TO_DEG
))

print(string.format("Nxt: %.2f %.2f %.2f %.2f",
qLArmTarget[1]*Body.RAD_TO_DEG,
qLArmTarget[2]*Body.RAD_TO_DEG,
qLArmTarget[3]*Body.RAD_TO_DEG,
qLArmTarget[4]*Body.RAD_TO_DEG

))
--]]

  if Rrt_Run == 0 then
    
    -- used saved path for the RRT
    --[[
    Rrt_Run, RRT_Path = run_RRT_code()
  --state.jnwook_test()
    RRT_Path_size = table.getn(RRT_Path);
    ]]

    RRT_idx = 1;


  -- receive RRT path from MATLAB
  
    print("rrt data is received\n")
    local t_plane = rrt_ch:receive(true)

    print("t_plane is ")
    print(t_plane)
    --print(t_plane)
     if t_plane then
      Rrt_Run = 1;
             --print(t_plane)
      local mplanes = unpack(t_plane)
      planes = munpack(mplanes)
      local plane_size = table.getn(planes)
      print("plane_size")
      print(plane_size)
      --print(planes[1])
       print("Planes data!!!!!!!!!!!!!!!!!\n")

       RRT_Path_size = plane_size / 7

      --print(planes[1].."d"..planes[101+1].."d"..planes[2*101+1].."d"..planes[3*101+1].."d"..planes[4*101+1].."d"..planes[5*101+1].."d"..planes[6*101+1])
      -- print(planes[1].."d"..planes[2].."d"..planes[3].."d"..planes[4].."d"..planes[5].."d"..planes[6].."d"..planes[7])
       tmp_RRT_Path = {}
       for idx = 1, RRT_Path_size-1 do
          tmp_vector = {}
          tmp_vector[1] =  planes[(idx-1)*7+1]
          tmp_vector[2] =  planes[(idx-1)*7+2]
          tmp_vector[3] =  planes[(idx-1)*7+3]
          tmp_vector[4] =  planes[(idx-1)*7+4]
          tmp_vector[5] =  planes[(idx-1)*7+5]
          tmp_vector[6] =  planes[(idx-1)*7+6]
          tmp_vector[7] =  planes[(idx-1)*7+7]
          tmp_RRT_Path[idx] = tmp_vector;
      end
      --0  -60.0000         0         0   80.0000         0
      RRT_Path = tmp_RRT_Path;
       end


  else
     if transit == 0 then

--[[
       qLArmTarget[1] = (RRT_Path[RRT_idx][2])*3.141592/180;
       qLArmTarget[2] = (RRT_Path[RRT_idx][3])*3.141592/180;
       qLArmTarget[3] = (-1*RRT_Path[RRT_idx][4])*3.141592/180;
       qLArmTarget[4] = (RRT_Path[RRT_idx][5])*3.141592/180;
       qLArmTarget[5] = (-1*RRT_Path[RRT_idx][6]+90)*3.141592/180;
       qLArmTarget[6] = (-1*RRT_Path[RRT_idx][7])*3.141592/180;
       qLArmTarget[7] = 0;
]]--
--[[       qRArmTarget[1] = (RRT_Path[RRT_idx][2])*3.141592/180;
       qRArmTarget[2] = (RRT_Path[RRT_idx][3])*3.141592/180;
       qRArmTarget[3] = (-1*RRT_Path[RRT_idx][4])*3.141592/180;
       qRArmTarget[4] = (RRT_Path[RRT_idx][5])*3.141592/180;
       qRArmTarget[5] = (-1*RRT_Path[RRT_idx][6]+90)*3.141592/180;
       qRArmTarget[6] = (-1*RRT_Path[RRT_idx][7])*3.141592/180;
       qRArmTarget[7] = 0;
]]--
       qRArmTarget[1] = (RRT_Path[RRT_idx][2])*3.141592/180;
       qRArmTarget[2] = -1*(RRT_Path[RRT_idx][3])*3.141592/180;
       qRArmTarget[3] = -1*(-1*RRT_Path[RRT_idx][4])*3.141592/180;
       qRArmTarget[4] = (RRT_Path[RRT_idx][5])*3.141592/180;
       qRArmTarget[5] = -1*(-1*RRT_Path[RRT_idx][6]+90)*3.141592/180;
       qRArmTarget[6] = -1*(-1*RRT_Path[RRT_idx][7])*3.141592/180;
       qRArmTarget[7] = 0;



       print(RRT_idx);
      --local ret = movearm.setArmJoints(qLArmTarget,qRArmTarget,dt)


    --Body.set_larm_command_position(qcLArm)
    --Body.set_rarm_command_position(qcRArm)

    print(string.format("QLArmTarget: %.2f %.2f %.2f %.2f %.2f %.2f" ,
unpack( vector.new(qLArmTarget)*RAD_TO_DEG)
))

  local dqArmLim = vector.new(util.shallow_copy(Config.arm.vel_angular_limit_init))
  if IS_WEBOTS then dqArmLim = dqArmLim*2 end

    local ret = setArmJoints(qLArmTarget,qRArmTarget,dt,dqArmLim,true)

      if ret == 1 then
      RRT_idx = RRT_idx + 1;
      end

    end

    if RRT_idx > (RRT_Path_size - 2) then
        print ("RRT ended")
        finished = 1;
        return'teleopraw'
    end

  end

  if finished == 1 then
   --     return "done"
  end
  --      return'done'

end





-- if transit == 0 then
-- qLArmTarget[1] = 0;
-- qLArmTarget[2] = 0;
-- qLArmTarget[3] = 0;
-- qLArmTarget[4] =  -90*3.141592/180;
-- qLArmTarget[5] = 0;
-- qLArmTarget[6] = 0;



--  --qLArmTarget, tree_f, tree_b = util.RRT_bidirectional(qLArmTarget,0,0)

--  local ret = movearm.setArmJoints(qLArmTarget,qRArmTarget,dt)
--  if ret == 1 then
--   transit = 1;
--  end

-- else

--     qLArmTarget[1] = 0;
--     qLArmTarget[2] = 90*3.141592/180;
--     qLArmTarget[3] = 90*3.141592/180;
--     qLArmTarget[4] = -90*3.141592/180;
--     qLArmTarget[5] = 0;
--     qLArmTarget[6] = 0;

--     local ret2 = movearm.setArmJoints(qLArmTarget,qRArmTarget,dt)

--       if ret2 == 1 then
--           return "done"
--       end
-- end


function state.exit()
  print(state._NAME..' Exit')

end

return state



--[[
function state.exit()
  local libArmPlan = require 'libArmPlan'
  local arm_planner = libArmPlan.new_planner()
  --print("qLArm:",unpack(qLArmTarget))
  arm_planner:reset_torso_comp(qLArmTarget,qRArmTarget)


print("qRArm:",

unpack(vector.new(qRArmTarget)*180/math.pi))



  print(state._NAME..' Exit' )
end

return state

]]--
