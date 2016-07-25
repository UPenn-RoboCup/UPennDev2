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
require'rcm'

-- Cache some functions
local get_time = Body.get_time
-- Subscribe to important messages
local vision_ch = si.new_subscriber'vision0'


local rrt_streams = Config.net.streams['rrt1']
--local rrt_ch = si.new_subscriber'rrt1'
--local rrt_ch = si.new_subscriber(43351, '158.130.109.11')
local rrt_ch = si.new_subscriber(43351, '192.168.123.200')
--local rrt_ch = si.new_subscriber(43351, '158.130.104.207')
--local rrt_ch = si.new_subscriber(43351, '10.0.0.16')
--local rrt_ch = si.new_subscriber(43351, '172.20.20.20')



--local depth_net_ch = si.new_publisher(depth_streams.tcp)
--local color_net_ch = si.new_publisher(color_streams.tcp)
--
--local depth_ch = si.new_publisher(depth_streams.sub)
--local color_ch = si.new_publisher(color_streams.sub)




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
  dt = dt  --  dt = dt*6
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
--  rcm.set_RRT_finished(0);
  print(rcm.get_RRT_finished())
  --sequence = {unpack(Config.arm.ready)}

  stage = 1
  Rrt_Run = 0
  head_ch:send'mesh'
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

  --qLArmTarget = vector.new({90,0,0,0,0,0,0})*DEG_TO_RAD
  --qRArmTarget = vector.new({90,0,0,0,0,0, 0})*DEG_TO_RAD

  qLArmTarget = Body.get_larm_position()
  qRArmTarget = Body.get_rarm_position()


  
  -- Open rgrip
  --Grip_hold = {0,0,0}
  --Grip_open = {0.496719, 0.458369, 0.50132}
  --Body.set_lgrip_command_position(Grip_open)   --working one



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

  print("Wait RRT Path")

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
--  print("jinwook jinwook")
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
--  print(state._NAME..' Update' )

  dd = Body.get_lidar_position()
  dd = (dd*180)/3.141592
  print(dd)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local finished = 0;   --RRT

 --   print("Jinwook Jinwook Jinwook")

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
  
  --  print("rrt data is received\n")
    local t_plane = rrt_ch:receive(true)

    local t_test = rrt_ch:receive(true)
    --print(t_test)
    if t_test then

      print("Data is receving several times!")

    end

  --  print("t_plane is ")
  --  print(t_plane)
    --print(t_plane)
    if t_plane then
        Rrt_Run = 1;
             --print(t_plane)
        local mplanes = unpack(t_plane)
        planes = munpack(mplanes)
        local plane_size = table.getn(planes)
        --print("plane_size")
        --print(plane_size)
        --print("Planes data!!!!!!!!!!!!!!!!!\n")

        RRT_Path_size = plane_size / 7

        head_vector = {}
        head_vector[1] =  planes[1]
        head_vector[2] =  planes[2]
        head_vector[3] =  planes[3]
        head_vector[4] =  planes[4]
        head_vector[5] =  planes[5]
        head_vector[6] =  planes[6]
        head_vector[7] =  planes[7]
      
        local eef =  vector.zeros(6)
        eef[1] = planes[8]
        eef[2] = planes[9]
        eef[3] = planes[10]
        eef[4] = planes[11]
        eef[5] = planes[12]
        eef[6] = planes[13]

        print("****************************************************")
        print(rcm.get_RRT_eef())
        rcm.set_RRT_eef(eef);
        print(rcm.get_RRT_eef())

          --print(head_vector[1], head_vector[2], head_vector[3])

        RRT_Path_l_size = head_vector[1]
        RRT_Path_r_size = head_vector[2]
        RRT_l_end_effector = head_vector[3]
        RRT_r_end_effector = head_vector[4]
        RRT_Path_size = RRT_Path_l_size

      if RRT_Path_r_size > RRT_Path_l_size then
        RRT_Path_size = RRT_Path_r_size
      end

      --print(planes[1].."d"..planes[101+1].."d"..planes[2*101+1].."d"..planes[3*101+1].."d"..planes[4*101+1].."d"..planes[5*101+1].."d"..planes[6*101+1])
      -- print(planes[1].."d"..planes[2].."d"..planes[3].."d"..planes[4].."d"..planes[5].."d"..planes[6].."d"..planes[7])
       tmp_RRT_l_Path = {}

       for idx = 1, RRT_Path_l_size do
          tmp_vector = {}
          tmp_vector[1] =  planes[(idx+1)*7+1]
          tmp_vector[2] =  planes[(idx+1)*7+2]
          tmp_vector[3] =  planes[(idx+1)*7+3]
          tmp_vector[4] =  planes[(idx+1)*7+4]
          tmp_vector[5] =  planes[(idx+1)*7+5]
          tmp_vector[6] =  planes[(idx+1)*7+6]
          tmp_vector[7] =  planes[(idx+1)*7+7]
          tmp_RRT_l_Path[idx] = tmp_vector;
      end
      --0  -60.0000         0         0   80.0000         0
      RRT_l_Path = tmp_RRT_l_Path;
      tmp_RRT_r_Path = {}

      for idx = (RRT_Path_l_size+1), (RRT_Path_l_size+RRT_Path_r_size) do
          tmp_vector = {}
          tmp_vector[1] =  planes[(idx+1)*7+1]
          tmp_vector[2] =  planes[(idx+1)*7+2]
          tmp_vector[3] =  planes[(idx+1)*7+3]
          tmp_vector[4] =  planes[(idx+1)*7+4]
          tmp_vector[5] =  planes[(idx+1)*7+5]
          tmp_vector[6] =  planes[(idx+1)*7+6]
          tmp_vector[7] =  planes[(idx+1)*7+7]
          tmp_RRT_r_Path[idx-RRT_Path_l_size] = tmp_vector;
      end
      --0  -60.0000         0         0   80.0000         0
      RRT_r_Path = tmp_RRT_r_Path;
--      gripper_ch:send'open' 
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
       RRT_r_idx = RRT_idx;
       RRT_l_idx = RRT_idx;

       if RRT_idx > RRT_Path_l_size then
          RRT_l_idx = RRT_Path_l_size
       end


       if RRT_idx > RRT_Path_r_size then
          RRT_r_idx = RRT_Path_r_size
       end

       --print(RRT_r_idx)
       --print(RRT_l_idx)
       qRArmTarget[1] = (RRT_r_Path[RRT_r_idx][2])*3.141592/180;
       qRArmTarget[2] = -1*(RRT_r_Path[RRT_r_idx][3])*3.141592/180;
       qRArmTarget[3] = -1*(-1*RRT_r_Path[RRT_r_idx][4])*3.141592/180;
       qRArmTarget[4] = (RRT_r_Path[RRT_r_idx][5])*3.141592/180;
       qRArmTarget[5] = -1*(-1*RRT_r_Path[RRT_r_idx][6]+90)*3.141592/180;
       qRArmTarget[6] = -1*(-1*RRT_r_Path[RRT_r_idx][7])*3.141592/180;
       qRArmTarget[7] = RRT_r_end_effector+3.141592/2;
       qRArmTarget[7] = RRT_r_end_effector;

       --print('RRT_l_path')
       --print(RRT_l_Path[RRT_l_idx][5])

       qLArmTarget[1] = (RRT_l_Path[RRT_l_idx][2])*3.141592/180;
       qLArmTarget[2] = (RRT_l_Path[RRT_l_idx][3])*3.141592/180;
       qLArmTarget[3] = (-1*RRT_l_Path[RRT_l_idx][4])*3.141592/180;
       qLArmTarget[4] = (RRT_l_Path[RRT_l_idx][5])*3.141592/180;
       qLArmTarget[5] = (-1*RRT_l_Path[RRT_l_idx][6]+90)*3.141592/180;
       qLArmTarget[6] = (-1*RRT_l_Path[RRT_l_idx][7])*3.141592/180;
       qLArmTarget[7] = RRT_l_end_effector+3.141592/2;     -- why it add 90 degree???
       qLArmTarget[7] = RRT_l_end_effector;     -- why it add 90 degree???

       --qLArmTarget[7] = 0;
--[[
        qLArmTarget[1] = (RRT_Path[RRT_idx][2])*3.141592/180;
        qLArmTarget[2] = (RRT_Path[RRT_idx][3])*3.141592/180;
        qLArmTarget[3] = (-1*RRT_Path[RRT_idx][4])*3.141592/180;
        qLArmTarget[4] = (RRT_Path[RRT_idx][5])*3.141592/180;
        qLArmTarget[5] = (-1*RRT_Path[RRT_idx][6]+90)*3.141592/180;
        qLArmTarget[6] = (-1*RRT_Path[RRT_idx][7])*3.141592/180;
        qLArmTarget[7] = 0;
]]--

        local qLArmActual = Body.get_larm_position()
        local qRArmActual = Body.get_rarm_position()
        local qWaistActual = Body.get_waist_position()
        local qLArmCommand = Body.get_larm_command_position()
        local qRArmCommand = Body.get_rarm_command_position()
        local qWaistCommand = Body.get_waist_command_position()



       print(string.format("qLArmCommand: %.2f %.2f %.2f %.2f %.2f %.2f %.2f" ,
       unpack(qLArmCommand)))


       --print(RRT_idx);
      --local ret = movearm.setArmJoints(qLArmTarget,qRArmTarget,dt)


    --Body.set_larm_command_position(qcLArm)
    --Body.set_rarm_command_position(qcRArm)


        local dqArmLim = vector.new(util.shallow_copy(Config.arm.vel_angular_limit_init))
        if IS_WEBOTS then dqArmLim = dqArmLim*2 end

        local ret = setArmJoints(qLArmTarget,qRArmTarget,dt,dqArmLim,true)

        if ret == 1 then
          RRT_idx = RRT_idx + 1;
        end

    end

    if RRT_idx > (RRT_Path_size) then

    --    Grip_close = {0.496719, 0.458369, 0.50132}
    --    Grip_open = {-0.496719, -0.458369, -0.50132}
    --    Body.set_rgrip_command_position(Grip_open)

	gripper_ch:send'open'
	rgrip_position = Body.get_rgrip_position()
	print(rgrip_position[1])
	print("test_gripper position")

	if rgrip_position[1] < -1.0 then

	        print ("RRT ended")
        	finished = 1;
        	Rrt_Run = 0;
--        return'teleopraw'

        	print("shared")
        	print(rcm.get_RRT_finished())
        	rcm.set_RRT_finished(255);
        	print(rcm.get_RRT_finished())
		return'grasping'

	end

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
    -- Close rgrip
--  Grip_hold = {0,0,0}
--  Grip_open = {0.496719, 0.458369, 0.50132}

  print("armRRTbasedmove_exit")
--    Body.set_lgrip_command_position(Grip_hold)  --working one 

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
