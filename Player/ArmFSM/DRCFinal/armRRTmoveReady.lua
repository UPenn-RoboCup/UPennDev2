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


require'rcm'


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

  dt = dt
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

   stage = 1
  Rrt_Run = 0
   --rcm.set_RRT_finished(0);

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()


--print(string.format("qLArmr: %.2f %.2f %.2f %.2f %.2f %.2f %.2f" ,
--unpack(qLArm*RAD_TO_DEG)
--))

 
--qLArmTarget = vector.new({90,0,0,0,0,0,0})*DEG_TO_RAD
--qRArmTarget = vector.new({90,0,0,0,0,0, 0})*DEG_TO_RAD

qLArmTarget = qLArm
qRArmTarget = qRArm

  -- Open rgrip
  Grip_hold = {0,0,0}
  Grip_open = {0.496719, 0.458369, 0.50132}

  --Body.set_lgrip_torque_enable(1)
  --Body.set_lgrip_command_position(Grip_open)
  --Body.set_lgrip_command_position(Grip_hold)

  print("Wait RRT Path")


end


function state.update()
--  print("jinwook jinwook")
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
--  print(state._NAME..' Update' )
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local finished = 0;   --RRT

--    print("Jinwook Jinwook Jinwook")

  if Rrt_Run == 0 then
      
      -- used saved path for the RRT
      --[[
      Rrt_Run, RRT_Path = run_RRT_code()
    --state.jnwook_test()
      RRT_Path_size = table.getn(RRT_Path);
      ]]

      RRT_idx = 1;


    -- receive RRT path from MATLAB
    
  --    print("rrt data is received\n")
      local t_plane = rrt_ch:receive(true)

      print("t_plane is ")
      print(t_plane)
      print("this")
      --print(t_plane)
      if t_plane then
        rcm.set_RRT_finished(0);
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

        head_vector = {}
        head_vector[1] =  planes[1]
        head_vector[2] =  planes[2]
        head_vector[3] =  planes[3]
        head_vector[4] =  planes[4]
        head_vector[5] =  planes[5]
        head_vector[6] =  planes[6]
        head_vector[7] =  planes[7]
            
        print(head_vector[1], head_vector[2], head_vector[3])

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
            tmp_vector[1] =  planes[(idx)*7+1]
            tmp_vector[2] =  planes[(idx)*7+2]
            tmp_vector[3] =  planes[(idx)*7+3]
            tmp_vector[4] =  planes[(idx)*7+4]
            tmp_vector[5] =  planes[(idx)*7+5]
            tmp_vector[6] =  planes[(idx)*7+6]
            tmp_vector[7] =  planes[(idx)*7+7]
            tmp_RRT_l_Path[idx] = tmp_vector;
        end
        --0  -60.0000         0         0   80.0000         0
        RRT_l_Path = tmp_RRT_l_Path;


        tmp_RRT_r_Path = {}
        for idx = (RRT_Path_l_size+1), (RRT_Path_l_size+RRT_Path_r_size) do
            tmp_vector = {}
            tmp_vector[1] =  planes[(idx)*7+1]
            tmp_vector[2] =  planes[(idx)*7+2]
            tmp_vector[3] =  planes[(idx)*7+3]
            tmp_vector[4] =  planes[(idx)*7+4]
            tmp_vector[5] =  planes[(idx)*7+5]
            tmp_vector[6] =  planes[(idx)*7+6]
            tmp_vector[7] =  planes[(idx)*7+7]
            tmp_RRT_r_Path[idx-RRT_Path_l_size] = tmp_vector;
        end
        --0  -60.0000         0         0   80.0000         0
        RRT_r_Path = tmp_RRT_r_Path;



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

         print(RRT_r_idx)
         print(RRT_l_idx)
         qRArmTarget[1] = (RRT_r_Path[RRT_r_idx][2])*3.141592/180;
         qRArmTarget[2] = -1*(RRT_r_Path[RRT_r_idx][3])*3.141592/180;
         qRArmTarget[3] = -1*(-1*RRT_r_Path[RRT_r_idx][4])*3.141592/180;
         qRArmTarget[4] = (RRT_r_Path[RRT_r_idx][5])*3.141592/180;
         qRArmTarget[5] = -1*(-1*RRT_r_Path[RRT_r_idx][6]+90)*3.141592/180;
         qRArmTarget[6] = -1*(-1*RRT_r_Path[RRT_r_idx][7])*3.141592/180;
         qRArmTarget[7] = RRT_r_end_effector+3.141592/2;


         qLArmTarget[1] = (RRT_l_Path[RRT_l_idx][2])*3.141592/180;
         qLArmTarget[2] = (RRT_l_Path[RRT_l_idx][3])*3.141592/180;
         qLArmTarget[3] = (-1*RRT_l_Path[RRT_l_idx][4])*3.141592/180;
         qLArmTarget[4] = (RRT_l_Path[RRT_l_idx][5])*3.141592/180;
         qLArmTarget[5] = (-1*RRT_l_Path[RRT_l_idx][6]+90)*3.141592/180;
         qLArmTarget[6] = (-1*RRT_l_Path[RRT_l_idx][7])*3.141592/180;
         qLArmTarget[7] = RRT_l_end_effector+3.141592/2;
        -- qLArmTarget[7] = 0;

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



          print(RRT_idx);
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
          print ("RRT ended")
          finished = 1;
          Rrt_Run = 0;
  --        return'teleopraw'
          print("shared")
          print(rcm.get_RRT_finished())
          rcm.set_RRT_finished(255);
          print(rcm.get_RRT_finished())

          return'done'

      end

    end

    if finished == 1 then
    end

end


function state.exit()
  print(state._NAME..' Exit')
  -- Open rgrip
  Grip_hold = {0,0,0}
  Grip_open = {0.496719, 0.458369, 0.50132}


  print("armRRTmoveReady_exit")
  --Body.set_lgrip_command_position(Grip_hold)
  --Body.set_lgrip_command_position(Grip_open)


end

return state
