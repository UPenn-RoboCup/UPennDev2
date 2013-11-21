-- libArmPlan
-- (c) 2013 Seung-Joon Yi
-- Arm movement planner

local vector = require'vector'
local util = require'util'
require'mcm'
local movearm = require'movearm'

--debug_on = true
debug_on = false
debug_on_2 = false

local function print_transform(tr)
  if not tr then return end
  local str= string.format("%.2f %.2f %.2f (%.1f %.1f %.1f)",
    tr[1],tr[2],tr[3],tr[4]*180/math.pi,tr[5]*180/math.pi,tr[6]*180/math.pi)
  return str
end

local function print_jangle(q)
  local str= string.format("%d %d %d %d %d %d %d", unpack(vector.new(q)*180/math.pi)  )
  return str
end

local function calculate_margin(qArm,isLeft)
  local jointangle_margin
  if not qArm then return -math.huge 
  elseif isLeft==1 then --Left arm
    jointangle_margin = math.min(
      math.abs(qArm[2]-math.pi/2),       --Shoulder Roll 
      math.abs(qArm[2]),
      math.abs(qArm[6]-math.pi/2),        --Wrist Roll
      math.abs(qArm[6]+math.pi/2)            
      )
    if math.abs(qArm[6]) < 10*math.pi/180 then
      jointangle_margin = math.min(jointangle_margin,math.abs(qArm[6])) 
    end
  else --Right arm
    jointangle_margin = math.min(
      math.abs(qArm[2]+math.pi/2),       --Shoulder Roll
      math.abs(qArm[2]),
      math.abs(qArm[6]-math.pi/2), --Wrist Roll
      math.abs(qArm[6]+math.pi/2)      
      )
    if math.abs(qArm[6]) < 10*math.pi/180 then
      jointangle_margin = math.min(jointangle_margin,math.abs(qArm[6])) 
    end
  end

  --Clamp the margin
  jointangle_margin = math.min(
    Config.arm.plan.max_margin,
    jointangle_margin)
  return jointangle_margin
end

local function search_shoulder_angle(self,qArm,trArmNext,isLeft, yawMag, qWaist)


  local step = Config.arm.plan.search_step

  --Calculte the margin for current shoulder yaw angle
  local qArmMaxMargin, qArmNext
  local max_margin = -math.huge  
  local debugmsg=false

  if isLeft>0 then 
    local shoulderYawTarget = self.shoulder_yaw_target_left
    if shoulderYawTarget then
      local shoulderYaw = util.approachTol(qArm[3],shoulderYawTarget, yawMag, 1)
      local qArmNext = Body.get_inverse_larm(qArm,trArmNext, shoulderYaw, mcm.get_stance_bodyTilt(), qWaist)            
      return qArmNext
    end
  else 
    local shoulderYawTarget = self.shoulder_yaw_target_right    
    if shoulderYawTarget then
      local shoulderYaw = util.approachTol(qArm[3],shoulderYawTarget, yawMag, 1)
      local qArmNext = Body.get_inverse_rarm(qArm,trArmNext, shoulderYaw, mcm.get_stance_bodyTilt(), qWaist)
      return qArmNext
    end
  end
 
  for div = -1,1,step do
    local qShoulderYaw = qArm[3] + div * yawMag
    if debugmsg then print("CHECKING SHOULDERYAW ",qShoulderYaw*180/math.pi) end
    local qArmNext
    if isLeft>0 then qArmNext = Body.get_inverse_larm(qArm,trArmNext, qShoulderYaw, mcm.get_stance_bodyTilt(), qWaist)
    else qArmNext = Body.get_inverse_rarm(qArm,trArmNext, qShoulderYaw, mcm.get_stance_bodyTilt(), qWaist) end
    local margin = self.calculate_margin(qArmNext,isLeft)
    if margin>max_margin then
      qArmMaxMargin = qArmNext
      max_margin = margin
    end
  end
  if max_margin<0 then
    print("CANNOT FIND CORRECT SHOULDER ANGLE, at trNext:",self.print_transform(trArmNext))
    return
  else
    if debugmsg then print("arm found with shoulderangle",qArmMaxMargin[3]*180/math.pi) end
    return qArmMaxMargin
  end
end

local function check_arm_joint_velocity(qArm0, qArm1, dt,velLimit)
  --Slow down the total movement time based on joint velocity limit
  velLimit = velLimit or Config.arm.joint_vel_limit_plan
  local qArmMovement = vector.new(qArm1) - vector.new(qArm0);
  local max_movement_ratio = 1
  for i=1,7 do
    local movement_ratio = math.abs(util.mod_angle(qArmMovement[i]))/(velLimit[i]*dt)
    max_movement_ratio = math.max(max_movement_ratio,movement_ratio)
  end
  return max_movement_ratio
end

local function set_hand_mass(self,mLeftHand, mRightHand)
  self.mLeftHand, self.mRightHand = mLeftHand, mRightHand
end

local function reset_torso_comp(self,qLArm,qRArm)
  local qWaist = Body.get_waist_command_position()

  local com = Kinematics.com_upperbody(qWaist,qLArm,qRArm, 
         mcm.get_stance_bodyTilt(), 0,0)

--hack
  --local com = Kinematics.com_upperbody(qWaist,qLArm,qRArm, 
--         Config.walk.bodyTilt, 0,0)


  self.torsoCompBias = {-com[1]/com[4],-com[2]/com[4]}

  mcm.set_stance_uTorsoCompBias(self.torsoCompBias)  
  self:save_boundary_condition({qLArm,qRArm,qLArm,qRArm,{0.0}})
end

local function get_torso_compensation(self,qLArm,qRArm,massL,massR)
  local qWaist = Body.get_waist_command_position() --TODO: dynamic waist movement
  if mcm.get_status_iskneeling()==1 or Config.stance.enable_torso_compensation==0 then
    return {0,0}
  else      
    local com = Kinematics.com_upperbody(qWaist,qLArm,qRArm,mcm.get_stance_bodyTilt(), massL, massR)
    local uTorsoCompBias = mcm.get_stance_uTorsoCompBias()
    local uTorsoCompNew = {-com[1]/com[4]-uTorsoCompBias[1],-com[2]/com[4]-uTorsoCompBias[2]}
    return uTorsoCompNew
  end
end

local function get_next_movement(self, init_cond, trLArm1,trRArm1, dt_step, waistYaw)

  local velTorsoComp = Config.arm.torso_comp_limit
  local velYaw = Config.arm.shoulder_yaw_limit
  local massL, massR = self.mLeftHand, self.mRightHand
  local qLArm,qRArm, qLArmComp , qRArmComp, uTorsoComp = unpack(init_cond)

  local yawMag = dt_step * velYaw
  local qLArmNext, qRArmNext = qLArm, qRArm
  qLArmNext = self:search_shoulder_angle(qLArm,trLArm1,1, yawMag, {waistYaw,Body.get_waist_command_position()[2]})
  qRArmNext = self:search_shoulder_angle(qRArm,trRArm1,0, yawMag, {waistYaw,Body.get_waist_command_position()[2]})

  if not qLArmNext or not qRArmNext then print("ARM PLANNING ERROR")
    return 
  end

  local trLArmNext, trRArmNext = Body.get_forward_larm(qLArmNext),Body.get_forward_rarm(qRArmNext)
  local vec_comp = vector.new({-uTorsoComp[1],-uTorsoComp[2],0,0,0,0})
  local trLArmNextComp = vector.new(trLArmNext) + vec_comp
  local trRArmNextComp = vector.new(trRArmNext) + vec_comp
    
  --Actual arm angle considering the torso compensation
  local qLArmNextComp = self:search_shoulder_angle(qLArmComp,trLArmNextComp,1, yawMag, {waistYaw,Body.get_waist_command_position()[2]})
  local qRArmNextComp = self:search_shoulder_angle(qRArmComp,trRArmNextComp,0, yawMag, {waistYaw,Body.get_waist_command_position()[2]})

  if not qLArmNextComp or not qRArmNextComp or not qLArmNext or not qRArmNext then 
    print("ARM PLANNING ERROR")
    return 
  else
    local max_movement_ratioL = check_arm_joint_velocity(qLArmComp, qLArmNextComp, dt_step)
    local max_movement_ratioR = check_arm_joint_velocity(qRArmComp, qRArmNextComp, dt_step)
    local dt_step_current = dt_step * math.max(max_movement_ratioL,max_movement_ratioR)
    --TODO: WAIST 
    local uTorsoCompNextTarget = self:get_torso_compensation(qLArmNext,qRArmNext,massL,massR)
    local uTorsoCompNext, torsoCompDone = util.approachTol(uTorsoComp, uTorsoCompNextTarget, velTorsoComp, dt_step_current )

    local new_cond = {qLArmNext, qRArmNext, qLArmNextComp, qRArmNextComp, uTorsoCompNext, waistYaw}
    return new_cond, dt_step_current, torsoCompDone    
  end
end


local function plan_unified(self, plantype, init_cond, init_param, target_param)
  --param: {trLArm,trRArm} for move
  --param: {trLArm,trRArm} for wrist


  if not init_cond then return end
  local done, failed = false, false, false

  local current_cond = {init_cond[1],init_cond[2],init_cond[3],init_cond[4],{init_cond[5][1],init_cond[5][2]},
     init_cond[6] or Body.get_waist_command_position()[1] }
  local trLArm, trRArm = Body.get_forward_larm(init_cond[1]), Body.get_forward_rarm(init_cond[2])
  local qLArm0, qRArm0 = init_cond[1],init_cond[2]

  --Insert initial arm joint angle to the queue
  local dt_step0 = Config.arm.plan.dt_step0
  local dt_step = Config.arm.plan.dt_step
  local qLArmQueue,qRArmQueue, uTorsoCompQueue = {{init_cond[3],dt_step0}}, {{init_cond[4],dt_step0}}, {init_cond[5]}
  local qWaistQueue={current_cond[6]}

  local current_param={unpack(init_param)}
  local qArmCount = 2
  local vel_param  

  local dpArmMax = Config.arm.linear_slow_limit  
  local dpWristMax = Config.arm.wrist_turn_limit

  if plantype=="move" then
  elseif plantype=="wrist" then
  elseif plantype=="door" then
    target_param[4] = target_param[4] or current_cond[6]

    local dpDoorMax = vector.slice(Config.arm.linear_slow_limit,1,3)
    local dqDoorRollMax = Config.armfsm.dooropen.velDoorRoll
    local dqDoorYawMax = Config.armfsm.dooropen.velDoorYaw
    local dqWaistYawMax = Config.armfsm.dooropen.velWaistYaw
    vel_param={dpDoorMax, dqDoorRollMax, dqDoorYawMax, dqWaistYawMax}

  elseif plantype=="valvetwoarm" then
    vel_param={
      Config.armfsm.valvetwoarm.velTurnAngle,
      Config.armfsm.valvetwoarm.velTurnAngle,
      Config.armfsm.valvetwoarm.velInsert,
      Config.armfsm.valvetwoarm.velInsert}
  elseif plantype=="valveonearm" then
    vel_param={
      Config.armfsm.valveonearm.velTurnAngle,
      Config.armfsm.valveonearm.velInsert}
  elseif plantype=="barvalve" then        
    vel_param={
      Config.armfsm.valvebar.velTurnAngle,
      Config.armfsm.valvebar.velTurnAngle,
      Config.armfsm.valvebar.velInsert}
  end
  
  local done, torsoCompDone = false, false
  local trLArmNext, trRArmNext, waistNext
  local new_param = {}

  local t0 = unix.time()  

  while not done and not failed  do        
    if plantype=="move" then
      trLArmNext,doneL = util.approachTolTransform(trLArm, target_param[1], dpArmMax, dt_step )
      trRArmNext,doneR = util.approachTolTransform(trRArm, target_param[2], dpArmMax, dt_step )
      waistNext = current_cond[6]
      done = doneL and doneR
    elseif plantype=="wrist" then
      trLArmNext,doneL = util.approachTolWristTransform(trLArm, target_param[1], dpWristMax, dt_step )      
      trRArmNext,doneR = util.approachTolWristTransform(trRArm, target_param[2], dpWristMax, dt_step )
      done = doneL and doneR
      waistNext = current_cond[6]
      local qLArmTemp = Body.get_inverse_arm_given_wrist( qLArm0, trLArmNext)
      local qRArmTemp = Body.get_inverse_arm_given_wrist( qRArm0, trRArmNext)
      trLArmNext = Body.get_forward_larm(qLArmTemp)
      trRArmNext = Body.get_forward_rarm(qRArmTemp)  

    elseif plantype=="door" then
      local done1,done2,done3,done4      
      new_param[1], done1 = util.approachTol(current_param[1],target_param[1], vel_param[1],dt_step)
      new_param[2], done2 = util.approachTol(current_param[2],target_param[2], vel_param[2],dt_step)
      new_param[3], done3 = util.approachTol(current_param[3],target_param[3], vel_param[3],dt_step)
      new_param[4], done4 = util.approachTol(current_param[4],target_param[4], vel_param[4],dt_step)
      done = done1 and done2 and done3 and done4
      trLArmNext = trLArm
      trRArmNext = movearm.getDoorHandlePosition(current_param[1],current_param[2],current_param[3])      
      waistNext = new_param[4]
    elseif plantype =="valvetwoarm" then
      new_param[1], done1 = util.approachTol(current_param[1],target_param[1], vel_param[1],dt_step)
      new_param[2], done2 = util.approachTol(current_param[2],target_param[2], vel_param[2],dt_step)
      new_param[3], done3 = util.approachTol(current_param[3],target_param[3], vel_param[3],dt_step)
      new_param[4], done4 = util.approachTol(current_param[4],target_param[4], vel_param[4],dt_step)
      done = done1 and done2 and done3 and done4
      trLArmNext, trRArmNext =  movearm.getLargeValvePosition(unpack(new_param))
      waistNext = current_cond[6]

    elseif plantype =="valveonearm" then
      new_param[1], done1 = util.approachTol(current_param[1],target_param[1], vel_param[1],dt_step)
      new_param[2], done2 = util.approachTol(current_param[2],target_param[2], vel_param[2],dt_step)
      new_param[3],new_param[4] = target_param[3],target_param[4]      
      done = done1 and done2

      if new_param[3]>0 then --left arm      
        trLArmNext, trRArmNext =  movearm.getLargeValvePositionSingle(unpack(new_param)), trRArm
      else
        trLArmNext, trRArmNext =  trLArm, movearm.getLargeValvePositionSingle(unpack(new_param))
      end
      waistNext = current_cond[6]
    elseif plantype =="barvalve" then
      new_param[1], done1 = util.approachTol(current_param[1],target_param[1], vel_param[1],dt_step)
      new_param[2], done2 = util.approachTol(current_param[2],target_param[2], vel_param[2],dt_step)
      new_param[3], done3 = util.approachTol(current_param[3],target_param[3], vel_param[3],dt_step)
      new_param[4] = target_param[4]      
      done = done1 and done2 and done3

      if new_param[4]>0 then --left arm      
        trLArmNext, trRArmNext =  movearm.getBarValvePositionSingle(unpack(new_param)), trRArm
      else
        trLArmNext, trRArmNext =  trLArm, movearm.getBarValvePositionSingle(unpack(new_param))
      end
      waistNext = current_cond[6]

    end

    local new_cond, dt_step_current, torsoCompDone=    
      self:get_next_movement(current_cond, trLArmNext, trRArmNext, dt_step, waistNext)

    done = done and torsoCompDone
    if not new_cond then 
      print("Plan failure at",self.print_transform(trLArmNext))    
      --TODO: We can just skip singularities
      failed = true    
    else
      trLArm, trRArm = trLArmNext, trRArmNext
      qLArmQueue[qArmCount] = {new_cond[3],dt_step_current}
      qRArmQueue[qArmCount] = {new_cond[4],dt_step_current}
      qWaistQueue[qArmCount] = waistNext 
      uTorsoCompQueue[qArmCount] = {new_cond[5][1],new_cond[5][2]}      
      current_cond = new_cond
      current_param = new_param
      qArmCount = qArmCount + 1
    end    
  end

  local t1 = unix.time()  
  
  if debug_on_2 then
    print(string.format("%d steps planned, %.2f ms elapsed:",qArmCount,(t1-t0)*1000 ))
    print("trLArm:",self.print_transform( Body.get_forward_larm( qLArmQueue[1][1]  ) ))
    print("trRArm:",self.print_transform( Body.get_forward_rarm( qRArmQueue[1][1]  )))
    print(string.format("TorsoComp: %.3f %.3f",uTorsoCompQueue[1][1],uTorsoCompQueue[1][2]) )
  end


  if failed then return
  else return qLArmQueue,qRArmQueue, uTorsoCompQueue, qWaistQueue, current_cond, current_param end
end



local function plan_arm_sequence2(self,arm_seq)
  --This function plans for a arm sequence using multiple arm target positions
  --and initializes the playback if it is possible
  
--  {
--    {'move', trLArm, trRArm}  : move arm transform to target
--    {'wrist', trLArm, trRArm} : rotate wrist to target transform
--    {'valve', }      
--  }

  local init_cond = self:load_boundary_condition()
  local LAPs, RAPs, uTPs, WPs = {},{},{},{}
  local counter = 1

  for i=1,#arm_seq do
    local trLArm = Body.get_forward_larm(init_cond[1])
    local trRArm = Body.get_forward_rarm(init_cond[2])
    local LAP, RAP, uTP,end_cond
    local WP, end_doorparam --for door
    if arm_seq[i][1] =='move' then
      LAP, RAP, uTP, WP, end_cond  = self:plan_unified('move',
        init_cond,
        {trLArm,trRArm},{arm_seq[i][2] or trLArm,arm_seq[i][3] or trRArm} )
    elseif arm_seq[i][1] =='wrist' then

      LAP, RAP, uTP, WP, end_cond  = self:plan_unified('wrist',
        init_cond,
        {trLArm,trRArm},
        {arm_seq[i][2] or trLArm,  arm_seq[i][3] or trRArm} )

    elseif arm_seq[i][1] =='door' then
      LAP, RAP, uTP, WP, end_cond, end_doorparam  = self:plan_unified('door',
        init_cond, 
        self.init_doorparam, 
        vector.slice(arm_seq[i],2,#arm_seq[i]) )

    elseif arm_seq[i][1] =='valvetwoarm' then
      LAP, RAP, uTP, WP, end_cond, end_valveparam  = self:plan_unified('valvetwoarm',
        init_cond, 
        self.init_valveparam, 
        vector.slice(arm_seq[i],2,#arm_seq[i]) )

    elseif arm_seq[i][1] =='valveonearm' then
      LAP, RAP, uTP, WP, end_cond, end_valveparam  = self:plan_unified('valveonearm',
        init_cond, 
        self.init_valveparam, 
        vector.slice(arm_seq[i],2,#arm_seq[i]) )      

    elseif arm_seq[i][1] =='barvalve' then
      LAP, RAP, uTP, WP, end_cond, end_valveparam  = self:plan_unified('barvalve',
        init_cond, 
        self.init_valveparam, 
        vector.slice(arm_seq[i],2,#arm_seq[i]) )      

    end
    if not LAP then 
      hcm.set_state_success(-1) --Report plan failure
      return 
    end
    init_cond = end_cond    
    if end_doorparam then self.init_doorparam = end_doorparam end
    if end_valveparam then self.init_valveparam = end_valveparam end
    
    for j=1,#LAP do
      LAPs[counter],RAPs[counter],uTPs[counter],WPs[counter]= 
        LAP[j],RAP[j],uTP[j],WP[j]
      counter = counter+1
    end
  end

  self:save_boundary_condition(init_cond)
  local arm_plan = {LAP = LAPs, RAP = RAPs,  uTP =uTPs, WP=WPs}
  self:init_arm_sequence(arm_plan,Body.get_time())
  return true
end






local function init_arm_sequence(self,arm_plan,t0)
  if not arm_plan then return end
  self.leftArmQueue = arm_plan.LAP
  self.rightArmQueue = arm_plan.RAP
  self.torsoCompQueue = arm_plan.uTP
  
  self.t_last = t0
  self.armQueuePlayStartTime = t0
  self.armQueuePlayEndTime = t0 + self.leftArmQueue[2][2]

  self.qLArmStart = self.leftArmQueue[1][1]
  self.qLArmEnd = self.leftArmQueue[2][1]
  self.qRArmStart = self.rightArmQueue[1][1]
  self.qRArmEnd = self.rightArmQueue[2][1]
  self.uTorsoCompStart=vector.new(self.torsoCompQueue[1])
  self.uTorsoCompEnd=vector.new(self.torsoCompQueue[2])

  if arm_plan.WP then
    self.waistQueue = arm_plan.WP
    self.waistStart = self.waistQueue[1]
    self.waistEnd = self.waistQueue[2]
  else
    self.waistQueue = nil
  end
  self.armQueuePlaybackCount = 2


  self:print_segment_info() 
end

local function print_segment_info(self)
  if debug_on then
    print(string.format("%d uTC: %.3f %.3f to %.3f %.3f, t=%.2f",
    self.armQueuePlaybackCount,
    self.uTorsoCompStart[1],self.uTorsoCompStart[2],
    self.uTorsoCompEnd[1],self.uTorsoCompEnd[2],
    self.armQueuePlayEndTime - self.armQueuePlayStartTime 
     ))
  end
end

local function play_arm_sequence(self,t)
  local dt =  t - self.t_last
  self.t_last = t
  if #self.leftArmQueue < self.armQueuePlaybackCount then
    return true
  else
   --Skip keyframes if needed
    while t>self.armQueuePlayEndTime do        
      self.armQueuePlaybackCount = self.armQueuePlaybackCount +1        
      if #self.leftArmQueue < self.armQueuePlaybackCount then
        --Passed the end of the queue. return the last joint angle
        return 
          self.leftArmQueue[#self.leftArmQueue][1],
          self.rightArmQueue[#self.leftArmQueue][1],
          self.torsoCompQueue[#self.leftArmQueue]
      end
      --Update the frame start and end time
      self.armQueuePlayStartTime = self.armQueuePlayEndTime        
      self.armQueuePlayEndTime = self.armQueuePlayStartTime + 
          self.leftArmQueue[self.armQueuePlaybackCount][2]
      --Update initial and final joint angle
      self.qLArmStart = vector.new(self.leftArmQueue[self.armQueuePlaybackCount-1][1])
      self.qLArmEnd = vector.new(self.leftArmQueue[self.armQueuePlaybackCount][1])
      self.qRArmStart = vector.new(self.rightArmQueue[self.armQueuePlaybackCount-1][1])
      self.qRArmEnd = vector.new(self.rightArmQueue[self.armQueuePlaybackCount][1])
      self.uTorsoCompStart = vector.new(self.torsoCompQueue[self.armQueuePlaybackCount-1])
      self.uTorsoCompEnd = vector.new(self.torsoCompQueue[self.armQueuePlaybackCount])

      if self.waistQueue then
        self.waistStart = self.waistQueue[self.armQueuePlaybackCount-1]
        self.waistEnd = self.waistQueue[self.armQueuePlaybackCount]
      end

      self:print_segment_info()      
    end
    local ph = (t-self.armQueuePlayStartTime)/ 
              (self.armQueuePlayEndTime-self.armQueuePlayStartTime)
    local qLArm,qRArm,qWaist={},{}
    for i=1,7 do
      qLArm[i] = self.qLArmStart[i] + ph * (util.mod_angle(self.qLArmEnd[i]-self.qLArmStart[i]))
      qRArm[i] = self.qRArmStart[i] + ph * (util.mod_angle(self.qRArmEnd[i]-self.qRArmStart[i]))
    end
    local uTorsoComp = (1-ph)*self.uTorsoCompStart + ph*self.uTorsoCompEnd


    --Update transform information
    local trLArmComp = Body.get_forward_larm(qLArm)
    local trRArmComp = Body.get_forward_rarm(qRArm)
    local trLArm = vector.new(trLArmComp)+
              vector.new({uTorsoComp[1],uTorsoComp[2],0, 0,0,0})
    local trRArm = vector.new(trRArmComp)+
              vector.new({uTorsoComp[1],uTorsoComp[2],0, 0,0,0})
    hcm.set_hands_left_tr(trLArm)
    hcm.set_hands_right_tr(trRArm)
    hcm.set_hands_left_tr_target(trLArm)
    hcm.set_hands_right_tr_target(trRArm)

    --Move joints
    movearm.setArmJoints(qLArm,qRArm,dt)
    mcm.set_stance_uTorsoComp(uTorsoComp)    

    if self.waistQueue then
      qWaist = self.waistStart + ph * (self.waistEnd - self.waistStart)
      Body.set_waist_command_position({qWaist,Body.get_waist_command_position()[2]})
    end
  end
  return false
end

local function save_boundary_condition(self,arm_end)
  mcm.set_arm_qlarm(arm_end[1])
  mcm.set_arm_qrarm(arm_end[2])        
  mcm.set_arm_qlarmcomp(arm_end[3])
  mcm.set_arm_qrarmcomp(arm_end[4])
end

local function load_boundary_condition(self)
  local qLArm=mcm.get_arm_qlarm()
  local qRArm=mcm.get_arm_qrarm()        
  local qLArmComp=mcm.get_arm_qlarmcomp()
  local qRArmComp=mcm.get_arm_qrarmcomp()
  local uTorsoComp = mcm.get_stance_uTorsoComp()
  local init_cond = {qLArm,qRArm,qLArmComp,qRArmComp,uTorsoComp}
  return init_cond
end

local function save_doorparam(self,doorparam)
  self.init_doorparam=doorparam
end

local function save_valveparam(self,valveparam)
  self.init_valveparam=valveparam
end

local function set_shoulder_yaw_target(self,left,right)
  self.shoulder_yaw_target_left = left
  self.shoulder_yaw_target_right = right
end

local libArmPlan={}

libArmPlan.new_planner = function (params)

  params = params or {}
  local s = {}
  --member variables
  s.armQueue = {}
  s.armQueuePlaybackCount = 1
  s.armQueuePlayStartTime = 0
  s.mLeftHand = 0
  s.mRightHand = 0
  s.shoulder_yaw_target_left = nil
  s.shoulder_yaw_target_right = nil

  s.torsoCompBias = {0,0}



  s.leftArmQueue={}
  s.rightArmQueue={}
  s.torsoCompQueue={}
  s.waistQueue={}

  s.init_cond = {}
  s.current_plan = {}
  s.current_endcond = {}

  s.init_doorparam = {}
  s.init_valveparam = {0,0,0,0}

  --member functions
  s.print_transform = print_transform
  s.print_jangle = print_jangle
  s.print_segment_info = print_segment_info

  s.calculate_margin = calculate_margin
  s.search_shoulder_angle = search_shoulder_angle  
  s.get_torso_compensation = get_torso_compensation
  s.set_hand_mass = set_hand_mass
  s.check_arm_joint_velocity = check_arm_joint_velocity
  s.reset_torso_comp = reset_torso_comp
  s.get_next_movement = get_next_movement

  s.plan_arm_sequence = plan_arm_sequence2
  s.plan_arm_sequence2 = plan_arm_sequence2  
  s.plan_unified = plan_unified

  s.init_arm_sequence = init_arm_sequence
  s.play_arm_sequence = play_arm_sequence

  s.save_boundary_condition=save_boundary_condition
  s.load_boundary_condition=load_boundary_condition

  s.save_doorparam = save_doorparam  
  s.save_valveparam = save_valveparam
  s.set_shoulder_yaw_target = set_shoulder_yaw_target

  return s
end

return libArmPlan
