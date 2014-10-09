-- ZMP Preview library
-- Using the old lua matrix library (for now)
-- (c) 2013 Stephen McGill, Seung-Joon Yi
local vector = require'vector'
local util   = require'util'
local matrix = require('matrix_zmp')
require'mcm'
require'Config'
--We maintain Two queue for preview duration

--ZMP position queue : uSupport
--Step status queue : uLeft_now, uRight_now, uLeft_next,uRight_next,supportLeg


--Initialize a stationary preview queue
local function init_preview_queue(self,uLeft,uRight,uTorso,t,step_planner)
  --[[
  local uLSupport,uRSupport = step_planner.get_supports(uLeft,uRight)
  local uSupport = util.se2_interpolate(0.5, uLSupport, uRSupport)
  --]]

  local uSupport = uTorso

  self.x = matrix:new{{uTorso[1],uTorso[2]},{0,0},{0,0}}
  
  self.preview_interval = self.preview_steps * self.preview_tStep

  self.preview_queue={}
  self.preview_queue_zmpx={}
  self.preview_queue_zmpy={}

  self.zmp_current={}
  for i=1,self.preview_steps do
    local preview_item = {}    
    preview_item.uLeft_now = uLeft
    preview_item.uLeft_next = uLeft
    preview_item.uRight_now = uRight
    preview_item.uRight_next = uRight
    preview_item.supportLeg = 2 --Double support

    --for Trapezoid ZMP curve 
    preview_item.uSupport_now = step_planner.get_torso(uLeft,uRight);
    preview_item.uSupport_target = step_planner.get_torso(uLeft,uRight);
    preview_item.uSupport_next = step_planner.get_torso(uLeft,uRight);

    preview_item.tStart = t
    preview_item.tEnd = t + self.preview_interval

    self.preview_queue[i] = preview_item
    self.preview_queue_zmpx[i] = uSupport[1]
    self.preview_queue_zmpy[i] = uSupport[2]

    mcm.set_stance_last_support(uSupport)        
  end  
  self.is_estopping = false
end

local function get_current_step_info(self,t)
  local current_info = self.preview_queue[1]
  local ph = 0
  --if current_info.supportLeg<2 then
    ph =  (t-current_info.tStart)/
          (current_info.tEnd-current_info.tStart)
  --end
  return 
    current_info.uLeft_now,
    current_info.uRight_now,
    current_info.uLeft_next,
    current_info.uRight_next,    
    current_info.supportLeg,
    ph,
    current_info.ended,
    current_info.stepParams,
    current_info.is_last
end

local function update_preview_queue_velocity(self,step_planner,t,stoprequest)
  local supportLeg
  local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next

  table.remove(self.preview_queue,1)
  table.remove(self.preview_queue_zmpx,1)
  table.remove(self.preview_queue_zmpy,1)

  local last_preview_item = self.preview_queue[#self.preview_queue]
  local last_preview_zmpx = self.preview_queue_zmpx[#self.preview_queue]
  local last_preview_zmpy = self.preview_queue_zmpy[#self.preview_queue]
  local new_preview_item = {}

  if last_preview_item.tEnd >= t + self.preview_interval then  --Old step
    table.insert(self.preview_queue,last_preview_item)
--[[    
    table.insert(self.preview_queue_zmpx,last_preview_zmpx)
    table.insert(self.preview_queue_zmpy,last_preview_zmpy)
--]]
--
  local  ph = (t+self.preview_interval-last_preview_item.tStart)/(last_preview_item.tEnd-last_preview_item.tStart);
    
    --for Trapezoid ZMP curve handling
    if ph<Config.walk.phZmp[1] then
      self.zmp_current = util.se2_interpolate(
        (ph/Config.walk.phZmp[1]),
        last_preview_item.uSupport_now,
        last_preview_item.uSupport_target
        )

    elseif ph>Config.walk.phZmp[2] then
      self.zmp_current = util.se2_interpolate(
        (1-ph)/(1-Config.walk.phZmp[2]),
        last_preview_item.uSupport_next,
        last_preview_item.uSupport_target
        )
    else
      self.zmp_current = last_preview_item.uSupport_target;
    end
    table.insert(self.preview_queue_zmpx,self.zmp_current[1])
    table.insert(self.preview_queue_zmpy,self.zmp_current[2])
--    
  elseif stoprequest==0 then --New step
    if last_preview_item.supportLeg==2 then supportLeg = 0 
    else supportLeg = 1-last_preview_item.supportLeg  end

    step_planner:update_velocity(mcm.get_walk_vel())

    uLeft_now, uRight_now, uTorso_now, uLeft_next,
      uRight_next, uTorso_next, uSupport = step_planner:get_next_step_velocity(
        last_preview_item.uLeft_next,
        last_preview_item.uRight_next,
        step_planner.get_torso(
          last_preview_item.uLeft_next,last_preview_item.uRight_next),
        supportLeg,false)
    
    new_preview_item.uLeft_now = uLeft_now
    new_preview_item.uLeft_next = uLeft_next
    new_preview_item.uRight_now = uRight_now
    new_preview_item.uRight_next = uRight_next
    new_preview_item.supportLeg = supportLeg
    new_preview_item.tStart = last_preview_item.tEnd
    new_preview_item.tEnd = last_preview_item.tEnd + self.tStep

    table.insert(self.preview_queue,new_preview_item)

    new_preview_item.uSupport_now = step_planner.get_torso(uLeft_now,uRight_now)
    new_preview_item.uSupport_target = uSupport    
    new_preview_item.uSupport_next = step_planner.get_torso(uLeft_next,uRight_next)

    self.zmp_current = new_preview_item.uSupport_now
    table.insert(self.preview_queue_zmpx,new_preview_item.uSupport_now[1])
    table.insert(self.preview_queue_zmpy,new_preview_item.uSupport_now[2])

  else --Stop requested, insert double support
    new_preview_item.uLeft_now = last_preview_item.uLeft_next
    new_preview_item.uLeft_next = last_preview_item.uLeft_next
    new_preview_item.uRight_now = last_preview_item.uRight_next
    new_preview_item.uRight_next = last_preview_item.uRight_next
    new_preview_item.supportLeg = 2 --Double support
    new_preview_item.tStart = last_preview_item.tEnd
    new_preview_item.tEnd = last_preview_item.tEnd + self.preview_tStep
    uSupport = step_planner.get_torso(
        last_preview_item.uLeft_next,last_preview_item.uRight_next)

    new_preview_item.uSupport_now = uSupport;    
    new_preview_item.uSupport_target = uSupport;    
    new_preview_item.uSupport_next = uSupport;    
    self.zmp_current = uSupport;
         
    table.insert(self.preview_queue,new_preview_item)
    table.insert(self.preview_queue_zmpx,uSupport[1])
    table.insert(self.preview_queue_zmpy,uSupport[2])
  end
end



local function update_preview_queue_steps(self,step_planner,t)
  local t_future = t + self.preview_interval
  if self.is_estopping then
    self:update_preview_queue_estop(step_planner,t)
    return
  end

  table.remove(self.preview_queue,1)
  table.remove(self.preview_queue_zmpx,1)
  table.remove(self.preview_queue_zmpy,1)

  local last_preview_item = self.preview_queue[#self.preview_queue]
  local last_preview_zmpx = self.preview_queue_zmpx[#self.preview_queue]
  local last_preview_zmpy = self.preview_queue_zmpy[#self.preview_queue]

  if last_preview_item.tEnd >= t_future then --Old step
    if last_preview_item.trapezoidparams then --moving support
      local trapezoidparams = last_preview_item.trapezoidparams
      table.insert(self.preview_queue,last_preview_item)
      local t_passed = t_future - last_preview_item.tStart
      if t_passed < trapezoidparams[1] then
        local uSupportCurrent = util.se2_interpolate(
          t_passed/trapezoidparams[1],
          last_preview_item.uSupport0,
          last_preview_item.uSupport1
          )
        table.insert(self.preview_queue_zmpx,uSupportCurrent[1])
        table.insert(self.preview_queue_zmpy,uSupportCurrent[2])
      elseif t_passed < trapezoidparams[1]+trapezoidparams[2] then
        table.insert(self.preview_queue_zmpx,last_preview_item.uSupport1[1])
        table.insert(self.preview_queue_zmpy,last_preview_item.uSupport1[2])
      else
        local uSupportCurrent = util.se2_interpolate(
          (t_passed-trapezoidparams[1]-trapezoidparams[2])/trapezoidparams[3],
          last_preview_item.uSupport1,
          last_preview_item.uSupport2
          )
        table.insert(self.preview_queue_zmpx,uSupportCurrent[1])
        table.insert(self.preview_queue_zmpy,uSupportCurrent[2])
      end
    else --fixed support
      table.insert(self.preview_queue,last_preview_item)
      table.insert(self.preview_queue_zmpx,last_preview_zmpx)
      table.insert(self.preview_queue_zmpy,last_preview_zmpy)
    end
  else --New step    
    local supportLeg, tStep, uSupport, stepParams, trapezoidparams
    local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next

      uLeft_now, uRight_now, uTorso_now, 
      uLeft_next, uRight_next, uTorso_next,
      uSupport, supportLeg, tStep, 
      stepParams, is_last, trapezoidparams,
      uSupport_next
            = step_planner:get_next_step_queue(
            last_preview_item.uLeft_next,
            last_preview_item.uRight_next,
            step_planner.get_torso(
              last_preview_item.uLeft_next,last_preview_item.uRight_next),
            false, --initial_step
            {last_preview_zmpx,last_preview_zmpy,0} --uSupport_now
            )

    local new_preview_item = {}
    if not uLeft_now then --No more footsteps            

      new_preview_item.uLeft_now = last_preview_item.uLeft_next
      new_preview_item.uLeft_next = last_preview_item.uLeft_next
      new_preview_item.uRight_now = last_preview_item.uRight_next
      new_preview_item.uRight_next = last_preview_item.uRight_next
      new_preview_item.supportLeg = 2 --Double support
      new_preview_item.tStart = last_preview_item.tEnd
      new_preview_item.tEnd = last_preview_item.tEnd + self.preview_tStep
      new_preview_item.ended = true

      uSupport = step_planner.get_torso(
            last_preview_item.uLeft_next,last_preview_item.uRight_next)
      
      if mcm.get_walk_stoprequest()==0 then
        if last_preview_item.uSupport2 then
          uSupport = last_preview_item.uSupport2
          new_preview_item.uSupport0 = uSupport
          new_preview_item.uSupport1 = uSupport
          new_preview_item.uSupport2 = uSupport
          new_preview_item.trapezoidparams = trapezoidparams        
        end
      else

      end
    else -- We have footstep to execute
      
      new_preview_item.uLeft_now = uLeft_now
      new_preview_item.uLeft_next = uLeft_next
      new_preview_item.uRight_now = uRight_now
      new_preview_item.uRight_next = uRight_next
      new_preview_item.supportLeg = supportLeg
      new_preview_item.tStart = last_preview_item.tEnd
      new_preview_item.tEnd = last_preview_item.tEnd + tStep
      new_preview_item.stepParams = stepParams
      new_preview_item.is_last = is_last

      if trapezoidparams then
        new_preview_item.trapezoidparams = trapezoidparams        
        local last_support = mcm.get_stance_last_support()        
        new_preview_item.uSupport0 = last_support
        new_preview_item.uSupport1 = uSupport
        new_preview_item.uSupport2 = uSupport_next

--[[
        if last_preview_item.uSupport2 then
          new_preview_item.uSupport0 = last_preview_item.uSupport2
        else
          new_preview_item.uSupport0 = uTorso_now
        end
--]]        

        --DS phase and not stopping - we keep the target support till the end
        if new_preview_item.supportLeg==2 and mcm.get_walk_stoprequest()==0 then
          new_preview_item.uSupport2 = uSupport
        end      

--[[
        print("New step")
        print(string.format("Last support: %.3f %.3f\nSupport0: %.3f %.3f\n Support1: %.3f %.3f\n Support2: %.3f %.3f\n",
          last_support[1],last_support[2],
          new_preview_item.uSupport0[1],new_preview_item.uSupport0[2],
          new_preview_item.uSupport1[1],new_preview_item.uSupport1[2],
          new_preview_item.uSupport2[1],new_preview_item.uSupport2[2]
          ))
--]]

        mcm.set_stance_last_support(new_preview_item.uSupport2)        

      end
    end

    table.insert(self.preview_queue,new_preview_item)
    if new_preview_item.trapezoidparams then --moving support      
      table.insert(self.preview_queue_zmpx,uTorso_now[1])
      table.insert(self.preview_queue_zmpy,uTorso_now[2])
    else --Fixed support
      table.insert(self.preview_queue_zmpx,uSupport[1])
      table.insert(self.preview_queue_zmpy,uSupport[2])
    end
  end
end

local function emergency_stop(self,step_planner,t)
  --Find the end time of current step
  local current_info = self.preview_queue[1]
  local future_index = math.floor( (current_info.tEnd-t)/ self.preview_tStep + 0.999)

--[[
  print("Current discrete t:",t)
  print("Current step start time:",current_info.tStart)
  print("Current step end time:",current_info.tEnd)
  print("Current tStep :",self.preview_tStep)
  print("future index:",future_index)
--]]
  --Insert the DS step
  local new_preview_item = {}  
  local last_preview_item = self.preview_queue[1]

  new_preview_item.uLeft_now = last_preview_item.uLeft_next
  new_preview_item.uLeft_next = last_preview_item.uLeft_next
  new_preview_item.uRight_now = last_preview_item.uRight_next
  new_preview_item.uRight_next = last_preview_item.uRight_next
  new_preview_item.supportLeg = 2 --Double support
  new_preview_item.tStart = last_preview_item.tEnd
--  new_preview_item.tEnd = last_preview_item.tEnd + self.preview_tStep
--  new_preview_item.tEnd = last_preview_item.tEnd + self.preview_tStep
  new_preview_item.tEnd = last_preview_item.tEnd + 4.0
  new_preview_item.ended = true
  uSupport = step_planner.get_torso(
     last_preview_item.uLeft_next,last_preview_item.uRight_next)
  self.preview_queue[2] = new_preview_item

  hcm.set_motion_estop(0)
  self.is_estopping = true

  --Wipe out the zmp position queue after current step
  for i=future_index, #self.preview_queue_zmpx do
    self.preview_queue_zmpx[i]=uSupport[1]
    self.preview_queue_zmpy[i]=uSupport[2]
  end

end

local function update_preview_queue_estop(self,step_planner,t)
  --Now we should have 1 or 2 preview items in the queue
  -- last step (SS), DS 
  -- DS

  local first_preview_item = self.preview_queue[1]
  if first_preview_item.ended then --now we're executing the last DS step
    --We don't need to do anything now 
  else --We still have to execute the last SS step
    if t<first_preview_item.tEnd then 
      --execute step
   
    else --advance to the DS step
      table.remove(self.preview_queue,1)
    end
  end

  --Update zmp xy array
  table.remove(self.preview_queue_zmpx,1)
  table.remove(self.preview_queue_zmpy,1)
  uSupport = step_planner.get_torso(
      first_preview_item.uLeft_next,first_preview_item.uRight_next)
  table.insert(self.preview_queue_zmpx,uSupport[1])
  table.insert(self.preview_queue_zmpy,uSupport[2])
end



local function trim_preview_queue(self,step_planner,t0)
  --Trim initial nPreview-1 entries of preview queue
  --So that we can start motion without delay
  local t_discrete = t0
  for i=1,self.preview_steps-1 do
    self:update_preview_queue_steps(step_planner,t_discrete)
    t_discrete = t_discrete + self.preview_tStep
  end
  return (self.preview_steps-1)*self.preview_tStep --return how much we have to shift the time
end

local function get_zmp(self)  
  local zmp={self.preview_queue_zmpx[1],
          self.preview_queue_zmpy[1],
          0}
  return zmp
end

local function update_state(self)

--  Update state variable
--  u = param_k1_px * x - param_k1* zmparray; --Control output
--  x = param_a * x + param_b * u;
  local x = self.x
  local ux =  self.param_k1_px[1][1] * x[1][1]+
              self.param_k1_px[1][2] * x[2][1]+
              self.param_k1_px[1][3] * x[3][1]

  local uy =  self.param_k1_px[1][1] * x[1][2]+
              self.param_k1_px[1][2] * x[2][2]+
              self.param_k1_px[1][3] * x[3][2]

  for i=1,self.preview_steps do
    ux = ux - self.param_k1[1][i]*self.preview_queue_zmpx[i]
    uy = uy - self.param_k1[1][i]*self.preview_queue_zmpy[i]
  end
  local x_next = self.param_a*x + self.param_b * matrix:new({{ux,uy}})
  self.x = x_next
--  return vector.new({x_next[1][1],x_next[1][2]}) --Torso XY

  return {x_next[1][1],x_next[1][2]}, --COM
          {self.preview_queue_zmpx[1],self.preview_queue_zmpy[1]} --ZMP
end

local function can_stop(self)
  local x = self.x
  local current_info = self.preview_queue[1]
  local com_velocity_threshold = 0.001
  if current_info.supportLeg==2 and
    math.sqrt(x[2][1]*x[2][1] + x[2][2]*x[2][2])<com_velocity_threshold then
  return true
  else return false
  end  
end

local function save_param(self)
  local outfile=assert(io.open(HOME.."/Data/zmpparams.lua","w")); 
  local data=''
  data=data..string.format("-- tZmp: %.2f\n",self.tZmp)
  data=data..string.format("zmpstep.params = true;\n")
  data=data..string.format("zmpstep.param_k1_px={%f,%f,%f}\n",
   self.param_k1_px[1][1],self.param_k1_px[1][2],self.param_k1_px[1][3]);
  data=data..string.format("zmpstep.param_a={\n");
  for i=1,3 do
    data=data..string.format("  {%f,%f,%f},\n",
  self.param_a[i][1],self.param_a[i][2],self.param_a[i][3]);
  end
  data=data..string.format("}\n");
  data=data..string.format("zmpstep.param_b={%f,%f,%f,%f}\n",
    self.param_b[1][1],self.param_b[2][1],self.param_b[3][1],self.param_b[4][1]);
  data=data..string.format("zmpstep.param_k1={\n    ");

  for i=1,self.preview_steps do
    data=data..string.format("%f,",self.param_k1[1][i]);
    if i%10==0 then   data=data.."\n    " ;end
  end
  data=data..string.format("}\n");
  outfile:write(data);
  outfile:flush();
  outfile:close();
end

local function precompute(self)
  ------------------------------------
  --We only need following parameters
  -- param_k1_px : 1x3
  -- param_k1 : 1xnPreview 
  -- param_a : 3x3
  -- param_b : 4x1
  ------------------------------------

  local timeStep = self.preview_tStep
  self.param_a=matrix {{1,timeStep,timeStep^2/2},{0,1,timeStep},{0,0,1}}
  self.param_b=matrix.transpose({{timeStep^3/6, timeStep^2/2, timeStep,timeStep}})  
  
  if Config.zmpstep.params then
    self.param_k1_px = matrix:new({Config.zmpstep.param_k1_px});
    self.param_k1 = matrix:new({Config.zmpstep.param_k1});
--    self.param_a = matrix:new(Config.zmpstep.param_a);
--    self.param_b = matrix.transpose(matrix:new({Config.zmpstep.param_b}));
    print("ZMP parameters loaded")
  else
    print("Generating ZMP parameters")
    local timeStep = self.preview_tStep
    local tZmp = self.tZmp
    local nPreview = self.preview_steps
    local r_q = self.r_q

    local px,pu0,pu = {}, {}, {}
    for i=1, nPreview do
      px[i]={1, i*timeStep, i*i*timeStep*timeStep/2 - tZmp*tZmp}
      pu0[i]=(1+3*(i-1)+3*(i-1)^2)/6 *timeStep^3 - timeStep*tZmp*tZmp
      pu[i]={}
      for j=1, nPreview do pu[i][j]=0 end
      for j0=1,i do
        j = i+1-j0
        pu[i][j]=pu0[i-j+1]
      end
    end
    local param_pu = matrix:new(pu)
    local param_px = matrix:new(px)
    local param_pu_trans = matrix.transpose(param_pu)
    local param_a=matrix {{1,timeStep,timeStep^2/2},{0,1,timeStep},{0,0,1}}
    local param_b=matrix.transpose({{timeStep^3/6, timeStep^2/2, timeStep,timeStep}})
    local param_eye = matrix:new(nPreview,"I")
    local param_k=-matrix.invert(
        (param_pu_trans * param_pu) + (r_q*param_eye)
        )* param_pu_trans
    local k1={};
    k1[1]={};
    for i=1,nPreview do k1[1][i]=param_k[1][i] end
    local param_k1 = matrix:new(k1)
    local param_k1_px = param_k1 * param_px

    self.param_k1_px = param_k1_px
    self.param_k1 = param_k1
--    self.param_a = param_a
--    self.param_b = param_b

    self:save_param()
    print("ZMP parameters saved")
  end
end

-- Begin the library code
local libZMP = {}
-- Make a new solver with certain parameters
-- You can update these paramters on the fly, of course
libZMP.new_solver = function( params )
  params = params or {}
	local s = {}

  s.preview_interval = Config.zmpstep.preview_interval
  s.preview_tStep = Config.zmpstep.preview_tStep
  s.preview_steps = s.preview_interval / s.preview_tStep  
  s.r_q = Config.zmpstep.param_r_q

  s.x = matrix:new{{0,0},{0,0},{0,0}}
  s.preview_queue={}
  s.preview_queue_zmpx={}
  s.preview_queue_zmpy={}

  s.tStep = params.tStep or 1 --Walk tStep
  s.tZmp  = params.tZMP or .25
  

  s.init_preview_queue = init_preview_queue
  s.get_current_step_info = get_current_step_info
  s.update_preview_queue_velocity = update_preview_queue_velocity
  s.update_preview_queue_steps = update_preview_queue_steps
  s.update_state = update_state
  s.precompute = precompute
  s.save_param = save_param
  s.get_zmp = get_zmp
  s.trim_preview_queue = trim_preview_queue

  s.can_stop = can_stop
  s.emergency_stop = emergency_stop
  s.update_preview_queue_estop = update_preview_queue_estop
  s.is_estopping = false
  
	return s
end

return libZMP
