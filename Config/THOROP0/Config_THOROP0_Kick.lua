assert(Config, 'Need a pre-existing Config table!')
--print("Robot hostname:",HOSTNAME)

--THOR mk2 specific walk config
--Only one robot (chipettes)
local vector = require'vector'

local kick = {}



local walk={}
walk.tStep = 0.80
walk.stepHeight = 0.03 --hack
walk.phSingle = {0.2,0.8}

local tSlope1 = walk.tStep*walk.phSingle[1]
local tSlope2 = walk.tStep*(1-walk.phSingle[2])
local tStepMid =walk.tStep-tSlope1-tSlope2




local height1 = 0.03 --step height
local height2 = 0.06 --walkkick height
local height3 = 0.04 --longkick height





kick.stepqueue={}


--Walkkick #1

kick.stepqueue["LeftKick0"]=
  {
    {{0.12,0,0},0,  tSlope1, tStepMid, tSlope2,   {-0.02,0.02,0},{0,height1,0}}, --ls
    {{0.18,0,0},1,  tSlope1, 1.2, tSlope2,   {-0.02,-0.02,0},{-1,height2,0}}, --rf kick    

    {{0.06,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,height1,0}},
    {{0.0,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }

kick.stepqueue["RightKick0"]=
  {
    {{0.12,0,0},1,  tSlope1, tStepMid, tSlope2,   {-0.02,-0.02,0},{0,height1,0}}, --ls
    {{0.18,0,0},0,  tSlope1, 1.2, tSlope2,   {-0.02,0.02,0},{-1,height2,0}}, --rf kick    

    {{0.06,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, --ls
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }




--Stronger kick
--Works nice! 


local height3 = 0.06 --longkick height
local height3 = 0.03 
local kickfd = 0.12


local kickdur = 2.0
local yShift = 0.0
kick.stepqueue["LeftKick1"]=
  {
    {{kickfd,0,0},1,  0.3,kickdur,0.3,   {0.0,yShift,0},{-2,height3,0}}, --rf kick    
    {{0,0,0,},  2,   0.1, 3, 0.1,     {-0.01,0.0,0},  {0, 0, 0}},                  
    {{kickfd,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{0,walk.stepHeight,0}}, --ls
    {{0.0,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }

kick.stepqueue["RightKick1"]=
  {
    {{kickfd,0,0},0,  0.3,kickdur,0.3,   {0.00,-yShift,0},{-2,height3,0}}, --rf kick    
    {{0,0,0,},  2,   0.1, 3, 0.1,     {-0.01,0.0,0},  {0, 0, 0}},                  
    {{kickfd,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{0,walk.stepHeight,0}}, --ls
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }

-----------------------------------------------------------------------------


--Weaker walkkick
kick.stepqueue["LeftKick2"]=
  {
    {{0.12,0,0},0,  tSlope1, tStepMid, tSlope2,   {-0.02,0.02,0},{0,height1,0}}, --ls
    {{0.18,0,0},1,  tSlope1, 1.2, tSlope2,   {-0.02,-0.02,0},{-1,height2,0}}, --rf kick    

    {{0.06,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,height1,0}},
    {{0.0,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }

kick.stepqueue["RightKick2"]=
  {
    {{0.12,0,0},1,  tSlope1, tStepMid, tSlope2,   {-0.02,-0.02,0},{0,height1,0}}, --ls
    {{0.18,0,0},0,  tSlope1, 1.2, tSlope2,   {-0.02,0.02,0},{-1,height2,0}}, --rf kick    

    {{0.06,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, --ls
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }



kick.stepqueue["null"]=
  {
    {{0.0,0,0},2,  tSlope1, tStepMid, tSlope2,   {0,0,0},{0,0,0}}, 
  }










kick.traj={}
kick.traj.walk = "foot_trajectory_base"
kick.traj.kick = "foot_trajectory_kick"
kick.traj.walkkick = "foot_trajectory_walkkick"
kick.traj.walkkick = "foot_trajectory_walkkick2"









Config.kick = kick

return Config
