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


kick.stepqueue={}


--Walkkick #1

kick.stepqueue["LeftKick0"]=
  {
--    {{0.12,0,0},0,  tSlope1, tStepMid, tSlope2,   {-0.02,0.02,0},{0,walk.stepHeight,0}}, --ls
--    {{0.18,0,0},1,  tSlope1, 1.2, tSlope2,   {-0.02,0.04,0},{-1,1.5*walk.stepHeight,0}}, --rf kick    


    {{0.12,0,0},0,  tSlope1, tStepMid, tSlope2,   {-0.02,0.02,0},{0,walk.stepHeight,0}}, --ls
    {{0.16,0,0},1,  tSlope1, 1.2, tSlope2,   {-0.02,-0.02,0},{-1,1.5*walk.stepHeight,0}}, --rf kick    

    {{0.06,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}},
    {{0.0,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }

kick.stepqueue["RightKick0"]=
  {
--    {{0.12,0,0},1,  tSlope1, tStepMid, tSlope2,   {-0.02,-0.02,0},{0,walk.stepHeight,0}}, --ls
--    {{0.18,0,0},0,  tSlope1, 1.2, tSlope2,   {-0.02,-0.04,0},{-1,1.5*walk.stepHeight,0}}, --rf kick    

    {{0.12,0,0},1,  tSlope1, tStepMid, tSlope2,   {-0.02,-0.02,0},{0,walk.stepHeight,0}}, --ls
    {{0.16,0,0},0,  tSlope1, 1.2, tSlope2,   {-0.02,0.02,0},{-1,1.5*walk.stepHeight,0}}, --rf kick    

    {{0.06,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, --ls
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }




--Stronger kick
--Works nice! 

local kickdur = 2.0
local yShift = 0.0

kick.stepqueue["LeftKick1"]=
  {
    {{0.12,0,0},1,  0.3,kickdur,0.3,   {0.0,yShift,0},{-2,walk.stepHeight*1,0}}, --rf kick    
    {{0,0,0,},  2,   0.1, 3, 0.1,     {-0.01,0.0,0},  {0, 0, 0}},                  
    {{0.12,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{0,walk.stepHeight,0}}, --ls
    {{0.0,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }

kick.stepqueue["RightKick1"]=
  {
    {{0.12,0,0},0,  0.3,kickdur,0.3,   {0.00,-yShift,0},{-2,walk.stepHeight*1,0}}, --rf kick    
    {{0,0,0,},  2,   0.1, 3, 0.1,     {-0.01,0.0,0},  {0, 0, 0}},                  
    {{0.12,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{0,walk.stepHeight,0}}, --ls
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }

-----------------------------------------------------------------------------


--Walkkick #2

kick.stepqueue["LeftKick2"]=
  {
    {{0.12,0,0},0,  tSlope1, tStepMid, tSlope2,   {-0.02,0.02,0},{0,walk.stepHeight,0}}, --ls
    {{0.12,0,0},1,  tSlope1, 1.2, tSlope2,   {-0.02,0.04,0},{-1,1.5*walk.stepHeight,0}}, --rf kick    
    {{0.00,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}},
    {{0.0,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }

kick.stepqueue["RightKick2"]=
  {
    {{0.12,0,0},1,  tSlope1, tStepMid, tSlope2,   {-0.02,-0.02,0},{0,walk.stepHeight,0}}, --ls
    {{0.12,0,0},0,  tSlope1, 1.2, tSlope2,   {-0.02,-0.04,0},{-1,1.5*walk.stepHeight,0}}, --rf kick    
    {{0.00,0,0},1,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, --ls
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
    {{0.0,0,0},0,  tSlope1, tStepMid, tSlope2,   {0,0,0},{-9,walk.stepHeight,0}}, 
  }

kick.stepqueue["null"]=
  {
    {{0.0,0,0},2,  tSlope1, tStepMid, tSlope2,   {0,0,0},{0,0,0}}, 
  }




















Config.kick = kick

return Config