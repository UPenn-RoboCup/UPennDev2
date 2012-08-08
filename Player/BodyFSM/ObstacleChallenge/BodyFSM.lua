if Config.fsm.avoidance_mode == 0 then 
  print("=====Walk Through Body FSM Loaded====")
  BodyFSM = require('BodyFSMGoal');
elseif Config.fsm.avoidance_mode == 1 then
  print("======Dribble Body FSM Loaded========")
  BodyFSM = require('BodyFSMDribble');
elseif Config.fsm.avoidance_mode == 2 then
  print("========Potential Field==============")
  BodyFSM = require('BodyFSMPField');
end
