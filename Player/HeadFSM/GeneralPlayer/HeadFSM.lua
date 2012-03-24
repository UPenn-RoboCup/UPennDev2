if Config.game.role==4 then
  HeadFSM=require('HeadFSMGoalie');	
elseif Config.fsm.playMode==1 then 
  HeadFSM=require('HeadFSMDemo');
else 
  HeadFSM=require('HeadFSM1');
end
