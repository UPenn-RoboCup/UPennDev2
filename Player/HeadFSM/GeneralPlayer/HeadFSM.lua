if Config.game.role==4 then
  print("Goalie HeadFSM loaded")
  HeadFSM=require('HeadFSMGoalie');	
elseif Config.fsm.playMode==1 then 
  print("Demo HeadFSM loaded")
  HeadFSM=require('HeadFSMDemo');
else 
  print("Player HeadFSM loaded")
  HeadFSM=require('HeadFSM1');
end
