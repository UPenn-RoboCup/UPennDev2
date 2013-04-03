module(..., package.seeall);
name = ...;

require('Config')
require('vector')
require('Kinematics')
require('Body')
require('walk')
require('mcm')

active = true;
t0 = 0;

--Plays back IK-based symmetric keyframe

torsoX = Config.walk.torsoX;
footY = Config.walk.footY;
supportX = Config.walk.supportX;
bodyHeight = Config.walk.bodyHeight;
bodyTilt = Config.walk.bodyTilt;
qLArm0 = Config.walk.qLArm;


torsoXKneel = Config.kneel.torsoX;
bodyHeightKneel = Config.kneel.bodyHeight;
bodyTiltKneel = Config.kneel.bodyTilt;
legX = Config.kneel.legX;
legY = Config.kneel.legY;
armX = Config.kneel.armX;
armY = Config.kneel.armY;
armZ = Config.kneel.armZ;

qLArm0Kneel = Config.kneel.qLArm0;


print("qlarm:",unpack(qLArm0))



keyframe_stance = {
  {torsoX,bodyHeight,bodyTilt }, --Torso X,Z and pitch
  qLArm0,
  {-supportX,footY,0,0}, --LFoot XYZ and pitch 
}

keyframe_sit= {
  {torsoX, 0.65,-3*math.pi/180 }, --Torso X,Z and pitch
  qLArm0,
  {-supportX,footY,0,0}, --LFoot XYZ and pitch 
}

keyframe_crawl = {
  {torsoXKneel,bodyHeightKneel,bodyTiltKneel }, --Torso X,Z and pitch
  qLArm0Kneel,
  {legX, legY,0,0}, --LFoot XYZ and pitch 
}

keyframe_kneel1 = {
  {torsoXKneel, 0.5,math.pi/4 }, --Torso X,Z and pitch
--  {math.pi/4, 0,0,0,-1.57,0},
  {math.pi/4, 0,0,0,-1.57,-1.57},
  {-0.18,footY,0,0}, --LFoot XYZ and pitch 
}

keyframe_kneel2 = {
  {torsoXKneel-0.05, 0.65,-3*math.pi/180 }, --Torso X,Z and pitch
  {math.pi/4, 0,0,0,-1.57,0},
  {-supportX,footY,0,0}, --LFoot XYZ and pitch 
}

motion_stancetocrawl={
  {keyframe_stance,1},
  {keyframe_sit,1},
  {keyframe_kneel2,1},
  {keyframe_kneel1,2},
  {keyframe_crawl,1},
}

motion_crawltostance={
  {keyframe_crawl,1},
  {keyframe_kneel1,1},
  {keyframe_kneel2,2},
  {keyframe_sit,1},
  {keyframe_stance,1},
}

function entry()
  print("Motion SM:".._NAME.." entry");
  walk.stop();
  crawl.stop();
  started=false;
  is_bipedal = mcm.get_walk_bipedal();
  if is_bipedal>0 then
    current_motion = motion_stancetocrawl;  
  else
    current_motion = motion_crawltostance;  
  end
end


function switch_init()
  --This makes the robot look up and see goalposts while sitting down
  Body.set_head_command({0,-20*math.pi/180});
  Body.set_head_hardness(.5);
  Body.set_larm_hardness(.1);
  Body.set_rarm_hardness(.1);
  t0=Body.get_time();
  Body.set_syncread_enable(1); 

  keyframe_count = 2;
  keyframe_start_time = Body.get_time();

end

function calculate_current_transfom()
end


function update()
  local t = Body.get_time();
  if walk.active then 
     walk.stop();
     walk.update();
     t0=Body.get_time();
     return;
  end
  if crawl.active then 
     crawl.stop();
     crawl.update();
     t0=Body.get_time();
     return;
  end
  if not started then
    started = true;
    switch_init();
    return;
  end

  local current_keyframe = current_motion[keyframe_count][1];
  local current_duration = current_motion[keyframe_count][2];

  if t>keyframe_start_time+current_duration then
    if keyframe_count == #current_motion then
      if is_bipedal>0 then
        return "crawldone"
      else
        return "stancedone"
      end
    end
    keyframe_start_time = keyframe_start_time+current_duration;
    keyframe_count = keyframe_count + 1;
  end

  local ph =  (t-keyframe_start_time)/current_duration;
  local previous_keyframe = current_motion[keyframe_count-1][1];


  local torsoXZP = (1-ph)*vector.new(previous_keyframe[1])
	+ ph*vector.new(current_keyframe[1]);

  local qLArm = (1-ph)*vector.new(previous_keyframe[2])
	+ph*vector.new(current_keyframe[2]);

  local qRArm = {qLArm[1],-qLArm[2],-qLArm[3],qLArm[4],-qLArm[5],-qLArm[6]};


  local footXYZP = (1-ph)*vector.new(previous_keyframe[3])
	+ph*vector.new(current_keyframe[3]);

  local pTorso = vector.new({	torsoXZP[1],0,torsoXZP[2],0,torsoXZP[3],0});
  local pLLeg = vector.new({ footXYZP[1],footXYZP[2],footXYZP[3],
			0,footXYZP[4],0});
  local pRLeg = vector.new({ footXYZP[1],-footXYZP[2],footXYZP[3],
			0,footXYZP[4],0});

  local q = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, 0);


  Body.set_larm_command(qLArm);
  Body.set_rarm_command(qRArm);

  Body.set_lleg_command(q);

end

function exit()

end
