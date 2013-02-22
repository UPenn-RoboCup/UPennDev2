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



stancetocrawl={
  {--Bipedal stance
	{torsoX,bodyHeight,bodyTilt }, --Torso X,Z and pitch
	qLArm0,
        {-supportX,footY,0,0}, --LFoot XYZ and pitch 
	1, --Keyframe duration
  },
  {
	{torsoXKneel, 0.7,bodyTilt }, --Torso X,Z and pitch
	qLArm0,
        {-supportX,footY,0,0}, --LFoot XYZ and pitch 
	1, --Keyframe duration
  },
  {
	{torsoXKneel, 0.7,bodyTilt }, --Torso X,Z and pitch
	{math.pi/4, 0,0,0,-1.57,0},
        {-supportX,footY,0,0}, --LFoot XYZ and pitch 
	1, --Keyframe duration
  },
  {
	{torsoXKneel, 0.5,bodyTiltKneel }, --Torso X,Z and pitch
	qLArm0Kneel,
        {-0.18,footY,0,0}, --LFoot XYZ and pitch 
	3, --Keyframe duration
  },
  {--Quadruped stance
	{torsoXKneel,bodyHeightKneel,bodyTiltKneel }, --Torso X,Z and pitch
	qLArm0Kneel,
        {legX, legY,0,0}, --LFoot XYZ and pitch 
	1, --Keyframe duration
  },
}


crawltostance={
  {--Quadruped stance
	{torsoXKneel,bodyHeightKneel,bodyTiltKneel }, --Torso X,Z and pitch
	qLArm0Kneel,
        {legX, legY,0,0}, --LFoot XYZ and pitch 
	1, --Keyframe duration
  },
  {
	{torsoXKneel, 0.5,math.pi/4 }, --Torso X,Z and pitch
	{math.pi/4, 0,0,0,-1.57,0},
        {-0.18,footY,0,0}, --LFoot XYZ and pitch 
	1, --Keyframe duration
  },
  {
	{torsoXKneel, 0.7,0 }, --Torso X,Z and pitch
	qLArm0,
        {-supportX,footY,0,0}, --LFoot XYZ and pitch 
	3, --Keyframe duration
  },
  {
	{torsoXKneel, 0.6,0 }, --Torso X,Z and pitch
	qLArm0,
        {-supportX+0.05,footY,0,0}, --LFoot XYZ and pitch 
	2, --Keyframe duration
  },
  {--Bipedal stance
	{torsoX,bodyHeight,bodyTilt }, --Torso X,Z and pitch
	qLArm0,
        {-supportX,footY,0,0}, --LFoot XYZ and pitch 
	1, --Keyframe duration
  },
}





function entry()
  print("Motion SM:".._NAME.." entry");
  walk.stop();
  crawl.stop();
  started=false;
  is_bipedal = mcm.get_walk_bipedal();
  if is_bipedal>0 then
    current_keyframe = stancetocrawl;  
  else
    current_keyframe = crawltostance;  
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

  if t>keyframe_start_time+current_keyframe[keyframe_count][4] then
    if keyframe_count == #current_keyframe then
      if is_bipedal>0 then
        return "crawldone"
      else
        return "stancedone"
      end
    end
    keyframe_start_time = keyframe_start_time+current_keyframe[keyframe_count][4];
    keyframe_count = keyframe_count + 1;
  end

  ph =  (t-keyframe_start_time)/current_keyframe[keyframe_count][4];

  torsoXZP = (1-ph)*vector.new(current_keyframe[keyframe_count-1][1])
	+ ph*vector.new(current_keyframe[keyframe_count][1]);


  qLArm = (1-ph)*vector.new(current_keyframe[keyframe_count-1][2])
	+ph*vector.new(current_keyframe[keyframe_count][2]);


  qRArm = {qLArm[1],-qLArm[2],-qLArm[3],qLArm[4],-qLArm[5],-qLArm[6]};

  Body.set_larm_command(qLArm);
  Body.set_rarm_command(qRArm);


  footXYZP = (1-ph)*vector.new(current_keyframe[keyframe_count-1][3])
	+ph*vector.new(current_keyframe[keyframe_count][3]);

  pTorso = vector.new({	torsoXZP[1],0,torsoXZP[2],0,torsoXZP[3],0});
  pLLeg = vector.new({ footXYZP[1],footXYZP[2],footXYZP[3],
			0,footXYZP[4],0});
  pRLeg = vector.new({ footXYZP[1],-footXYZP[2],footXYZP[3],
			0,footXYZP[4],0});

  q = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, 0);

  Body.set_lleg_command(q);

end

function exit()

end
