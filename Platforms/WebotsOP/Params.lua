Params = {}
Params.platformName = 'WebotsOP'

function Params.get_field(field)
  local t = Params
  for k in string.gmatch(field, '(%a[%a_%d]*)') do
    if (type(t) ~= 'table') then
      return nil
    end
    t = t[k]
  end
  return t
end


Params.jointNames = {
  "Neck", "Head",
  "ShoulderL", "ArmUpperL", "ArmLowerL",
  "PelvYL", "PelvL", "LegUpperL", "LegLowerL", "AnkleL", "FootL", 
  "PelvYR", "PelvR", "LegUpperR", "LegLowerR", "AnkleR", "FootR",
  "ShoulderR", "ArmUpperR", "ArmLowerR",
};

Params.jointBias={
  0,0,
  -math.pi/2,0,math.pi/2,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  -math.pi/2,0,math.pi/2,
}

Params.vision = {
  update_interval = 0.04 
}

Params.jointReverse={
  1,--Head: 1,2
  --LArm: 3,4,5
  7,8,9,--LLeg: 6,7,8,9,10,11,
  16,--RLeg: 12,13,14,15,16,17
  18,20--RArm: 18,19,20
}

Params.moveDir={};
for i=1,#Params.jointNames do Params.moveDir[i]=1; end
for i=1,#Params.jointReverse do Params.moveDir[Params.jointReverse[i]]=-1; end
Params.vision = { 
  camera_interval = 1/30; -- 30Hz
  -- Set lidar_interval to 0 to disable its use
  --lidar_interval  = 1/40; -- 40Hz
  lidar_interval  = 0; -- Disable
}
