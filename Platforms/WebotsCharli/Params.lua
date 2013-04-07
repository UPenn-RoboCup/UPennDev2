Params = {}
Params.platformName = 'WebotsCharli'

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
  "Head","Neck",
  "L_Shoulder_Pitch", "L_Shoulder_Roll", "L_Shoulder_Yaw","L_Elbow",
  "L_Hip_Yaw", "L_Hip_Roll", "L_Hip_Pitch", "L_Knee_Pitch", "L_Ankle_Pitch", "L_Ankle_Roll",
  "R_Hip_Yaw", "R_Hip_Roll", "R_Hip_Pitch", "R_Knee_Pitch", "R_Ankle_Pitch", "R_Ankle_Roll",
  "R_Shoulder_Pitch", "R_Shoulder_Roll", "R_Shoulder_Yaw","R_Elbow",
  "Waist_Roll",
};

Params.jointReverse={
  3,4,5,6,--LArm:  3 4 5 6
  --LLeg: 7 8 9 10 11 12
  --RLeg: 13 14 15 16 17 18
  22,--RArm: 19 20 21 22
  --Waist: 23
}

Params.jointBias={
  0,0,
  -90*math.pi/180,0,0,0,
  0,0,0,0,0,0,
  0,0,0,0,0,0,
  -90*math.pi/180,0,0,0,
  0,
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

