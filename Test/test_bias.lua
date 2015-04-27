#!/usr/bin/env luajit
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end
local getch = require'getch'

targetvel={0,0,0}
targetvel_new={0,0,0}
local servo_names={
  "hipyaw",
  "hiproll",
  "hippitch",
  "kneepitch",
  "anklepitch",
  "ankleroll"
}

local selected_servo = 1
local bias_mag = 0.0675*math.pi/180

function process_keyinput()
  local byte=getch.block();
  local bias_edited = false

  if byte then
    local legBias = mcm.get_leg_bias()
  
    -- Walk velocity setting
    if byte==string.byte("i") then      targetvel_new[1]=targetvel[1]+0.02;
    elseif byte==string.byte("j") then  targetvel_new[3]=targetvel[3]+0.1;
    elseif byte==string.byte("k") then  targetvel_new[1],targetvel_new[2],targetvel_new[3]=0,0,0;
    elseif byte==string.byte("l") then  targetvel_new[3]=targetvel[3]-0.1;
    elseif byte==string.byte(",") then  targetvel_new[1]=targetvel[1]-0.02;
    elseif byte==string.byte("h") then  targetvel_new[2]=targetvel[2]+0.02;
    elseif byte==string.byte(";") then  targetvel_new[2]=targetvel[2]-0.02;

    elseif byte==string.byte("1") then      
      body_ch:send'init'

    elseif byte==string.byte("-") then      
      motion_ch:send'bias'
    elseif byte==string.byte("8") then  
      motion_ch:send'stand'
      body_ch:send'stop'
      if mcm.get_walk_ismoving()>0 then 
        print("requesting stop")
        mcm.set_walk_stoprequest(1) 
      end
    elseif byte==string.byte("9") then  
      motion_ch:send'hybridwalk'


    elseif byte==string.byte("2") then
      selected_servo = 1
      bias_edited = true
    elseif byte==string.byte("3") then
      selected_servo = 2
      bias_edited = true      
    elseif byte==string.byte("4") then
      selected_servo = 3        
      bias_edited = true      
    elseif byte==string.byte("5") then
      selected_servo = 4
      bias_edited = true      
    elseif byte==string.byte("6") then
      selected_servo = 5
      bias_edited = true      
    elseif byte==string.byte("7") then
      selected_servo = 6      
      bias_edited = true      

    elseif byte==string.byte("q") then
      legBias[selected_servo]=legBias[selected_servo]-bias_mag
      bias_edited = true        
    elseif byte==string.byte("w") then
      legBias[selected_servo]=legBias[selected_servo]+bias_mag
      bias_edited = true      
    elseif byte==string.byte("[") then    
      legBias[selected_servo+6]=legBias[selected_servo+6]-bias_mag
      bias_edited = true      
    elseif byte==string.byte("]") then        
      legBias[selected_servo+6]=legBias[selected_servo+6]+bias_mag
      bias_edited = true      

    elseif byte==string.byte("0") then
      print(string.format("Current bias: \n%.2f %.2f %.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f %.2f %.2f ",
      unpack(vector.new(legBias)*RAD_TO_DEG)))

      --Append at the end of calibration file
      outfile=assert(io.open("../Config/THOROP/calibration.lua","a+"));
      --TODO: which one should we use?
      data=string.format("\n\n-- Updated date: %s\n" , os.date() );
      data=data..string.format("cal[\"%s\"].legBias=vector.new({\n   ",unix.gethostname());
      for i=1,6 do data=data..string.format("%f,",legBias[i]*180/math.pi) end
      data=data..'\n   '
      for i=7,12 do data=data..string.format("%f,",legBias[i]*180/math.pi) end
      data=data..'\n   })*math.pi/180\n';
      outfile:write(data);
      outfile:flush();
      outfile:close();
    end

    if bias_edited then
      mcm.set_leg_bias(legBias)
      print(servo_names[selected_servo]," : ",
        legBias[selected_servo]*RAD_TO_DEG,
        legBias[selected_servo+6]*RAD_TO_DEG
    )
    end

    local vel_diff = (targetvel_new[1]-targetvel[1])^2+(targetvel_new[2]-targetvel[2])^2+(targetvel_new[3]-targetvel[3])^2
    if vel_diff>0 then
      targetvel[1],targetvel[2],targetvel[3] = targetvel_new[1],targetvel_new[2],targetvel_new[3]
      print(string.format("Target velocity: %.3f %.3f %.3f",unpack(targetvel)))
      mcm.set_walk_vel(targetvel)
    end
  end
end

mcm.set_leg_bias(Config.walk.legBias)
local t_last = Body.get_time()
local tDelay = 0.005*1E6

 while true do
  process_keyinput(); --why nonblocking reading does not work?
end
