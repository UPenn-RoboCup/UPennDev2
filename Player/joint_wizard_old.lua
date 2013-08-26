------------------------------
-- Generic DCM for Robotis NX/MX servos
-- Based on Steve's DCm
-- Generalized for multi-channel use 
------------------------------

dofile'../include.lua'
Config = require('ConfigPenn')

local Body = require(Config.Body);
local libDynamixel=require'libDynamixel';
local jcm=require'jcm'  -- SHM of the joint positions/commands
local unix=require'unix' -- Utilities

-- Dynamixel Settings
local Dynamixel = nil

--------------------------------------------------------
dcm_type = 1; --1 for arms / 2 for legs /3 for spine
--dcm_type = 2; --1 for arms / 2 for legs /3 for spine
--dcm_type = 3; --1 for arms / 2 for legs /3 for spine
--------------------------------------------------------

--[[
local fps_desired = 50;
local read_enable = 1;
--local read_enable = 0;
--]]

local fps_desired = Config.dcm.targetFPS;
local read_enable = Config.dcm.read_enable;

-----------------------------------------------
-- Chanel specific information
-----------------------------------------------

dcm_def={
  -- Channel 1 for arms
  {
		dev_name = '/dev/ttyUSB0', 
		dcm_name = 'Arm DCM',

--[[		
		nx_ids = {1,3,5,7,9,11,    --Left arm
  		        2,4,6,8,10,12      },
--		mx_ids = {14,16,18, -- left hand 1st, 2nd finger
--		          13,15,17}, --right hand 1st,2nd finger
		mx_ids = {14, 18, -- left hand 1st, 2nd finger
		          13,15,17}, --right hand 1st,2nd finger
    nx_ids_to_read={1,3,5,7,9,11,
                    2,4,6,8,10,12},                    
    nx_ids_to_ignore={},        
    mx_ids_to_read={},
    mx_ids_to_ignore={14,16,18,13,15,17},
    	    --]]


--Channel 1 for right arm

		nx_ids = {
  		        2,4,6,8,10,12      },
		mx_ids = { -- left hand 
		          13,15,17}, --right hand 1st,2nd finger
    nx_ids_to_read={
                    2,4,6,8,10,12},                    
    nx_ids_to_ignore={},        
    mx_ids_to_read={},
    mx_ids_to_ignore={13,15,17},




    rx_ids = {},
    rx_ids_to_read={},
    rx_ids_to_ignore={},
    NX_RADIAN_TO_REG = 	       --Radian to step for NX servos
    	{80000,80000,80000,80000,50000,50000,
	    80000,80000,80000,80000,50000,50000,},
	    

  },
  
  -- Channel 2 for left arm
  {
    dev_name = '/dev/ttyUSB1', 
    dcm_name = 'Leg DCM',
    
 		nx_ids = {1,3,5,7,9,11,
  		                       },
		mx_ids = {14,18, -- left hand 
		                 }, --right hand 1st,2nd finger
    nx_ids_to_read={
                    1,3,5,7,9,11,},                    
    nx_ids_to_ignore={},        
    mx_ids_to_read={},
    mx_ids_to_ignore={14,16,18},

    rx_ids = {},
    rx_ids_to_read={},
    rx_ids_to_ignore={},
    NX_RADIAN_TO_REG = 	       --Radian to step for NX servos
    	{80000,80000,80000,80000,50000,50000,
	    80000,80000,80000,80000,50000,50000,},
	
  },
  -- Channel 3 for neck/waist/LIDAR / Wheels
  {
    dev_name = '/dev/ttyUSB2', 
    dcm_name = 'Spine DCM',
    nx_ids = {25,26,		--Waist 
	      27,28,		--Neck
	      20, --Left wheel
	      19, --Right wheel
		},  	
    mx_ids = {36}, 	       --List of MX servos
    rx_ids = {}, 	       --List of MX servos
    NX_RADIAN_TO_REG = 	       --Radian to step for NX servos
	{80000,80000,
	 50000,50000,
	 80000,80000},
    nx_ids_to_read = {25,26,27,28,20,19},
    nx_ids_to_ignore = {},
    mx_ids_to_read = {36},
    mx_ids_to_ignore = {},
    rx_ids_to_read = {},
    rx_ids_to_ignore = {},
  },
}


--Read dcm definition
local dev_name = dcm_def[dcm_type].dev_name;
local dcm_name = dcm_def[dcm_type].dcm_name;
local nx_ids = dcm_def[dcm_type].nx_ids;
local mx_ids = dcm_def[dcm_type].mx_ids;
local rx_ids = dcm_def[dcm_type].rx_ids;
local nx_ids_to_read = dcm_def[dcm_type].nx_ids_to_read;
local mx_ids_to_read = dcm_def[dcm_type].mx_ids_to_read;
local rx_ids_to_read = dcm_def[dcm_type].rx_ids_to_read;
local nx_ids_to_ignore = dcm_def[dcm_type].nx_ids_to_ignore;
local mx_ids_to_ignore = dcm_def[dcm_type].mx_ids_to_ignore;
local rx_ids_to_ignore = dcm_def[dcm_type].rx_ids_to_ignore;

local NX_RADIAN_TO_REG = dcm_def[dcm_type].NX_RADIAN_TO_REG;
local MX_RADIAN_TO_REG = 2047 / math.pi;
local RX_RADIAN_TO_REG = 1024 / math.pi;
local RAD = math.pi/180;
local nJointNX = #nx_ids;
local nJointMX = #mx_ids;
local nJointRX = #rx_ids;


--Setter/getter function here
--Any better location for these?
function set_position_id(ids,val)
  local pos = jcm:get_position();
  for j,v in ipairs(ids) do pos[v] = val[j]; end
  jcm:set_position(pos);
end
function set_command_position_id(ids,val)
  local pos = jcm:get_command_position();
  for j,v in ipairs(ids) do pos[v] = val[j];  end
  jcm:set_command_position(pos);
end
function set_command_velocity_id(ids,val)
  local pos = jcm:get_command_velocity();
  for j,v in ipairs(ids) do pos[v] = val[j];  end
  jcm:set_command_velocity(pos);
end


function get_position_id(ids)
  local pos = jcm:get_position();
  ret={};
  for j,val in ipairs(ids) do ret[j] = pos[val]; end
  return ret;
end
function get_command_position_id(ids)
  local pos = jcm:get_command_position();
  ret={};
  for j,val in ipairs(ids) do  ret[j] =pos[val]; end
  return ret;
end
function get_command_velocity_id(ids)
  local vel = jcm:get_command_velocity();
  ret={};
  for j,val in ipairs(ids) do  ret[j] =vel[val]; end
  return ret;
end




local function entry()
  ---------- Connect to Dynamixel Bus ----------
  io.write(dcm_name,' | Opening Dynamixel bus... ')
  Dynamixel = libDynamixel.new_bus( dev_name )
  io.write('Done! ', Dynamixel.ttyname, '\n')

--  test, err = Dynamixel:get_nx_position( nx_ids[i] );

  ---------- Torque Off Motors ----------
  io.write(dcm_name,' | Torquing OFF the motors... ')
  
  --Dynamixel:ping_probe();

--  Dynamixel:set_mx_status_return_level(mx_ids,0)

  
  Dynamixel:set_mx_led( mx_ids, 0 )
  Dynamixel:set_nx_led_red( nx_ids, 0 )
  Dynamixel:set_rx_led( rx_ids, 0 )
	
  Dynamixel:set_mx_torque_enable( mx_ids, 0 )
  Dynamixel:set_nx_torque_enable( nx_ids, 0 )
	-- TODO: I think this is not possible!
  Dynamixel:set_rx_torque_enable( rx_ids, 0 )
  io.write('Done!\n')
end


local function init_sync()

  ---------- Synchronize MX Joint Positions ----------
  -- Uses single reads, not sync read
  io.write(dcm_name,' | Synchronizing initial MX joint positions...\n')
  local mx_position_reg = vector.zeros(nJointMX)
  local mx_position_deg = vector.zeros(nJointMX)
  
  
  for i=1,nJointMX do
    mx_position_reg[i] = Dynamixel:get_mx_position( mx_ids[i] )
    if not mx_position_reg[i] then
      print('MX #',i,'(id ',mx_ids[i],' Read error',err);
      print("Error in MX initialization, disabling.")      
      nJointMX = 0;
      mx_ids={};
      break;
    else
      print('MX #',i,'(id ',mx_ids[i],') Read OK');
      Dynamixel:set_mx_status_return_level(mx_ids[i],0)
    end
    mx_position_deg[i] = mx_position_reg[i] / MX_RADIAN_TO_REG / RAD;
  end
  set_position_id(mx_ids,mx_position_deg)
  set_command_position_id(mx_ids,mx_position_deg)
  io.write('Done!\n')



  ---------- Synchronize NX Joint Positions ----------
  -- Uses single reads, not sync read





  io.write(dcm_name,' | Synchronizing initial NX joint positions...')
  
  
  
  
  
  
  local nx_position_reg = vector.zeros(nJointNX)
  local nx_position_deg = vector.zeros(nJointNX)

  for i=1,nJointNX do
    nx_position_reg[i], err = Dynamixel:get_nx_position( nx_ids[i] );
    if not nx_position_reg[i] then
      print('\nNX #',i,'(id ',nx_ids[i],' Read error',err);
      return false;
    end
    nx_position_deg[i] = nx_position_reg[i] / NX_RADIAN_TO_REG[i] / RAD;
  end
  -- Initial position and commanded position are the same
  -- SJ: now the function is parameterized with actual servo IDs

  set_position_id(nx_ids,nx_position_deg)
  set_command_position_id(nx_ids,nx_position_deg)
  
  --Reset velocity to zero
  set_command_velocity_id(nx_ids,vector.zeros(nJointNX))
  io.write('Done!\n')

  ---------- Synchronize RX Joint Positions ----------
  -- Uses single reads, not sync read
  io.write(dcm_name,' | Synchronizing initial RX joint positions...\n')
  local rx_position_reg = vector.zeros(nJointRX)
  local rx_position_deg = vector.zeros(nJointRX)
  for i=1,nJointRX do
    rx_position_reg[i] = Dynamixel:get_rx_position( rx_ids[i] )
    if not rx_position_reg[i] then
      print('RX #',i,'(id ',rx_ids[i],' Read error',err);
      print("Error in MX initialization, disabling.")      
      nJointRX = 0;
      rx_ids={};
      break;
    else
      print('RX #',i,'(id ',rx_ids[i],') Read OK');
    end
    rx_position_deg[i] = rx_position_reg[i] / RX_RADIAN_TO_REG / RAD;
  end

  set_position_id(rx_ids,rx_position_deg)
  set_command_position_id(rx_ids,rx_position_deg)
  io.write('Done!\n')
	
  ---------- Torque On Motors ----------
  io.write(dcm_name,' | Torquing ON the motors... ')
  Dynamixel:set_nx_led_green( nx_ids, 255 )
  Dynamixel:set_nx_torque_enable( nx_ids, 1 )

  Dynamixel:set_mx_led( mx_ids, 1 )
  Dynamixel:set_mx_torque_enable( mx_ids, 1 )

  Dynamixel:set_rx_led( rx_ids, 1 )
	-- TODO: DONT THINK THEY HAVE THIS...
  Dynamixel:set_rx_torque_enable( rx_ids, 1 )

  io.write('Done!\n')





  return true
end


-------------------------------------------------------
-- JCM Update
-- Synchronize SHM table with Motor ram table
-------------------------------------------------------
local function update_write()
  ---------- Send MX Motor Commands ----------

  if nJointMX> 0 then
    local mx_cmd_pos = get_command_position_id(mx_ids)
    for j,val in ipairs(mx_cmd_pos) do
      mx_cmd_pos[j] = val * MX_RADIAN_TO_REG*RAD;
    end
    Dynamixel:set_mx_command(mx_ids,mx_cmd_pos)
  end
  if nJointRX> 0 then
    local rx_cmd_pos = get_command_position_id(rx_ids)
    for j,val in ipairs(rx_cmd_pos) do
      rx_cmd_pos[j] = val * RX_RADIAN_TO_REG*RAD;
    end
    --Dynamixel:set_rx_command(rx_ids,rx_cmd_pos)
  end
	

  ---------Sync NX velocity (for wheel control)--------
  -- TODO: what's the unit for velocity?
  if dcm_type==3 then
    local nx_cmd_vel = get_command_velocity_id(nx_ids)
    Dynamixel:set_nx_command_velocity(nx_ids,nx_cmd_vel)
  end

  ---------- Send NX Motor Commands ----------
  local nx_cmd_pos = get_command_position_id(nx_ids)
  for j,val in ipairs(nx_cmd_pos) do
    nx_cmd_pos[j] = val * NX_RADIAN_TO_REG[j]*RAD;
  end
  Dynamixel:set_nx_command_position(nx_ids,nx_cmd_pos)

end

local function update_read_mx(ids)
  local ret
  ---------- Synchronize MX Joint Positions ----------
  -- This uses BULK_READ
  if ids and #ids>0 and nJointMX>0 then
    mx_pos_now = Dynamixel:get_mx_position( ids )
    if mx_pos_now and #mx_pos_now==#ids then
      for j,val in ipairs(mx_pos_now) do
        mx_pos_now[j] = val / MX_RADIAN_TO_REG / RAD;
      end
      set_position_id(ids, mx_pos_now)
    else
      print('CM | Bad MX SYNC_READ!')
    end
  end
end

local function update_read_rx(ids)
  local ret
  ---------- Synchronize MX Joint Positions ----------
  -- This does NOT use BULK_READ
  if ids and #ids>0 then
		for ii=1,#ids do
			local rx_pos_now = Dynamixel:get_rx_position( ids[ii] )
      set_position_id( {ids[ii]}, {rx_pos_now / RX_RADIAN_TO_REG / RAD} )
		end
  end
end

local function update_read_nx(ids)
   ---------- Synchronize NX Joint Positions ----------
   -- This uses BULK_READ
   nx_pos_now = Dynamixel:get_nx_position(ids )
   if nx_pos_now and #nx_pos_now==#ids then
     for j,val in ipairs(nx_pos_now) do
       nx_pos_now[j] = val / NX_RADIAN_TO_REG[j] / RAD;
     end
     set_position_id(ids,nx_pos_now)
   else
     print('CM | Left Arm Bad NX SYNC_READ!')
   end
end

local function update_battery()
  nx_vol_now = Dynamixel:get_nx_voltage( nx_ids )
  --Average voltage reading from servos
  sum = 0;
  if nx_vol_now then
    for i=1,#nx_vol_now do sum=sum+nx_vol_now[i]; end
    return sum / #nx_vol_now / 10;
  end
  return 0;
end


--Just copy commanded angle 
local function update_read_disabled_nx(ids)
  if ids then
    com_pos_nx = get_command_position_id(ids)
    set_position_id(ids, com_pos_nx);  
  end
end
local function update_read_disabled_mx(ids)
  if ids then
    com_pos_mx = get_command_position_id(ids)
    set_position_id(ids, com_pos_mx);  
  end
end
local function update_read_disabled_rx(ids)
  if ids then
    com_pos_rx = get_command_position_id(ids)
    set_position_id(ids, com_pos_rx);  
  end
end


local function exit()
  print('CM | Exit')
  Dynamixel:close()
end

-- Just update the right arm
local cnt = 0;
local t_start = unix.time();
local t_last = t_start;
local t_debug = t_start;
local t = t_last;

entry()
local init = init_sync()
if not init then
  print('CM | Bad Initialization.  Exiting now.')
  return
end





local t = t_last
read_time_mx = 0
read_time_nx = 0
read_time_rx = 0
write_time = 0
awake_ratio = 0


while true do
  -- Timing
  t_last = t
  t = unix.time()
  local t_diff = t - t_last
  if t_diff < 1/fps_desired then
    sleeptime = (1/fps_desired-t_diff);
    unix.usleep(1E6*sleeptime);
    t=unix.time();
    t_diff = t-t_last;
  else
    sleeptime = 0;
  end
  awake_ratio = awake_ratio + (1-sleeptime/t_diff);


  -- Perform the update

  tcheck0=unix.time();
  update_write();
  tcheck1=unix.time();
  write_time = write_time + (tcheck1-tcheck0);

  if read_enable>0 then
    update_read_nx(nx_ids);
    tcheck2=unix.time();
    read_time_nx = read_time_nx + (tcheck2-tcheck1);
    update_read_mx(mx_ids);
    tcheck3=unix.time();
    read_time_mx = read_time_mx + (tcheck3-tcheck2);
    update_read_rx(rx_ids);
    tcheck4=unix.time();
    read_time_rx = read_time_rx + (tcheck4-tcheck3);
  else
    update_read_nx(nx_ids_to_read);
    tcheck2=unix.time();
    read_time_nx = read_time_nx + (tcheck2-tcheck1);
    update_read_mx(mx_ids_to_read);
    tcheck3=unix.time();
    read_time_mx = read_time_mx + (tcheck3-tcheck2);
    update_read_rx(rx_ids_to_read);
    tcheck4=unix.time();
    read_time_rx = read_time_rx + (tcheck4-tcheck3);

    update_read_disabled_nx(nx_ids_to_ignore);
    update_read_disabled_mx(mx_ids_to_ignore);
    update_read_disabled_rx(rx_ids_to_ignore);
  end

  cnt = cnt + 1
  -- Debugging: Once 2 seconds
  local t_diff_debug = t - t_debug

  if t_diff_debug > 1 then
    local pos = jcm:get_position();
--    battery = update_battery();
--    jcm:set_battery(battery);
    battery = 0;  
    os.execute('clear')
    cur_pos_nx = get_position_id(nx_ids)
    cur_pos_mx = get_position_id(mx_ids)
		cur_pos_rx = get_position_id(rx_ids)
		
    com_pos_nx = get_command_position_id(nx_ids)
    com_pos_mx = get_command_position_id(mx_ids)
    com_pos_rx = get_command_position_id(rx_ids)

    com_vel_nx = get_command_velocity_id(nx_ids)

    io.write( string.format(
	'\n%s | Updating at %.2f FPS  Read: mx %.1f ms rx %.1fms nx %.1f ms Write: %.1f ms awake: %d %% \n',
        dcm_name,cnt/t_diff_debug, read_time_mx/cnt*1000, read_time_rx/cnt*1000, 
	read_time_nx/cnt*1000, write_time/cnt*1000,
	awake_ratio/cnt*100		
        ) )

    io.write( string.format("NX Pos :"));
    for i=1,nJointNX do io.write(string.format(" %d", cur_pos_nx[i]));end
    io.write( string.format("\nCommand:"));
    for i=1,nJointNX do io.write(string.format(" %d", com_pos_nx[i]));end
    io.write( string.format("\nC. Velocity:"));
    for i=1,nJointNX do io.write(string.format(" %d", com_vel_nx[i]));end
   
    if nJointMX>0 then
      io.write( string.format("\nMX Pos: "));
      for i=1,nJointMX do io.write(string.format(" %d",cur_pos_mx[i]));end

      io.write( string.format("\nMX Command: "));
      for i=1,nJointMX do io.write(string.format(" %d",com_pos_mx[i]));end
    end
    if nJointRX>0 then
      io.write( string.format("\nRX Pos: "));
      for i=1,nJointRX do io.write(string.format(" %d",cur_pos_rx[i]));end

      io.write( string.format("\nRX Command: "));
      for i=1,nJointRX do io.write(string.format(" %d",com_pos_rx[i]));end
    end
    io.write( string.format("\nBattery level: %.2f", battery ));

    io.flush();
    t_debug = t
    cnt = 0;
    read_time_mx = 0;
    read_time_nx = 0;
    read_time_rx = 0;
    write_time = 0;
    awake_ratio = 0;

  end
end

-- Exit
exit()
