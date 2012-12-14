cwd = os.getenv('PWD')
require('init')
require 'Config'
require 'skeleton'
require 'primecm'
require 'libpicker'
require 'pickercm'
require('getch')
getch.enableblock(1);

teamID   = Config.game.teamNumber;
playerID = Config.game.playerID;
nPlayers = Config.game.nPlayers;
nPlayers = 1
net = true

function entry()
  -- Check inputs
  print("Num args: ",#arg)
  inp_logs = {};
  if( arg[1] ) then
    nPlayers = #arg
    for i=1,nPlayers do
      dofile( arg[i] )
      inp_logs[i] = log;
    end
  end

  print '=====================';
  print('Team '..teamID,'Player '..playerID)
  print '=====================';

  -- Initialize FSMs
  skeleton.entry(inp_logs)

  if( net ) then
		require 'Comm'
		-- Initialization
    wired = true
		if( wired ) then
		  print("My address:",Config.dev.ip_wired)
		  Comm.init(Config.dev.ip_wired,Config.dev.ip_wired_port)
		else
		  print("My address:",Config.dev.ip_wireless)
		  Comm.init(Config.dev.ip_wireless,Config.dev.ip_wireless_port);
		end
    --require 'Team'
    --Team.entry(true) -- true means we have the primesense
  end


  t0 = unix.time();
  t_last_debug = t0;
  count = 0;

  pickercm.set_device_kind(1)
  pickercm.set_desired_pLArm( Kinematics.forward_larm({0,0,0,0}) )
  pickercm.set_desired_pRArm( Kinematics.forward_rarm({0,0,0,0}) )
	pickercm.set_desired_qRArm( vector.new({0,0,0,0}) )
	pickercm.set_desired_qLArm( vector.new({0,0,0,0}) )

  -- Calibration warning
  print('Please get in position...')

end

function update()
  local t_start = unix.time();

  -- Updates
  if not skeleton.update() then
    return false
  end

  -- Update arm coordinates
  skeleton.update_arms()
  skeleton.update_torso()
  skeleton.update_height()
  skeleton.update_velocity()

  -- Calibration
  tNow = unix.time()
  -- Wait to get in position
  if #inp_logs==0 and tNow-t0<2 then
    return true;
  end
  -- Start it
  if not calibrated and primecm.get_skeleton_found()==1 then -- 2 seconds to start
    if not tCalibrate then
      print('Calibrating!')
      tCalibrate = tNow;
      ncalibration = 0;
      torso_point = vector.zeros(3);
    end
    if tNow-tCalibrate>1 then -- 1 sec calibration
      print('DONE Calibrating!',ncalibration.." samples taken")
      torso_point = torso_point / ncalibration;
      print('Torso point:',torso_point )
      primecm.set_skeleton_torsocenter(torso_point)
      calibrated = true;
    end
    torso_point = torso_point + primecm.get_position_Torso();
    --		torso_point = torso_point + primecm.get_position_Waist();
    ncalibration = ncalibration + 1;
  end
  if not calibrated then
    return true;
  end


  if( net ) then
		local state = {};
		state.qL = skeleton.qLArm;
		state.qR = skeleton.qRArm;
		local ser = serialization.serialize(state)
		local ret = Comm.send( ser, #ser );
		print('Sent: '..ret..' bytes',ser)
    --Team.update()
  end
	
	
  -- Update the SHM blocks
  if primecm.get_skeleton_found()>0  then
    pickercm.set_desired_velocity( skeleton.velocity );
		pickercm.set_desired_qLArm(skeleton.qLArm);
		pickercm.set_desired_qRArm(skeleton.qRArm);
    local tLArm = Kinematics.forward_larm( skeleton.qLArm )
    local pLArm = vector.new({tLArm[1][4],tLArm[2][4],tLArm[3][4]})
    pickercm.set_desired_pLArm( pLArm )
    local tRArm = Kinematics.forward_rarm( skeleton.qRArm )
    local pRArm = vector.new({tRArm[1][4],tRArm[2][4],tRArm[3][4]})
    pickercm.set_desired_pRArm( pRArm )
    pickercm.set_desired_rpy( skeleton.rpy );
    pickercm.set_desired_bodyHeight( skeleton.bodyHeight )
    pickercm.set_device_kind(1);
  else
    pickercm.set_device_kind(0) -- Disabled
  end

  -- User Key input
  local keycode = process_keyinput();
  if keycode==string.byte("q") then
    return false
  end

  -- Toggle Logging
  if keycode==string.byte("l") then
    skeleton.toggle_logging()
  end

  -- Debugging
  if( t_start-t_last_debug>1 ) then
    local fps = count / (t_start-t_last_debug)
    t_last_debug = t_start
    count = 0;
    if primecm.get_skeleton_found()>0 then
      print('Found User...')
    else
      print('No User...')
    end
    print( string.format('%.1f FPS\n',fps) )
  end
  count = count+1;
  return true
end

function exit()
  skeleton.exit()
end

function process_keyinput()
  local str=getch.get();
  if #str>0 then
    local byte=string.byte(str,1);
    return byte
  end
end

entry()
while true do
  if not update() then
    exit()
    break;
  end
end
