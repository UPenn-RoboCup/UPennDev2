cwd = os.getenv('PWD')
require('init')
require 'Config'
require 'xbox'
require 'pickercm'
require('getch')
getch.enableblock(1);

teamID   = Config.game.teamNumber;
playerID = Config.game.playerID;

print '=====================';
print('Team '..teamID,'Player '..playerID)
print '=====================';

net = true

function entry()
  -- Initialize FSMs
  xbox.entry()
  pickercm.set_device_kind(2) -- XBOX360
  pickercm.set_desired_dpLArm( {0,0,0} )
  pickercm.set_desired_dpRArm( {0,0,0} )

  if( net ) then
    require 'Team'
    Team.entry(true) -- true means we have the primesense
  end

  -- Start variables
  t0 = unix.time();
  t_last_debug = t0;
  count = 0;

end

function update()
  local t_start = unix.time();

  -- Updates
  if not xbox.update() then
    return false
  end
  pickercm.set_device_kind(2)
  unix.usleep(1e4); -- Add 10ms delay to simulate the Team sending

  -- Update the SHM blocks
  pickercm.set_desired_velocity( xbox.get_velocity() );
  pickercm.set_desired_dpLArm( xbox.get_dpLArm() )
  pickercm.set_desired_dpRArm( xbox.get_dpRArm() )
  pickercm.set_desired_rpy( xbox.get_rpy() );
  pickercm.set_desired_dbodyHeight( xbox.get_dbodyHeight() )

  -- User Key input
  local keycode = process_keyinput();
  if keycode==string.byte("q") then
    return false
  end

  -- Toggle Logging
  if keycode==string.byte("l") then
    skeleton.toggle_logging()
  end

  -- Send out information
  if net then
    Team.update()
  end

  -- Debugging
  if( t_start-t_last_debug>1 ) then
    local fps = count / (t_start-t_last_debug)
    t_last_debug = t_start
    count = 0;
    --[[
    print( "Detected XBOX360 Controller?:",mcm.get_walk_override()[1])
    print( "Override Mode:",mcm.get_walk_override()[2])
    print( 'Velocity', mcm.get_walk_velocity() )
    print( 'Height', mcm.get_walk_bodyHeight() )
    print( 'Arms', mcm.get_walk_qLArm(),mcm.get_walk_qRArm() )
    --]]
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

