cwd = os.getenv('PWD')
require('init')
require 'gcm'

teamID   = gcm.get_team_number();
playerID = gcm.get_team_player_id();
nPlayers = gcm.get_game_nplayers();
if( arg[1] and tonumber(arg[1])<=nPlayers and tonumber(arg[1])>0 ) then
  forPlayer = tonumber(arg[1]);
else
  forPlayer = playerID;
end

-- Issue debug line telling which mode we are in
desired_fps = 60;
twait = 1/desired_fps;
print '=====================';
print('Desired FPS: ',desired_fps);
print '=====================';

-- Set up the Boxing FSM
require 'Boxer'
Boxer.init(forPlayer)
Boxer.entry();

count = 0;
t0 = unix.time();
while true do
  local t_start = unix.time();
  
  -- Updates
  Boxer.update();

  -- Timing
  if( count % desired_fps==0 ) then
    local fps = desired_fps / (unix.time()-(t_count or 0))
    t_count = unix.time();
    print('FPS: ',fps)
    count = 0;
  end
  count = count+1;

  local t_loop = unix.time() - t_start;
  unix.usleep( 1e6*math.max(twait-t_loop,0) );

end

