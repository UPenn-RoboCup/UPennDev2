cwd = os.getenv('PWD')
require('init')
require 'Config'
require 'skeleton'
require 'Boxer'

teamID   = Config.game.teamNumber;
playerID = Config.game.playerID;
nPlayers = Config.game.nPlayers;
nPlayers = 2

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
for pl=1,nPlayers do
	Boxer.entry(pl);
end

t0 = unix.time();
t_last_debug = t0;
count = 0;
while true do
  local t_start = unix.time();
  
  -- Updates
	if not skeleton.update() then
		break;
	end
	for pl=1,nPlayers do
  	Boxer.update(pl);
	end

  -- Debugging
  if( t_start-t_last_debug>1 ) then
    local fps = count / (t_start-t_last_debug)
		t_last_debug = t_start
    count = 0;
		print('Debugging...')
    print( string.format('%.1f FPS\n',fps) )
  end
  count = count+1;

end

skeleton.exit()
for pl=1,nPlayers do
	Boxer.exit(pl)
end