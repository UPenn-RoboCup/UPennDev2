cwd = os.getenv('PWD')
require('init')
require 'Config'
require 'skeleton'
require 'Boxer'

teamID   = Config.game.teamNumber;
playerID = Config.game.playerID;
nPlayers = Config.game.nPlayers;
nPlayers = 2

print '=====================';
print('Team '..teamID,'Player '..playerID)
print '=====================';

-- Initialize FSMs
skeleton.entry()
for pl=1,nPlayers do
	print('init:',pl)
	Boxer.entry(pl);
end

t0 = unix.time();
t_last_debug = t0;
count = 0;
while true do
  local t_start = unix.time();
  
  -- Updates
	skeleton.update();
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

