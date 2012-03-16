ps = require 'primesense'
require 'unix';

-- Wait at least 1 seconds
unix.sleep(10);
while(true) do
  print( "Torso: ",ps.get_torso() );
  --Puase .1 seconds
  unix.usleep(1E5);
end
