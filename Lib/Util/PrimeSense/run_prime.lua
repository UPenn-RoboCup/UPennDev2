ps = require 'primesense'
require 'unix';

-- Wait at least 1 seconds
--unix.sleep(10);
t0 = unix.time()
while(true) do
  local torso = ps.get_torso()
  if( torso ) then
--    print("Raw torso: ", torso)
    print( "Torso: ", unpack(torso) );
    print();
    -- Update Shm
    
  else
    print("No user detected... Waiting 1 second...")
    unix.sleep(1);
  end
  --Pause .1 seconds
  unix.usleep(1E5);
end
