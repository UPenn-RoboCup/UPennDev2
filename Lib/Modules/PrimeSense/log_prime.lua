require 'primesense'  
require 'vector'
require 'unix'

saveCount = 0;
filename = string.format("/tmp/prime_%03d.raw", saveCount);
f = io.open(filename, "w+");
f:write('log={\n');

count = 0;
init = false;

jointNames = { 
  'Head', 'Neck', 'Torso', 'Waist', -- 1-4
  'CollarL','ShoulderL', 'ElbowL', 'WristL', 'HandL', 'FingerL', --5-10
  'CollarR','ShoulderR', 'ElbowR', 'WristR', 'HandR', 'FingerR', -- 11-16
  'HipL', 'KneeL', 'AnkleL', 'FootL',  -- 17-20
  'HipR', 'KneeR', 'AnkleR', 'FootR' -- 21-24
};

x = vector.zeros(#jointNames);
y = vector.zeros(#jointNames);
z = vector.zeros(#jointNames);
posconf = vector.zeros(#jointNames);
rotconf = vector.zeros(#jointNames);

while( true ) do
  count = count + 1;
  ret = primesense.update_joints();
  timestamp = unix.time();

  if( not init ) then
    init = true;
    timestamp0 = timestamp;
  end

  -- Check each player
  -- Is each player active?
  active = {0,0}; -- init as no...
  for i,v in ipairs(jointNames) do
    pos, rot, confidence, active = primesense.get_jointtables(1,i);
    player = 1;
    if( active==0 ) then
      player = 2;
      pos, rot, confidence, active = primesense.get_jointtables(1,i);
    end
    if( active>0 ) then
      pos = vector.new(pos)/1000;
      x[i] = pos[1];
      y[i] = pos[2];
      z[i] = pos[3];
      posconf[i] = confidence[1];
      rotconf[i] = confidence[2];
    end
  end

  -- Write the log file
  f:write(  string.format("{t=%f,",timestamp-timestamp0)  );
  f:write(  string.format("x={%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f},",unpack(x))  );
  f:write(  string.format("y={%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f},",unpack(y))  );
  f:write(  string.format("z={%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f},",unpack(z))  );
  f:write(  string.format("posconf={%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f},",unpack(posconf))  );
  f:write(  string.format("rotconf={%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f}",unpack(rotconf))  );
  f:write('},\n');

  -- Timing
  if( init ) then
    if( count % 60==0 ) then
      local fps = 60 / (unix.time()-(t_count or 0))
      t_count = unix.time();
      print('FPS: ',fps)
      print('Time',timestamp-timestamp0 );
      if( active) then
        print('User: ', player);
      end
      print();
    end
    if( count % 300==0 ) then
      count = 0;
      print('Saving Log '..saveCount..'...')
      saveCount = saveCount + 1;
      -- Close old file, open new
      f:write('}\n');      
      f:close();
      print('Done!')
      filename = string.format("/tmp/prime_%03d.raw", saveCount);
      f = io.open(filename, "w+");
      f:write('log={\n');
    end
    --local t_loop = unix.time() - t_start;
    --local twait = 1/desired_fps;
    -- unix.usleep( 1e6*math.max(twait-t_loop,0) );
  end

end
-- After done playing, reset the skeleton found variable so no more movement
print('Finished!')

