dofile('init.lua')
require('SoundFilter');
require('Comm')
Comm.init('192.168.0.255', 54321);
unix.usleep(1000000);

while (1) do
  SoundFilter.update();

  state = {};
  state.time = unix.time();
  state.soundFilter = wcm.get_sound_detFilter();
  state.soundDetection = wcm.get_sound_detection();

  s = serialization.serialize_orig(state);
  Comm.send(s);

  unix.usleep(100000);
end
