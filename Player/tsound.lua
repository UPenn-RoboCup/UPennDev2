dofile('init.lua')
require('SoundComm');
SoundComm.set_receiver_volume(75);

function write_out_pcm(x, filename)
   local datestr = os.date('%Y_%m_%d_%H_%M_%S');
   filename = filename or string.format('pcm_%s.csv', datestr);
   fh = io.open(filename, 'w');
   for i = 1,#x do
      fh:write(string.format('%d\n', x[i]));
   end
end


function listen_and_save()
   local lastDet = SoundComm.get_detection();

   while (1) do
      local det = SoundComm.get_detection();

      if (det.count ~= lastDet.count) then
         -- construct filename
         local datestr = os.date('%Y_%m_%d_%H_%M_%S');
         local filename = string.format('pcm_%s.csv', datestr);
         local x = SoundComm.get_debug_pcm();
         write_out_pcm(x, filename);
      end

      unix.usleep(10000);
   end
end

