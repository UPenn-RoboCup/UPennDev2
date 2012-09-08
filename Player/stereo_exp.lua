dofile('init.lua')
require('unix')
require('vector')
require('Body')
require('SoundComm')
require('getch');
getch.enableblock(1);

headAngleMap = vector.new({0, 15, 30, 45, 60, 75, 90, 105}) * math.pi/180;


-- enable head movement
Body.set_head_command(Body.get_head_position())
Body.set_head_hardness(0.8);

prevDet = SoundComm.get_detection();

while (1) do
   det = SoundComm.get_detection();

   local cin = getch.get();
   local byte = nil;
   if (#cin > 0) then
      byte = string.byte(cin,1);
   end
   if (byte) then
      -- set head angles
      local ind = tonumber(string.char(byte));
      local a = headAngleMap[ind];
      if (a) then
         Body.set_head_command(a);
      end
   end

   if (det.count ~= prevDet.count) then
      prevDet = det;

      -- check left/right indices
      lindex = det.lIndex;
      rindex = det.rIndex;

      while (lindex > 512) do 
         lindex = lindex - 512;
      end
      while (rindex > 512) do 
         rindex = rindex - 512;
      end

      print('lindex - rindex = ', (lindex - rindex));
      if (math.abs(lindex - rindex) < 20) then
         -- idk what to do if there is a large difference
         mag = math.abs(lindex - rindex);
         if (lindex - rindex > 0) then
            dir = 1;
         else
            dir = -1;
         end

         -- update pan
         --print('dir: ', dir, ' mag: ', mag, ' rad: ', (mag*0.1))
         --headAngles = Body.get_head_position();
         --headAngles[1] = headAngles[1] - dir * (mag * 0.1);
         --Body.set_head_command(headAngles);
      end
   end

   unix.usleep(100000);
end
