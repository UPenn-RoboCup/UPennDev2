function err = dynamixelMove(id, pos, speed);

if nargin < 2,
   pos = 512;
end
if nargin < 3,
   speed = 0;
end

addr = 30; % 0x1E

data = [rem(pos, 256) floor(pos/256) ...
        rem(speed, 256) floor(speed/256)];

err = dynamixelWriteData(id, addr, data);
