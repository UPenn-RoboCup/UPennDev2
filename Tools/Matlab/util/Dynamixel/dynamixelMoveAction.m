function err = dynamixelMoveAction(ids, pos, speed);

addr = 30; % 0x1E

for i = 1:length(ids),
  id = ids(i);
  data = [rem(pos(i), 256) floor(pos(i)/256) ...
          rem(speed(i), 256) floor(speed(i)/256)];

  dynamixelRegWrite(id, addr, data);
end

dynamixelAction;
err = [];
