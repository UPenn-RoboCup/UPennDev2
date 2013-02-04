for i = 1:10,
  if dynamixelPing(i),
    fprintf(1, 'Found Dynamixel ID #%d\n', i);
  end
end

dynamixelMove(1, 512, 100);
dynamixelMove(2, 512, 100);
dynamixelMove(3, 512, 100);

dp = 150;

dynamixelMoveAction([2 3],512+[dp -2*dp],[100 200]);

pause(1);

sp = 1000;
dynamixelMoveAction([2 3],[512 512],[sp/2 sp]);

