if ~exist('fid'),
    fid = serialopen('/dev/tty.usbserial-A10019XW', 1000000);
end

inst = dynamixelMove(2,512,100);
fwrite(fid,inst,'uint8');
pause(0.05);
status = fread(fid,inf,'uint8');

inst = dynamixelMove(3,512,100);
fwrite(fid,inst,'uint8');
pause(0.05);
status = fread(fid,inf,'uint8');

pause(1);

dp = 150;

inst = dynamixelMove(2,512+dp,100);
fwrite(fid,inst,'uint8');
pause(0.05);
status = fread(fid,inf,'uint8');

inst = dynamixelMove(3,512-2*dp,200);
fwrite(fid,inst,'uint8');
pause(0.05);
status = fread(fid,inf,'uint8');

pause(2);

sp = 1000;
inst = dynamixelMove(3,512,sp);
fwrite(fid,inst,'uint8');
pause(0.01);
inst = dynamixelMove(2,512,sp/2);
fwrite(fid,inst,'uint8');
pause(0.05);
status = fread(fid,inf,'uint8');
