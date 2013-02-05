if ~exist('fid'),
    fid = serialopen('/dev/tty.usbserial-A10019XW', 1000000);
end

id = 1;

%for pos = [0 1023 0 1023 0 1023],
%for pos = [512 700 512],
for pos = [450 550 512],

pause(0.5);

inst = zeros(1,5,'uint8');
inst(1) = 255;
inst(2) = 255;
inst(3) = id; % id
inst(4) = 2; % length
inst(5) = 3; % instruction
% Parameter 1,...N
inst(6) = hex2dec('1E');
inst(7) = rem(pos, 256);
inst(8) = floor(pos/256);
inst(9) = 0;
inst(10) = 0;

inst(4) = length(inst) - 3; % Rewrite length field

checksum = 255-rem(sum(inst(3:end)),256);
inst(end+1) = checksum;

fwrite(fid,inst,'uint8');
pause(0.05);
status = fread(fid,inf,'uint8')

end
