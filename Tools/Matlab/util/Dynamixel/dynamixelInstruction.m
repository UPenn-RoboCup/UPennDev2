function ret = dynamixelInstruction(fid, id, inst, params);
% status = dynamixelInstruction(fid, id, inst, params)

if nargin < 4,
  params = [];
end

instPkt = uint8([255 ...
                 255 ...
                 id ...
                 2+length(params) ...
                 inst ...
                 params(:)']);

checksum = 255-rem(sum(instPkt(3:end)),256);
instPkt(end+1) = checksum;

% Send instruction packet
ret = fwrite(fid, instPkt, 'uint8');
