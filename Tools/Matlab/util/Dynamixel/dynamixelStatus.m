function [id, err, params] = dynamixelStatus(fid, timeout);

if nargin < 2,
  timeout = 0.1;
end

% Default return values:
id = [];
err = [];
params = [];

t0 = clock;
retPkt = [];
nPkt = 6;

while (etime(clock, t0) < timeout && length(retPkt) < nPkt),
  retPkt = [retPkt fread(fid, inf, 'uint8')'];
end

if length(retPkt) < nPkt || retPkt(1) ~= 255 || retPkt(2) ~= 255,
  return;
end

id = retPkt(3);
len = retPkt(4);
err = retPkt(5);
nPkt = 4+len;

% Read additional bytes if necessary
while (etime(clock, t0) < timeout && length(retPkt) < nPkt),
  retPkt = [retPkt fread(fid, inf, 'uint8')'];
end

params = retPkt(6:nPkt-1);
if rem(sum(retPkt(3:nPkt)),256) ~= 255,
  warning('Checksum in status packet incorrect');
end
