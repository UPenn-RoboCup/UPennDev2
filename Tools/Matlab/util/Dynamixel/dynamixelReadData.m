function [data, err] = dynamixelReadData(id, addr, len);

if nargin < 2,
  addr = 1;
end
if nargin < 3,
  len = 1;
end

inst = 2;
[id_ret, err, params] = dynamixelCommand(id, inst, addr, len);

if id_ret ~= id,
  data = [];
  return;
end

data = params;
