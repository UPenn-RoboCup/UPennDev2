function [status, err] = dynamixelPing(id);

inst = 1;
[id_ret, err] = dynamixelCommand(id, inst);

if id_ret == id,
  status = 1;
else
  status = 0;
end
