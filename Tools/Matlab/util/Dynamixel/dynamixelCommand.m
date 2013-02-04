function [id, err, params] = dynamixelCommand(id, inst, varargin);
% status = dynamixelCommand(id, inst, parameters)

global DYNAMIXEL

if isempty(DYNAMIXEL) || ~isfield(DYNAMIXEL, 'fid'),
  error('Dynamixel interface not open');
end

if length(varargin) > 0,
  instParams = [varargin{:}];
else
  instParams = [];
end
dynamixelInstruction(DYNAMIXEL.fid, id, inst, instParams);

if id == 254, % Broadcast
  id = [];
  err = [];
  params = [];
else
  [id, err, params] = dynamixelStatus(DYNAMIXEL.fid);
end
