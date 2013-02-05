function err = dynamixelWriteData(id, addr, varargin);

if addr < 17,
  warning('Writing to EEPROM');
end

inst = 3;
[id_ret, err, params] = dynamixelCommand(id, inst, addr, varargin{:});

if id_ret ~= id,
  err = [];
  return;
end
