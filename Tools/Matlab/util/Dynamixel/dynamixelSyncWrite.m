function err = dynamixelSyncWrite(ids, addr, data);

if addr < 17,
  warning('Writing to EEPROM');
end

bid = 254; % Broadcast
inst = 131; % 0x83

nids = length(ids);
[mdata, ndata] = size(data);
if (ndata ~= nids),
  error('Number of ids needs to match number of cols in data');
end

params = [ids(:)'; data];
params = [addr mdata params(:)'];
dynamixelCommand(bid, inst, params);

err = [];
