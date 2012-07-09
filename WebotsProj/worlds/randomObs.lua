fname = 'op_obs_base.wbt'
f = io.open(fname, 'r')
assert(f, 'Could not open file')
header = f:read('*a')
--print(header)

fname_save = 'op_obs.wbt'
fs = io.open(fname_save, 'w+')


obs_num = 5;

fs:write(header)
for cnt = 1, obs_num do
  fs:write('DEF OBSTACLE'..cnt..' cylinder {\n')
  fs:write('\theight 0.5\n')
  fs:write('}\n\n');
end

fs:close();

