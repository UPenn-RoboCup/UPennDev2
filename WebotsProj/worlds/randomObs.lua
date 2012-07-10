cwd = os.getenv('PWD') 
package.path = cwd..'/../../Player/?.lua;'..package.path
cwd = cwd ..'/../../Player'
require('init')
require('util')

range_y = {-1.0, 1.0}
range_x = {-1.5, 1.5}

trigrid = {}
trigrid.x = {}
trigrid.y = {}

trigrid_file = 'triangle_grid.txt'
f = io.open(trigrid_file, 'r')
line = f:read("*l")
nline = 0;
while line ~= nil do

  y = tonumber(string.sub(line, 1, 18))
  x = tonumber(string.sub(line, 19))
  if (x >= range_x[1] and x <= range_x[2]) and ( y >= range_y[1] and y <= range_y[2]) then
      nline = nline + 1;
      trigrid.x[nline] = x;
      trigrid.y[nline] = y;
  end
--  print(x, y)
  line = f:read("*l")
end
trigrid.num = nline

util.ptable(trigrid)


fname = 'op_obs_base.wbt'
f = io.open(fname, 'r')
assert(f, 'Could not open file')
header = f:read('*a')
--print(header)

fname_save = 'op_obs.wbt'
fs = io.open(fname_save, 'w+')

obs_num = 5;
obs = {}
obs.x = {}
obs.y = {}
for cnt = 1, obs_num do 
  index = math.random(trigrid.num);
  obs.x[cnt] = trigrid.x[index];
  obs.y[cnt] = trigrid.y[index];
end

fs:write(header)
for cnt = 1, obs_num do
  fs:write('DEF OBSTACLE'..cnt..' cylinder {\n')
  fs:write('\ttranslation '..obs.x[cnt]..' 0.25 '..obs.y[cnt]..'\n')
  fs:write('\theight 0.5\n')
  fs:write('}\n\n');
end

fs:close();

