cwd = os.getenv('PWD') 
package.path = cwd..'/../../Player/?.lua;'..package.path
cwd = cwd ..'/../../Player'
require('init')
require('util')
require('vector')

io.write("Enter Triangle Grid Density and Fillrate: ");
io.flush();
trigrid_den, obs_fillrate = io.read("*number","*number");

--range_y = {-1.0, 1.0}
--range_x = {-1.5, 1.5}
range_y = {-2.0, 2.0}
range_x = {-2.5, 2.5}

trigrid = {}
trigrid.x = {}
trigrid.y = {}

--trigrid_den = 0.9
print('triangle grid minimum distance: '..trigrid_den);
trigrid_file = 'triangle_grid'..trigrid_den..'.txt';
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

--util.ptable(trigrid)


fname = 'op_obs_base.wbt'
f = io.open(fname, 'r')
assert(f, 'Could not open file')
header = f:read('*a')
--print(header)

fname_save = 'op_obs_auto.wbt'
fs = io.open(fname_save, 'w+')

--obs_fillrate = 1
obs_num = math.floor(obs_fillrate * trigrid.num + 0.5);
print('triangle grid fillrate: '..obs_fillrate..' with '..obs_num..' obs');


obs = {}
obs.x = {}
obs.y = {}
obs.pos = vector.zeros(trigrid.num)
obs.direct = {}
obs.direct_num = 0;
obs.side = {}
obs.side_num = 0;

math.randomseed( os.time() )
-- separate obs to directly facing obs and side obs
for cnt = 1, trigrid.num do
  if math.abs(trigrid.y[cnt]) < 0.2 then
    obs.direct_num = obs.direct_num + 1;
    obs.direct[obs.direct_num] = cnt;
  else
    obs.side_num = obs.side_num + 1;
    obs.side[obs.side_num] = cnt;
  end
end
print(trigrid.num..' Spots with '..obs.direct_num..
      ' Direct Spots and '..obs.side_num..' Side Spots');
obs_direct_num = math.random(math.min(obs.direct_num, obs_num));
obs_direct_num = math.random(math.min(obs.direct_num, obs_num));
obs_direct_num = math.random(math.min(obs.direct_num, obs_num));
obs_side_num = obs_num - obs_direct_num;

if obs_side_num > obs.side_num then
  obs_direct_num = obs_direct_num + (obs_side_num - obs.side_num);
  obs_side_num = obs.side_num
end

print('Randomize '..obs_direct_num..' directly facing obs and '..obs_side_num..' side obs');
for cnt = 1, obs_direct_num do 
  repeat
    index = math.random(obs.direct_num);
    index = math.random(obs.direct_num);
  until obs.pos[obs.direct[index]] == 0
  obs.pos[obs.direct[index]] = 1;
  obs.x[cnt] = trigrid.x[obs.direct[index]];
  obs.y[cnt] = trigrid.y[obs.direct[index]];
end
print('Random direct obs done!')
for cnt = 1, obs_side_num do 
  repeat
    index = math.random(obs.side_num);
    index = math.random(obs.side_num);
  until obs.pos[obs.side[index]] == 0
  obs.pos[obs.side[index]] = 1;
  obs.x[cnt + obs_direct_num] = trigrid.x[obs.side[index]];
  obs.y[cnt + obs_direct_num] = trigrid.y[obs.side[index]];
end


fs:write(header)
for cnt = 1, obs_num do
  fs:write('DEF OBSTACLE'..cnt..' cylinder {\n')
  fs:write('\ttranslation '..obs.x[cnt]..' 0.25 '..obs.y[cnt]..'\n')
  fs:write('}\n\n');
end

fs:close();

