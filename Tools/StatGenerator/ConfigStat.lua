cwd = os.getenv('PWD')
cwd = cwd..'/../../Player';
package.path = cwd..'/?.lua;'..package.path

package.path = cwd.."/Config/?.lua;"..package.path;
--[[
package.path = cwd.."/Config/Camera/?.lua;"..package.path;
package.path = cwd.."/Config/Robot/?.lua;"..package.path;
package.path = cwd.."/Config/Walk/?.lua;"..package.path;
package.path = cwd.."/Config/Kick/?.lua;"..package.path;
package.path = cwd.."/Config/Vision/?.lua;"..package.path;
package.path = cwd.."/Config/World/?.lua;"..package.path;
package.path = cwd.."/Config/FSM/?.lua;"..package.path;
--]]

require('init')

require('util')

function scandir(directory)
  local i, t, popen = 0, {}, io.popen
  for filename in popen('ls -a "'..directory..'"'):lines() do
    i = i + 1
    t[i] = filename
  end
  return t
end

-- Get File List
files = scandir('../../Player/Config')

ConfigFiles, i = {}, 0
-- Filter Config Files
for k, v in ipairs(files) do
  if string.find(v, 'Config_') == 1 then
    i = i + 1
    ConfigFiles[i] = string.gsub(v, '.lua', '');
  end
end

-- Load ConfigFiles
Platform = {}
for i = 1, 1 do -- #ConfigFiles do
--  Config = {}
--  Config = require(ConfigFiles[i])
--  Platform[i].Config = Config;
end
util.ptable(Platform)

Config = {}
require('Config_OP')
for k, v in pairs(Config) do
  if type(v) == 'table' then
    print(v._NAME)
    util.ptable(v)
  else
    print(v)
  end
end
--util.ptable(Config)
