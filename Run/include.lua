-- sets include paths for lua module access
-- usage: dofile('include.lua')

local function shell(command)
  local pipe = io.popen(command, 'r')
  -- Sometimes in V-Rep we close the pipe before we've actually
  -- read anything. Make sure that we actually get our output!
  local result
  while result == nil do
    result = pipe:read('*a')
  end
  pipe:close()
  return result
end

-- get dynamic lib suffix
local uname = shell('uname') 
if string.match(uname, 'Darwin') then
  csuffix = 'dylib'
else
  csuffix = 'so'
end

-- Get absolute path prefix for code directory.
-- If the script calling this one is not in a subdirectory,
-- it must supply the prefix itself.
local pwd = shell('pwd')
local prefix = prefix or string.gsub(pwd, '/Run.*$', '')

-- set path for lua modules 
package.path = prefix.."/Config/?.lua;"..package.path
package.path = prefix.."/Data/?.lua;"..package.path
package.path = prefix.."/Framework/Comms/?.lua;"..package.path
package.path = prefix.."/Framework/Cognition/Slam/?.lua;"..package.path
package.path = prefix.."/Framework/Motion/?.lua;"..package.path
package.path = prefix.."/Framework/Motion/Controls/?.lua;"..package.path
package.path = prefix.."/Framework/Motion/FSMs/?.lua;"..package.path
package.path = prefix.."/Framework/Motion/States/?.lua;"..package.path
package.path = prefix.."/Framework/Platform/?.lua;"..package.path
package.path = prefix.."/Framework/Proprioception/?.lua;"..package.path
package.path = prefix.."/Framework/Cognition/Slam/?.lua;"..package.path
package.path = prefix.."/Framework/Util/?.lua;"..package.path

-- set path for c modules 
package.cpath = prefix.."/Framework/Cognition/Slam/?."..csuffix..";"..package.cpath
package.cpath = prefix.."/Framework/Lib/?/?."..csuffix..";"..package.cpath
package.cpath = prefix.."/Framework/Lib/lcm/?."..csuffix..";"..package.cpath
package.cpath = prefix.."/Framework/Lib/unix/?."..csuffix..";"..package.cpath
package.cpath = prefix.."/Framework/Cognition/Slam/?."..csuffix..";"..package.cpath
package.cpath = prefix.."/Framework/Platform/?."..csuffix..";"..package.cpath
