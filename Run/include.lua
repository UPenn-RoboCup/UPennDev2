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
_HOME_ = prefix or string.gsub(pwd, '/Run.*$', '')

-- set path for lua modules 
package.path = _HOME_.."/Config/?.lua;"..package.path
package.path = _HOME_.."/Data/?.lua;"..package.path
package.path = _HOME_.."/Framework/Comms/?.lua;"..package.path
package.path = _HOME_.."/Framework/Cognition/Slam/?.lua;"..package.path
package.path = _HOME_.."/Framework/Motion/?.lua;"..package.path
package.path = _HOME_.."/Framework/Motion/Controls/?.lua;"..package.path
package.path = _HOME_.."/Framework/Motion/FSMs/?.lua;"..package.path
package.path = _HOME_.."/Framework/Motion/States/?.lua;"..package.path
package.path = _HOME_.."/Framework/Platform/?.lua;"..package.path
package.path = _HOME_.."/Framework/Proprioception/?.lua;"..package.path
package.path = _HOME_.."/Framework/Cognition/?.lua;"..package.path
package.path = _HOME_.."/Framework/Util/?.lua;"..package.path

-- set path for c modules 
package.cpath = _HOME_.."/Framework/Cognition/Slam/?."..csuffix..";"..package.cpath
package.cpath = _HOME_.."/Framework/Lib/?/?."..csuffix..";"..package.cpath
package.cpath = _HOME_.."/Framework/Lib/lcm/?."..csuffix..";"..package.cpath
package.cpath = _HOME_.."/Framework/Lib/unix/?."..csuffix..";"..package.cpath
package.cpath = _HOME_.."/Framework/Cognition/Slam/?."..csuffix..";"..package.cpath
package.cpath = _HOME_.."/Framework/Platform/?."..csuffix..";"..package.cpath
