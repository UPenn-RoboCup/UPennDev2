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

-- Get absolute path for THOR repository
-- If the script calling this one is not in a subdirectory,
-- then the THOR_HOME env variable must be set.

local pwd = shell('pwd')
THOR_HOME = os.getenv('THOR_HOME') or string.gsub(pwd, '/Run.*$', '')

-- set path for lua modules 
package.path = THOR_HOME.."/Config/?.lua;"..package.path
package.path = THOR_HOME.."/Data/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Comms/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Cognition/Slam/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Motion/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Motion/Controls/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Motion/FSMs/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Motion/States/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Motion/States/Locomotion/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Motion/States/Manipulation/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Motion/States/Attention/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Platform/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Proprioception/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Cognition/?.lua;"..package.path
package.path = THOR_HOME.."/Framework/Util/?.lua;"..package.path

-- set path for c modules
csuffix = {'dylib', 'so'}
for i = 1, #csuffix do
  package.cpath = THOR_HOME.."/Framework/Cognition/Slam/?."..csuffix[i]..";"..package.cpath
  package.cpath = THOR_HOME.."/Framework/Lib/?/?."..csuffix[i]..";"..package.cpath
  package.cpath = THOR_HOME.."/Framework/Lib/lcm/?."..csuffix[i]..";"..package.cpath
  package.cpath = THOR_HOME.."/Framework/Lib/unix/?."..csuffix[i]..";"..package.cpath
  package.cpath = THOR_HOME.."/Framework/Lib/manip/?."..csuffix[i]..";"..package.cpath
  package.cpath = THOR_HOME.."/Framework/Platform/?."..csuffix[i]..";"..package.cpath
end
