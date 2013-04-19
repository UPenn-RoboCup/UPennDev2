dofile('include.lua')

--------------------------------------------------------------------------------
-- Motion Terminal
--------------------------------------------------------------------------------

require('rpc')
require('vector')

-- connect to motion manager
local motion_manager_endpoint = 'tcp://localhost:12001'
local motion_manager = rpc.client.new(motion_manager_endpoint)
print('Attempting to connect to motion_manager at '..motion_manager_endpoint)
motion_manager:connect(nil)
motion_manager:set_timeout(0.05)

function set_timeout(...)
  motion_manager:set_timeout(...)
end

function call(...)
  local success = motion_manager:call(...)
  if (success) then
    return motion_manager:get_return_values()
  else
    print('error : ', motion_manager:get_return_values())
  end
end

function exit()
  motion_manager:close()
  os.exit()
end
