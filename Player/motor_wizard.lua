-----------------------------------------------------------------
-- Dynamixel Motor Communication
-- Performs callbacks on read/writes to the chain
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'

-- Libraries
local unix = require'unix'
local signal = require'signal'
local libDynamixel = require'libDynamixel2'

-- Setup the dynamixels array
local dynamixels = {}

-- Initialize the dynamixels
local spine_dynamixel = libDynamixel.new_bus()

-- Spine dynamixel
if spine_dynamixel then
  spine_dynamixel.name = 'Spine'
  spine_dynamixel.data = {}
  table.insert(dynamixels,spine_dynamixel)
  local last_spine = 0
  local spine_cnt = 0
  -- Set up the callback when joints were read
  spine_dynamixel.callback = function(data)
    
    -- Update the shared memory
    for k,v in pairs(data) do
      spine_dynamixel.data[k] = v
    end
    
    -- If finished a read, then stop the reading process
    spine_dynamixel.perform_read = false
    
    -- Show debugging output
    local t_diff = spine_dynamixel.t_last_read - last_spine
    spine_cnt = spine_cnt+1
    if t_diff>1 then
      local spine_fps = spine_cnt / t_diff
      local spine_debug = string.format('%s chain reading at %.2f Hz',
      spine_dynamixel.name,spine_fps)
      print()
      print(spine_debug)
      spine_cnt = 0
      last_spine = spine_dynamixel.t_last_read
    end
    end-- callback function
end -- if spine chain

-- Begin to service
assert(#dynamixels>0,"No dynamixels detected!")
io.write('Servicing ',#dynamixels,' Dynamixel chains.\n\n')
io.flush()

local main = function()
  local main_cnt = 0
  local t0 = unix.time()
  
  -- Enter the coroutine
  while true do
    
    --------------------
    -- Set commands for next sync
    if #spine_dynamixel.instructions==0 then
      local sync_command_cmd = 
      spine_dynamixel:set_mx_command( spine_dynamixel.ids_on_bus, 2048, true )
      --table.insert( spine_dynamixel.instructions, sync_command_cmd )
    end
    
    -- Update the read every so often
    if unix.time()-spine_dynamixel.t_last_read > 1 then
      spine_dynamixel.perform_read = true
    end
    
    --------------------
    -- Show debugging information
    main_cnt = main_cnt + 1
    local t_now = unix.time()
    local t_diff = t_now - t0
    if t_diff>1 then
      local debug_str = string.format('\nMain loop: %7.2f Hz',main_cnt/t_diff)
      for i,d in ipairs(dynamixels) do
        local last_seen = t_now-math.max(d.t_last_read,d.t_last_write)
        debug_str = debug_str..string.format(
        '\n\tDynamixel %s chain was seen %5.3f seconds ago',
        d.name,last_seen)
      end
      print(debug_str)
      t0 = t_now
      main_cnt = 0
    end
    
    --------------------
    -- Do not yield anything for now
    coroutine.yield()
    
  end
end

--------------------
-- Print the motor state upon shutdown
function shutdown()
  print'Shutting down the Dynamixel chains...'
  for i,d in ipairs(dynamixels) do
    d:close()
    io.write('\n',d.name,' chain\n')
    for k,v in pairs(d.data) do
      io.write('Motor ',k,' @ ',v,'\n')
    end
  end
  error('Finished!')
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

libDynamixel.service( dynamixels, main )