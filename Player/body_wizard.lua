---------------------------
-- State Machine Manager --
-- (c) Stephen McGill    --
---------------------------
dofile'include.lua'
local Body = require(Config.dev.body)
-- Cache some functions
local get_time, usleep = Body.get_time, unix.usleep

-- Cleanly exit on Ctrl-C
local running, signal = true, nil
if not IS_WEBOTS then
  signal = require'signal'
  function shutdown ()
    running = false
    --os.exit()
  end
  signal.signal("SIGINT", shutdown)
  signal.signal("SIGTERM", shutdown)
end

-- Timing
local t_sleep = 1 / Config.fsm.update_rate
local t0, t = get_time()
local debug_interval, t_debug = 1.0, t0

-- Entry
Body.entry()
-- Update loop
while running do
  t = get_time()
  -- Update the body
  Body.update()
  -- If time for debug
  if t-t_debug>debug_interval then
    t_debug = t
		print(string.format('Body | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
    --print('Wire', vcm.get_wire_model())
	end
end

-- Exit
print'Exiting body wizard...'
Body.exit()

os.exit()
