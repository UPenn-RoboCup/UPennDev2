---------------------------------
-- Body Wizard for Team THOR
-- This just runs the Body entry/update/exit
-- (c) Stephen McGill
---------------------------------
dofile'include.lua'
local Body = require'Body'
local simple_ipc = require'simple_ipc'
local mp = require'msgpack'
local signal = require'signal'
require'unix'

-- Send a Body pulse after each update
local pulse_ch = simple_ipc.new_publisher'pulse'

-- Get the timing
local t_wait = Body.update_cycle
local t_wait_us = t_wait*1e6

-- Clean Shutdown function
function shutdown()
  print'Shutting down the Body...'
  Body.exit()
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

local get_time = Body.get_sim_time or Body.get_time

local cnt = Body.entry()
local t_last = get_time()

-- Send the entry pulse
local pulse_tbl = {mode='entry',t=t_last}
pulse_ch:send(mp.pack(pulse_tbl))

pulse_tbl.mode='update'



while true do

  local t = get_time()
  local t_diff = t - t_last

  if t_diff>t_wait then
    --print('Update',t,cnt,t_last)
		t_last = t
    cnt = Body.update(cnt)
		local tt = unix.time()
		local t_sleep_us = 1e6*math.max(t_wait-(tt-t_last),0)
		--print('sleeping...',t_sleep_us,tt-t,t_wait-(tt-t_last))

    -- Webots must always update...
    if not IS_WEBOTS then unix.usleep(t_sleep_us) end
		-- Send a pulse after each update, so that the state wizard may proceed
		pulse_tbl.t = t
		--pulse_ch:send(mp.pack(pulse_tbl))
  else
    if not IS_WEBOTS then print('NOP CYCLE') end
    Body.nop()
		-- No pulse sent, since no update
  end

end

Body.exit()
pulse_tbl.t = get_time()
pulse_tbl.mode='exit'
pulse_ch:send(mp.pack(pulse_tbl))
