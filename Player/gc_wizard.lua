#!/usr/bin/env luajit
---------------------------
-- Game State Manager --
---------------------------
dofile'include.lua'
local GC = require'GameControlReceiver'
require'gcm'
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

-- Update loop
local gc_pktNum, gc_state = 0
while running do
  t = get_time()
	-- Listen to game controller
	local gc_packet = GC.receive()
	if gc_packet and gc_packet.packetNumber>gc_pktNum then
		gc_state = gc_packet.state
		gcm.set_game_state(gc_state)
		gc_pktNum = gc_packet.packetNumber
	end

  -- If time for debug
  if t-t_debug>debug_interval then
    --os.execute('clear')
    t_debug = t
		print(string.format('Game state: %d | pktNum %d',
			gcm.get_game_state(), gc_pktNum))
	end
	collectgarbage('step')
	local t_s = (t_sleep - (get_time() - t))
	--if t_s>0 then usleep(1e6 * t_s) end
end

-- Exit
os.exit()
