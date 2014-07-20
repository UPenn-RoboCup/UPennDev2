---------------------------
-- Game State Manager --
---------------------------
dofile'include.lua'
local libGC = require'GameControlReceiver.ffi'
require'gcm'
local Body = require(Config.dev.body)
-- Cache some functions
local get_time, usleep = Body.get_time, unix.usleep
local ENABLE_COACH = false

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

local gc = libGC.init(43, 1, '192.168.100.1')
local util = require'util'

local ret, msg
local send_count, recv_count = 0, 0
local t, t_send, t_debug = unix.time(), 0, 0
local last_pkt, cur_pkt = -1, -1
while running do
  collectgarbage('step')
  local gc_pkt = gc:wait_for_gc()
  if gc_pkt then
		gc_state = tonumber(gc_pkt.state)
		gcm.set_game_state(gc_state)
    cur_pkt = gc_pkt.packetNumber
    print(
      "\t",
      util.color("State: "..gc_state,'yellow'),
      util.color('Packet '..gc_pkt.packetNumber, 'green')
    )
    ret, msg = gc:send_return()
    if msg then print("Bad return", msg) end
    recv_count = recv_count + 1
  end
  t = get_time()
  if ENABLE_COACH and t - t_send > SEND_RATE then
    t_send = t
		ret, msg = gc:send_coach("go team!")
    if msg then print("Bad coach", msg) end
    send_count = send_count + 1
  end
  if t - t_debug > 2 then
    t_debug = t
    if cur_pkt==last_pkt then
		  print(string.format('TIMEOUT | Game state: %d | Received %d',
			  gcm.get_game_state(), recv_count))
    end
    last_pkt = cur_pkt
  end
end