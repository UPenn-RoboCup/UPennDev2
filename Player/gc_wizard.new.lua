---------------------------
-- Game State Manager --
---------------------------
dofile'include.lua'
local libGC = require'GameControlReceiver.ffi'
require'gcm'
local Body = require(Config.dev.body)
-- Cache some functions
local get_time = Body.get_time
local util = require'util'
local color = util.color
--
local ENABLE_COACH = false

-- Cleanly exit on Ctrl-C
local running, signal = true
local function shutdown()
	running = false
end
if not IS_WEBOTS then
  signal = require'signal'
  signal.signal("SIGINT", shutdown)
  signal.signal("SIGTERM", shutdown)
end

local teamId = 36
local gc = libGC.init(teamId, 1, '192.168.100.1')

local OUR_GAME
local ret, msg
local send_count, recv_count = 0, 0
local t, t_send, t_pkt = 0, 0, 0
local last_pkt, cur_pkt = -1, -1

local function process_packet(pkt, t)
	if not pkt then
		if OUR_GAME then
			local secs = gcm.get_game_secsremaining()
			secs = secs - (t - t_timeupdate)
			t_timeupdate = t
			gcm.set_game_secsremaining(secs)
		end
		return
	end
	local teams = pkt.teams
	OUR_GAME = teams[0].teamNumber==teamId or teams[1].teamNumber==teamId
	-- Set things
	gcm.set_game_state(pkt.state)
	gcm.set_game_secsremaining(pkt.secsRemaining)
	t_timeupdate = t
	-- Save the packet number
	last_pkt = cur_pkt
	cur_pkt = gc_pkt.packetNumber
	-- Send a return to the game controller
	ret, msg = gc:send_return()
	if msg then print("GC | Return error", msg) end
	-- Increment the number of packets we have received
	recv_count = recv_count + 1
	--
	if cur_pkt~=last_pkt then t_pkt = t end
	-- Print debug
	local state_name = libGC.state_to_name[pkt.state] or "UNKNOWN"
	local debug_str = table.concat({
		color("State: "..state_name 'yellow'),
		color('Packet '..cur_pkt, 'green'),
		color(gc_pkt.secsRemaining..' seconds left', 'red'),
		''
	}, '\n')
	print(debug_str)
end

local pkt
while running do
  collectgarbage('step')
	pkt = gc:wait_for_gc()
  t = get_time()
	process_packet(pkt, t)
  if ENABLE_COACH and t - t_send > SEND_RATE then
    t_send = t
		ret, msg = gc:send_coach("go team!")
    if msg then print("GC | Coach error", msg) end
    send_count = send_count + 1
  end
	if t - t_pkt > 2 then
		print("GC | TIMEOUT")
	end
end
