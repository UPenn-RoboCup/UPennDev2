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
-- Cleanly exit on Ctrl-C
local running = true
local function shutdown () running = false end
local signal
if not IS_WEBOTS then
  signal = require'signal'
  signal.signal("SIGINT", shutdown)
  signal.signal("SIGTERM", shutdown)
end

local ENABLE_COACH = false
local teamId = 36
local gc = libGC.init(teamId, 1, '192.168.100.1')

local OUR_GAME
local ret, msg
local send_count, recv_count = 0, 0
local t, t_send, t_pkt, t_switch = 0, 0, 0, 0
local last_pkt, cur_pkt, pkt = -1, -1
local led_count, last_position = 0
local rgb = {0, 0, 0}
gcm.set_game_role(Config.default_role or 2) --Testing
gcm.set_game_state(Config.default_state or 5) --Pre-init

--To stop the head movement for tester role
local head_ch = require'simple_ipc'.new_publisher('HeadFSM!')

local function process_switch(t)
	if t-t_switch < 1 then return end
	t_switch = t
	led_count = led_count +1
	local current_lidar_deg = Body.get_lidar_position()[1] * RAD_TO_DEG  
	local cur_position 
	if current_lidar_deg>45 then
		cur_position = 1
	elseif current_lidar_deg<-45 then
		cur_position = -1
	else
		cur_position = 0
	end
	-- Check the last position
	if cur_position~=last_position then
		last_position = cur_position
		local role, state
		if cur_position == 1 then
			role = 1 -- One: Attacker
			state = 0
		elseif cur_position == -1 then
			role = 0 --Zero: goalie
			state = 0
		else
			-- Testing role/stae
			role = 2
			state = 6
			head_ch:send'teleop'
			gcm.set_game_autoadvance(0)
		end
		-- Set in shared memory
		gcm.set_game_role(role)
		gcm.set_game_state(state)
		-- Set the LEDs
		if role==0 then
			rgb = {0,0,255}      
		elseif role==1 then
			rgb = {255,0,0}
		else
			rgb = {255,0,255}
		end
		Body.set_head_led_red(rgb[1]*intensity)
    Body.set_head_led_blue(rgb[3]*intensity)
    Body.set_head_led_green(rgb[3]*intensity)
	end
	-- Change the intensity if GC packets received
	local intensity = 1.0
	local gamecontroller_timeout = Config.gamecontroller_timeout or 5.0
	if get_time() - gcm.get_game_gctime() > gamecontroller_timeout then
		intensity = led_count % 2
	end
end

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
	-- If not our game, then return
	if not OUR_GAME then return end
	-- Set things
	gcm.set_game_gctime(t) -- GC time of received packet
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

while running do
  collectgarbage('step')
	pkt = gc:wait_for_gc()
  t = get_time()
	process_packet(pkt)
	process_switch()
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
