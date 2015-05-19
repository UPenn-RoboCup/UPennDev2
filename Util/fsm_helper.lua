-- Helper in loading an fsm with triggers over channels
local fsm_helper = {}

-- Necessary modules
local fsm = require'fsm'
local color = require'util'.color
local new_subscriber = require'simple_ipc'.new_subscriber

-- Make the FSM from transitions table
local function make_fsm(transitions)
	assert(type(transitions)=='table', "BAD TRANSITIONS")
	local states, sm = {}
	for _, v in ipairs(transitions) do
		local state0 = states[v[1]]
		if not state0 then
			-- Require if needed
			state0 = require(v[1])
			states[v[1]] = state0
			if not sm then sm = fsm.new(state0) else sm:add_state(state0) end
		end
		local state1 = states[v[3]]
		if not state1 then
			-- Require if needed
			state1 = require(v[3])
			states[v[3]] = state1
			if not sm then sm = fsm.new(state1) else sm:add_state(state1) end
		end
		local trigger = v[2]
		assert(type(trigger)=='string', 'Bad trigger')
	end
	for _, v in ipairs(transitions) do
		sm:set_transition(states[v[1]], v[2], states[v[3]])
	end
	return sm
end

-- Check for out of process events in non-blocking fashion
local function update(self)
	local events = self.evts:receive(true)
	if events then
		for _, event in ipairs(events) do
			print(color(self._NAME, 'green'), event)
			self.sm:add_event(event)
		end
	end
	--local state = self.sm.currentState
	local ret = self.sm:update() --self.sm.events
	--if ret then print(color(state._NAME, 'magenta'), ret) end
	return ret
end

local function entry(self)
	return self.sm:entry()
end

local function exit(self)
	local ret = self.sm:exit()
	self.evts = nil
	return ret
end

local function wrap_fsm(fsm, name)
	assert(type(fsm)=='table',"No FSM given")
	assert(type(name)=='string', "Need a name for the FSM")
	local evts = new_subscriber(name..'FSM!')
	local obj = {
		_NAME = name..'FSM',
		sm = fsm,
		evts = evts,
		entry = entry,
		update = update,
		exit = exit,
	}
	return obj
end

-- NOTE: We can somehow require multiple same FSMs, now... (i.e. for two hands)
function fsm_helper.load(fsm_name)
	assert(type(fsm_name)=='string', 'Need the name of the FSM')
	assert(type(Config)=='table' and type(Config.fsm)=='table', 'Bad Config for FSM')
  -- Set some temporary globals
  local transitions = assert(Config.fsm[fsm_name], fsm_name..' does not exist')
	return wrap_fsm(make_fsm(transitions), fsm_name)
end

return fsm_helper
