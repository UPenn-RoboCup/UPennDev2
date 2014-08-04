-- Generate a new fsm generator object
local fsm = {}

--[[-------
  Lua finite state machine implementation
  Usage:
  sm = fsm.new({state1, state2, state3, ...})
  sm:set_transition(state1, event, state2, action)
  sm:add_state(state)
  sm:add_event(event)

  sm:entry()
  sm:update()
  sm:exit()

  states are tables with member functions:
  state.entry(), event = state.update(), state.exit(event) 

  events are strings: "timeout", "done", etc.
  actions are optional functions to be called

  jordan:
  added
  statesNames - mapping state index to string name
  statesHash - mapping state string name to state index
  sm:set_state(state_string) -- sets the state by the string name

--]]-------

-- TODO: Remove getfenv, as it just 
-- associates each function in fsm.lua with an fsm:new() object
--local mt = getfenv()
local mt = {}
mt.__index = mt

function fsm.new(state1, ...)
  assert(type(state1) == "table", "No initial state")
  local o = {
		states = {state1, ...},
		reverseStates = {},
		statesNames = {},
		statesHash = {},
		transitions = {},
		actions = {},
	}
  for i = 1,#o.states do
    -- Reverse indexes of states
    o.reverseStates[o.states[i]] = i
    o.statesNames[i] = o.states[i]._NAME
    o.statesHash[o.statesNames[i]] = i
    -- Transition and action tables
    o.transitions[o.states[i]] = {}
    o.actions[o.states[i]] = {}
  end
  o.events = {}
  o.initialState = o.states[1]
  o.currentState = o.initialState
  o.previousState = nil
  o.nextState = nil
  o.nextAction = nil

  return setmetatable(o, mt)
end

-- Remove a transition
function mt.remove_transition(self, fromState, event, toState)
  -- Make sure we are valid
  assert(self.reverseStates[fromState], "Unknown from state")
  assert(type(event) == "string", "Unknown event")
  assert(self.reverseStates[toState], "Unknown to state")

  -- Remove this transition if a transitions table exists
  if self.transitions[fromState] and self.transitions[fromState][event] then
    self.transitions[fromState][event] = nil
  end
  -- Remove this action if an actions table exists
  if self.actions[fromState] and self.actions[fromState][event] then
    self.actions[fromState][event] = nil
  end

end

-- Add a transition
function mt.set_transition(self, fromState, event, toState, action)
  assert(self.reverseStates[fromState], "Unknown from state")
  assert(type(event) == "string", "Unknown event")
  assert(self.reverseStates[toState], "Unknown to state")
  if action then
    assert(type(action) == "function", "Unknown action function")
  end

  -- If no transitions yet from this state, then make a transitions table
  if not self.transitions[fromState] then self.transitions[fromState] = {} end
  self.transitions[fromState][event] = toState
  -- If no actions yet from this state, then make an actions table
  if not self.actions[fromState] then self.actions[fromState] = {} end
  self.actions[fromState][event] = action
end

function mt.add_state(self, newState)
  local n = #self.states
  self.states[n+1] = newState
  self.reverseStates[newState] = n+1
  self.statesNames[n+1] = newState._NAME
  self.statesHash[self.statesNames[n+1]] = n+1
  self.transitions[newState] = {}
  self.actions[newState] = {}
end

function mt.add_event(self, event)
  --self.events[#self.events+1] = event
	table.insert(self.events, event)
end

function mt.set_state(self, nextState)
	assert(self.statesHash[nextState], 'Unkown state '..nextState)
  self.nextState = self.states[self.statesHash[nextState]]
end

function mt.get_current_state(self)
  return self.currentState
end
function mt.get_previous_state(self)
  return self.previousState
end

function mt.entry(self)
  local state = self.currentState
  if self.state_debug_handle then
    self.state_debug_handle(self.currentState._NAME,event)
  end
  return state.entry()
end

function mt.update(self)
  local ret, event
  local state = self.currentState

  -- if no nextState update current state:
  if not self.nextState then
    ret = state.update()
    -- add ret from state to events:
    if ret then
      self.events[#self.events+1] = ret
    end

    -- process events
    for i = 1,#self.events do
      event = self.events[i]
      if self.transitions[state][event] then
        self.nextState = self.transitions[state][event]
        self.nextAction = self.actions[state][event]
        break
      end
    end
    self.events = {}
  end

  -- check and enter next state
  if self.nextState then
    -- Give the exit return to the function
    local exit_val = state.exit(event)

    -- The function can make changes to the exit_val
    if self.nextAction then
      exit_val = self.nextAction(exit_val) or exit_val
      self.nextAction = nil
    end

    self.previousState = self.currentState
    self.currentState = self.nextState
    
    self.nextState = nil
    self.currentState.entry(exit_val)
    if self.state_debug_handle then
      self.state_debug_handle(self.currentState._NAME,event)
    end
  end

  self.nextState = nil
  self.nextAction = nil

  return ret
end

function mt.exit(self)
  local state = self.currentState
  state.exit()
  self.currentState = self.initialState
  if self.state_debug_handle then
    self.state_debug_handle(self.currentState._NAME, event)
  end
end

function mt.set_state_debug_handle(self, h)
  self.state_debug_handle = h
end

--return setmetatable(fsm, mt)
return fsm