-----------------------------
-- Setup the State Machine --
-----------------------------
-- Temporary globals
assert(type(transitions)=='table', 'Need a table of transitions')
assert(type(fsm_name)=='string', 'Need an fsm_name for the channel')

-- Necessary modules
local si = require'simple_ipc'
local fsm = require'fsm'
--local util = require'util'

-- Set up our transitions
local states, sm = {}
for _, v in ipairs(transitions) do
  local state0 = states[v[1]]
  if not state0 then
    -- Require if needed
    state0 = require(v[1])
    states[v[1]] = state0
    if not sm then sm = fsm.new(state0) else sm:add_state(state0) end
    --print(util.color('Loaded '..v[1], 'yellow'))
  end
  local state1 = states[v[3]]
  if not state1 then
    -- Require if needed
    state1 = require(v[3])
    states[v[3]] = state1
    if not sm then sm = fsm.new(state1) else sm:add_state(state1) end
    --print(util.color('Loaded '..v[3], 'yellow'))
  end
  local trigger = v[2]
  assert(type(trigger)=='string', 'Bad trigger')
end

for _, v in ipairs(transitions) do
  sm:set_transition(states[v[1]], v[2], states[v[3]])
end

--------------------------
-- Setup the FSM object --
--------------------------
local obj = {}
obj._NAME = fsm_name
obj.sm = sm
-- Simple IPC for remote state triggers
-- Add ! to invert
obj.evts = si.new_subscriber(obj._NAME..'!')
function obj:entry ()
  return self.sm:entry()
end
function obj:update ()
  -- Check for out of process events in non-blocking fashion
  local events = self.evts:receive(true)
  -- Event is a table now... :(
  if events then for _, event in ipairs(events) do
    --print(util.color(obj._NAME..' Event:','green'), event)
    print(obj._NAME, event)
    self.sm:add_event(event)
  end end
  return self.sm:update()
end
function obj.exit ()
  return self.sm:exit()
end

return obj
