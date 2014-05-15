-----------------------------
-- Setup the State Machine --
-----------------------------
-- Temporary globals
assert(type(transitions)=='table', 'Need a table of transitions')
assert(type(fsm_name)=='string', 'Need an fsm_name for the channel')

-- Necessary modules
local si = require'simple_ipc'
local fsm = require'fsm'
local util = require'util'

-- Set up our transitions
local states, sm = {}
for _, v in ipairs(transitions) do
  local state0 = states[v[1]]
  if not state0 then
    -- Require if needed
    state0 = require(v[1])
    states[v] = state0
    if not sm then sm = fsm.new(state0) else sm:add_state(state0) end
    util.color('Loaded '..v[1], 'yellow')
  end
  local state1 = states[v[3]]
  if not state1 then
    -- Require if needed
    state1 = require(v[3])
    states[v] = state1
    if not sm then sm = fsm.new(state1) else sm:add_state(state1) end
    util.color('Loaded '..v[3],'yellow')
  end
  local trigger = v[2]
  assert(type(trigger)=='string', 'Bad trigger')
  sm:set_transition(state0, trigger, state1)
end

--------------------------
-- Setup the FSM object --
--------------------------
local obj = {}
obj._NAME = fsm_name
-- Simple IPC for remote state triggers
-- Add ! to invert
local evts = si.new_subscriber(obj._NAME..'!')
local debug_str = util.color(obj._NAME..' Event:','green')
local special_evts = special_evts or {}
obj.entry = function()
  sm:entry()
end
obj.update = function()
  -- Check for out of process events in non-blocking fashion
  local event, has_more = evts:receive(true)
  if event then
    local debug_tbl = {debug_str,event}
    if has_more then
      extra = evts:receive(true)
      table.insert(debug_tbl,'Extra')
    end
    local special = special_evts[event]
    if special then
      special(extra)
      table.insert(debug_tbl,'Special!')
    end
    print(table.concat(debug_tbl,' '))
    sm:add_event(event)
  end
  sm:update()
end
obj.exit = function()
  sm:exit()
end

obj.sm = sm

return obj
