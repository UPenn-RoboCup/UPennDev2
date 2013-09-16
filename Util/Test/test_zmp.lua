dofile'../../include.lua'
util=require'util'
unix=require'unix'
vector=require'vector'
Body=require'Body'
os.execute'clear'

local base = collectgarbage('count')
print(util.color('Opening libZMP','green'))
libZMP=require'libZMP'
print('KBytes:',collectgarbage('count')-base)

print(util.color('Opening a new libZMP solver','green'))
s = libZMP.new_solver({
  tStep=.25,
  tZMP=.165,
  start_phase=.1,
  finish_phase=.9})
print('KBytes:',collectgarbage('count')-base)

-- Print out the solver elements
--for k,v in pairs(s) do print(k,v) end

-- Compute the preview segment
print(util.color('Computing the preview...','green'))
local t0 = unix.time()
s:compute_preview()
local t1 = unix.time()
--util.ptorch(s.K1)
print( util.color('Compute time (ms):','red'), (t1-t0)*1e3 )
print('KBytes:',collectgarbage('count')-base)

-- Print out the new solver elements
--for k,v in pairs(s) do print(k,v) end

-- Initialize the solver for a new run
uTorso = vector.pose{.01,0,0}
uLeft  = vector.pose{0,0.035,0}
uRight = vector.pose{0,-.035,0}
print(util.color('Initializing the preview...','green'))
s:init_preview(uTorso,uLeft,uRight)
print(util.color('Preview state:','red'))
util.ptorch(s.preview.state)
print('KBytes:',collectgarbage('count')-base)
-- Print out the new solver elements
--for k,v in pairs(s) do print(k,v) end
--for k,v in pairs(s.preview) do print(k,v) end

-- Provide a sample step sequence
step_seq = {}
-- From the drc webots for making steps?
gap0 = {0,0}
gap1 = {0.05,0}
supportX = 0
footY = 0.035
-- Initial waiting period
table.insert(step_seq, {
  {-supportX+gap0[1], footY+gap0[2], 0}, --uLeft
  {-supportX+gap0[1],-footY+gap0[2], 0}, --uRight
  2, -- Right support
  1, --duration
})
-- First crossing
table.insert(step_seq, {
  {-supportX+gap1[1], footY+gap1[2], 0}, --uLeft
  {-supportX+gap0[1],-footY+gap0[2], 0}, --uRight
  1, -- Right support
  0.8, --duration
})
-- Generate the step queue
print(util.color('Generating the step queue...','green'))
s:generate_step_queue(step_seq)

-- Update and solve:
print(util.color('Updating the preview...','green'))
s:update_preview( Body.get_time() )

for step=1,10 do
  print(util.color('Solving the preview for timestep...','green'),step)
  s:solve_preview()
end
print(util.color('Preview state:','red'))
util.ptorch(s.preview.state)
print('KBytes:',collectgarbage('count')-base)

collectgarbage()
print('KBytes:',collectgarbage('count')-base)