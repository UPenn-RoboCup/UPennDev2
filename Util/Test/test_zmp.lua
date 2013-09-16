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
t0 = unix.time()
s:compute_preview()
t1 = unix.time()
--util.ptorch(s.K1)
print( util.color('Compute time (ms):','red'), (t1-t0)*1e3 )
print('KBytes:',collectgarbage('count')-base)

-- Print out the new solver elements
--for k,v in pairs(s) do print(k,v) end

-- Initialize the solver for a new run
supportX = 0
supportY = 0.010
footY = 0.035
uLeftI=vector.new{-supportX,footY,0}
uRightI=vector.new{-supportX,-footY,0}
uTorsoI = vector.pose{0,0,0}

print(util.color('Initializing the preview...','green'))
s:init_preview(uTorsoI,uLeftI,uRightI)
print(util.color('Preview state:','red'))
util.ptorch(s.preview.state)
print('KBytes:',collectgarbage('count')-base)
-- Print out the new solver elements
--for k,v in pairs(s) do print(k,v) end
--for k,v in pairs(s.preview) do print(k,v) end

-- Provide a sample step sequence
step_seq = {}
-- DS step
table.insert(step_seq, {2, {0,0,0}, {0,0}, 0.10})
-- LS step  
table.insert(step_seq, {0, {0.060,0,0}, {0,0}, 0.5})
-- DS step
table.insert(step_seq, {2, {0,0,0}, {0,0}, 0.05})
seq_duration = 0
for _,k in ipairs(step_seq) do
  seq_duration = seq_duration + k[4]
end

-- Generate the step queue
print(util.color('Generating the step queue...','green'))
s:generate_step_queue(step_seq,uLeftI,uRightI)

-- Update and solve:
print(util.color('Update and solve the preview...','green'))
t0 = unix.time()
counter = 0
while not done do
  --print(util.color('Solving the preview for timestep...','green'),step)
  done = s:update_preview( Body.get_time(), supportX, supportY )
  s:solve_preview()
  local com = s:get_preview_com()
  print('CoM',com)
  counter = counter + 1
end
t1 = unix.time()
t_diff = t1-t0
print( util.color('Skew (%) & rate (Hz):','red'), 
  (t_diff-seq_duration)/seq_duration*100, counter/t_diff )
print(util.color('Preview state:','red'))
util.ptorch(s.preview.state)
print('KBytes:',collectgarbage('count')-base)

collectgarbage()
print('KBytes:',collectgarbage('count')-base)