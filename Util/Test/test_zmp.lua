dofile'../../include.lua'
util=require'util'
unix=require'unix'
vector=require'vector'
Body=require'Body'
carray = require'carray'
os.execute'clear'

-- OP parameters
supportX = 0
supportY = 0.010
footY = 0.035
uLeftI=vector.new{-supportX,footY,0}
uRightI=vector.new{-supportX,-footY,0}
uTorsoI = vector.pose{0,0,0}
tZMP  = .165
tStep = 0.25

local base = collectgarbage('count')
print(util.color('Opening libZMP','green'))
libZMP=require'libZMP'
print('KBytes:',collectgarbage('count')-base)

print(util.color('Opening a new libZMP solver','green'))
s = libZMP.new_solver({
  tStep=tStep,
  tZMP=tZMP,
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
-- More: Config/Walk/Config_WebotsOP_Walk.lua
table.insert(step_seq, {1, {0,-0.01,0},{-0.01,-0.01},0.2,1})
table.insert(step_seq, {1, {0.18,0,0},{-0.01,-0.01},1,2})
table.insert(step_seq, {1, {-0.06,0.01,0},{-0.0,-0.02},0.1,3})
table.insert(step_seq, {1, {0.0,0,0},{-0.01,-0.0},0.2,4})
table.insert(step_seq, {2, {0,0,0},{0,0},0.10})
table.insert(step_seq, {0, {0.06,0,0},{0,0},0.5})

seq_duration = 0
for _,k in ipairs(step_seq) do
  seq_duration = seq_duration + k[4]
end

-- Generate the step queue
print(util.color('Generating the step queue...','green'))
s:generate_step_queue(step_seq,uLeftI,uRightI)

-- Update and solve:
f = io.open('com.tmp','w')
print(util.color('Update and solve the preview...','green'))
t0 = unix.time()
counter = 1
while not done do
  --print(util.color('Solving the preview for timestep...','green'),step)
  unix.usleep(1e6/100)
  done = s:update_preview( Body.get_time(), supportX, supportY )
  s:solve_preview()
  com = s:get_preview_com()
  print(counter,'CoM',com)
  if com.y~=0 then
    f:write(string.format('%f %f\n',com.x,com.y))
    if math.abs(com.y)>1 then os.exit() end
  end
  counter = counter + 1
end
t1 = unix.time()
f:close()
t_diff = t1-t0
print( util.color('Skew (%) & rate (Hz):','red'), 
  (t_diff-seq_duration)/seq_duration*100, counter/t_diff )
print(util.color('Preview state:','red'))
util.ptorch(s.preview.state)
print('KBytes:',collectgarbage('count')-base)

collectgarbage()
print('KBytes:',collectgarbage('count')-base)