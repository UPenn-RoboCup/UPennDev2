dofile'../../include.lua'
util=require'util'
unix=require'unix'
os.execute'clear'

print(util.color('Testing libZMP','green'))

libZMP=require'libZMP'

s = libZMP.new_solver({
	tStep=.25,
	tZMP=.165,
	start_phase=.1,
	finish_phase=.9})

-- Print out the solver elements
for k,v in pairs(s) do print(k,v) end

-- Compute the preview segment
print(util.color('Computing the preview...','green'))
local t0 = unix.time()
s:compute_preview()
local t1 = unix.time()
--util.ptorch(s.K1)
print(util.color('Done!','green'))
print( util.color('Compute time (ms):','red'), (t1-t0)*1e3 )

-- Print out the new solver elements
for k,v in pairs(s) do print(k,v) end