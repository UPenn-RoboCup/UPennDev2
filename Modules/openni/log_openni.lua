local openni = require 'openni'
openni.enable_skeleton()
local n_users = openni.startup('/tmp/skel.log','record')
print( "Number of Skeletons:", n_users )

-- Assume 30FPS, run for 10 seconds
local fps = 30
local nsec = 5
local nframes = 5 * fps
for fr=1,nframes do
	openni.update_skeleton()
	print(fr)
end
-- Shutdown the skeleton
print("Shutting down the openni device...")
local shutdown_status = openni.shutdown()
print("Shutdown",shutdown_status)