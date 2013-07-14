dofile'../include.lua'
local openni = require 'openni'
openni.disable_skeleton()
local n_users = openni.startup()
assert(n_users==0,'Should not use skeletons')

-- Verify stream
local depth_info, color_info = openni.stream_info()
assert(depth_info.width==320,'Bad depth resolution')
assert(color_info.width==320,'Bad color resolution')

local jp = require'jpeg'

while true do
	local depth, color = openni.update_rgbd()
	print('Updating',depth,color)

	----[[
	jdepth = jpeg.compress_16(depth,320,240,4)
	print('compress to ',#jdepth)
	f = io.open('depth.jpg','w')
	f:write(jdepth)
	f:close()
	--return
	--]]
	
	----[[
	jcolor = jpeg.compress_rgb(color,320,240)
	print('compress to ',#jcolor)
	f = io.open('color.jpg','w')
	f:write(jcolor)
	f:close()
	return
	--]]
	
end