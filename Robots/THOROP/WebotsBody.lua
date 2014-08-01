local WebotsBody = {}

local ww = require'world_wizard'
local cw = require'camera_wizard'

function WebotsBody.entry()
	ww.entry()
end

function WebotsBody.update_head_camera(img, sz, cnt, t)
	cw.update(img, sz, cnt, t)
end

function WebotsBody.update()
	ww.update()
end

function WebotsBody.exit()
	ww.exit()
end

return WebotsBody