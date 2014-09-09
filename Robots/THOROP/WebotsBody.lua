local WebotsBody = {}

-- local ww = require'world_wizard'
local cw = require'camera_wizard'
local mw = require'mesh_wizard'
--local sw = require'slam_wizard'

local kb = require'test_robocup'
WebotsBody.USING_KB = type(kb)=='table' and type(kb.update)=='function'

function WebotsBody.entry()
	if ww then ww.entry() end
end

function WebotsBody.update_head_camera(img, sz, cnt, t)
	if cw then cw.update(img, sz, cnt, t) end
end

function WebotsBody.update_chest_lidar(metadata, ranges)
	if mw then mw.update(metadata, ranges) end
end

function WebotsBody.update_head_lidar(metadata, ranges)
  if sw then sw.update(metadata, ranges) end
end

function WebotsBody.update(keycode)
	if ww then ww.update() end

	if WebotsBody.USING_KB then kb.update(keycode) end
	-- Add logging capability
end

function WebotsBody.exit()
	if ww then ww.exit() end
end

return WebotsBody
