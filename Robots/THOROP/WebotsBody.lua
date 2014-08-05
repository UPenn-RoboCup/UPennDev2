local WebotsBody = {}

local ww = require'world_wizard'
local cw = require'camera_wizard'

local kb = require'test_robocup'
local USING_KB = type(kb)=='table' and type(kb.update)=='function'
WebotsBody.USING_KB = USING_KB

function WebotsBody.entry()
	ww.entry()
end

function WebotsBody.update_head_camera(img, sz, cnt, t)
	if cw then cw.update(img, sz, cnt, t) end
end

function WebotsBody.update(keycode)
	ww.update()
	if USING_KB then kb.update(keycode) end
end

function WebotsBody.exit()
	ww.exit()
end

return WebotsBody
