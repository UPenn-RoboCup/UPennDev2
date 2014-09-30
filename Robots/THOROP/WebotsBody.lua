local WebotsBody = {}

local ww = Config.wizards.world and require(Config.wizards.world)
local cw = Config.wizards.camera and require(Config.wizards.camera)
local mw = Config.wizards.mesh and require(Config.wizards.mesh)
local sw = Config.wizards.slam and require(Config.wizards.slam)
local fw = Config.wizards.feedback and require(Config.wizards.feedback)
local kb = Config.testfile and require(Config.testfile)

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
  if fw then fw.update() end

	if WebotsBody.USING_KB then kb.update(keycode) end
	-- Add logging capability
end

function WebotsBody.exit()
	if ww then ww.exit() end
end

return WebotsBody
