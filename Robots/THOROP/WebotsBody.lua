local WebotsBody = {}
local ww, cw, mw, kw, sw, fw, rw, kb
local ffi = require'ffi'

function WebotsBody.entry()

	cw = Config.sensors.head_camera and require(Config.sensors.head_camera)
  kw = Config.sensors.kinect and require(Config.sensors.kinect)
	mw = Config.sensors.chest_lidar and require(Config.sensors.chest_lidar)
	--
	fw = Config.sensors.feedback and require(Config.sensors.feedback)
  ww = Config.sensors.world and require(Config.sensors.world)
	kb = Config.testfile and require(Config.testfile)

	-- Marcell
	--rw = Config.wizards.remote and require(Config.wizards.remote)

	WebotsBody.USING_KB = type(kb)=='table' and type(kb.update)=='function'

	if ww then ww.entry() end
  if fw then fw.entry() end
  if rw then rw.entry() end
  if kw and kw.entry then kw.entry() end
end

function WebotsBody.update_head_camera(img, sz, cnt, t)
	if cw then cw.update(img, sz, cnt, t) end
end

function WebotsBody.update_head_lidar(metadata, ranges)
  if sw then sw.update(metadata, ranges) end
end

function WebotsBody.update_chest_lidar(metadata, ranges)
	if mw then mw.update(metadata, ranges) end
end

local depth_fl = ffi.new('float[?]', 1)
local n_depth_fl = ffi.sizeof(depth_fl)
local fl_sz = ffi.sizeof('float')
function WebotsBody.update_chest_kinect(rgb, depth)
	local n_pixels = depth.width * depth.height
	if n_pixels~=n_depth_fl then depth_fl = ffi.new('float[?]', n_pixels) end
	local byte_sz = n_pixels * fl_sz
	ffi.copy(depth_fl, depth.data, byte_sz)
	-- Convert to mm
	for i=1,n_pixels do depth_fl[i] = 1000 * depth_fl[i] end
	depth.data = ffi.string(depth_fl, byte_sz)
	depth.bpp = fl_sz
	if kw then kw.update(rgb, depth) end
end

function WebotsBody.update(keycode)
	if ww then ww.update() end
  if fw then fw.update() end
  if rw then rw.update() end

	if WebotsBody.USING_KB then kb.update(keycode) end
	-- Add logging capability
end

function WebotsBody.exit()
	if ww then ww.exit() end
end

return WebotsBody
