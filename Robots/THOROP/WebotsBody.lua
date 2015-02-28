local WebotsBody = {}
local ww, cw, mw, kw, sw, fw, rw, kb
local ffi = require'ffi'
local vector = require'vector'
local quaternion = require'quaternion'
local carray = require'carray'

-- TODO: Put in world config
local IS_SUPERVISOR = true
local q0
local world_tags = {}
local world_configurations = {}
local world_obj = {
	'ALVIN',
	'STEP0',
	'STEP1',
	'STEP2',
	'RAMP0',
	'RAMP1',
	'RAMP2',
	'RUBBLE0',
	'RUBBLE1',
	'RUBBLE2',
	'OBSTACLE0'
}

-- Webots x is our y, Webots y is our z, Webots z is our x,
-- Our x is Webots z, Our y is Webots x, Our z is Webots y
local function get_translation(self)
	local p = webots.wb_supervisor_field_get_sf_vec3f(self.translation)
	return vector.new{p[3], p[1], p[2]}
end
local function set_translation(self, position)
	local p_wbt = carray.double({position[2], position[3], position[1] })
	webots.wb_supervisor_field_set_sf_vec3f( self.translation, p_wbt )
end
local function get_rotation(self)
	local aa = webots.wb_supervisor_field_get_sf_rotation(self.rotation)
	return quaternion.from_angle_axis(aa[4],{aa[3],aa[1],aa[2]})
end
local function set_rotation(self, orientation)
	local angle, axis = quaternion.angle_axis(orientation)
	webots.wb_supervisor_field_set_sf_rotation(
		self.rotation,
		carray.double{axis[2], axis[3], axis[1], angle}
	)
end

local function reset()

	-- Change objest configurations
	for name,history in pairs(world_configurations) do
		if name~='ALVIN' then
			local obj = world_tags[name]
			local t0, r0 = unpack(history[1])
		end
	end

	-- Reset the Robot
	local obj = world_tags['ALVIN']
	local history = world_configurations['ALVIN']
	-- Always use the first position
	local t0, r0 = unpack(history[1])
	obj:set_translation(t0)
	obj:set_rotation(r0)
	-- Debug
	print('# of configurations', #history)
end

local function init()
	for i, obj_name in ipairs(world_obj) do
		local node_tag = webots.wb_supervisor_node_get_from_def(obj_name)
		local tags = {
			node = node_tag,
			translation = webots.wb_supervisor_node_get_field(node_tag, "translation"),
			rotation = webots.wb_supervisor_node_get_field(node_tag, "rotation"),
			get_translation = get_translation,
			set_translation = set_translation,
			get_rotation = get_rotation,
			set_rotation = set_rotation,
		}
		world_tags[obj_name] = tags
		-- Save Configurations through a run
		world_configurations[obj_name] = {
			{tags:get_translation(), tags:get_rotation()}
		}
	end
end

-- Use it in WebotsBody
WebotsBody.reset = reset

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

	-- Check if supervisor
	if IS_SUPERVISOR then
		q0 = Body.get_position()
		init()
	end

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

--local depth_array = carray.float(depth.data, n_pixels)
local depth_fl = ffi.new('float[?]', 1)
local n_depth_fl = ffi.sizeof(depth_fl)
local fl_sz = ffi.sizeof('float')
function WebotsBody.update_chest_kinect(rgb, depth)
	local n_pixels = depth.width * depth.height
	if n_pixels~=n_depth_fl then depth_fl = ffi.new('float[?]', n_pixels) end
	local byte_sz = n_pixels * fl_sz
	ffi.copy(depth_fl, depth.data, byte_sz)
	-- Convert to mm
	for i=0,n_pixels-1 do depth_fl[i] = 1000 * depth_fl[i] end
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
