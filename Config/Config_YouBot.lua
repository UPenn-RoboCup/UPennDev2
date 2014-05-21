-- The Config file is Global
Config = {}

assert(HOME, 'We need our HOME variable set')

------------------------
-- General parameters --
------------------------
Config.PLATFORM_NAME = 'YouBot'
Config.nJoint = 5

-----------------------
-- Device Interfaces --
-----------------------
Config.dev = {}
Config.dev.body = Config.PLATFORM_NAME..'Body'

---------------------------
-- Complementary Configs --
---------------------------
local exo = {'Net', 'FSM', 'Robot'}
-- Perform the load
for i, v in ipairs(exo) do
	local fname = {HOME, '/Config/Config_', Config.PLATFORM_NAME, '_', v, '.lua'}
	dofile(table.concat(fname))
end

---------------
-- Keyframes --
---------------
Config.km = {}

-------------
-- Cameras --
-------------
Config.camera = {}

table.insert(Config.camera,
{
	name = 'head',
	dev = '/dev/video1',
	fmt = 'yuyv',
	width = 320,
	height = 240,
	fps = 30,
	focal_length = 184.75,
	params = {

	},
})

return Config
