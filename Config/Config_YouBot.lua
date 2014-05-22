-- The Config file is Global
Config = {}
------------------------
-- General parameters --
------------------------
Config.PLATFORM_NAME = 'YouBot'

-----------------------
-- Device Interfaces --
-----------------------
Config.dev = {}
Config.dev.body = 'YouBotBody'

---------------------------
-- Complementary Configs --
---------------------------
local exo = {'Net', 'FSM', 'Robot', 'Vision'}
-- Perform the load
for i, v in ipairs(exo) do
	local fname = {HOME, '/Config/Config_', Config.PLATFORM_NAME, '_', v, '.lua'}
	dofile(table.concat(fname))
end

return Config
