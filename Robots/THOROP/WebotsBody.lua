local WebotsBody = {}

local ww = require'world_wizard'

function WebotsBody.entry()
	ww.entry()
end

function WebotsBody.update()
	ww.update()
end

function WebotsBody.exit()
	ww.exit()
end

return WebotsBody