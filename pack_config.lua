#!/usr/bin/env luajit
dofile'include.lua'
local mp = require'msgpack.MessagePack'
local mConfig = mp.pack(Config)
io.write(mConfig)

--[[
function stringy(instruction, a, b)
	local instruction_bytes = {}
	local instruction_ints = {}
	for i, v in ipairs({instruction:byte(a or 1, a and (b and b or -1))}) do
		table.insert(instruction_bytes, string.format(' %02X', v))
		table.insert(instruction_ints, string.format('%3d', v))
	end
	return table.concat(instruction_bytes, ' '), table.concat(instruction_ints, ' ')
end

io.stderr:write('nBytes writte:',#mConfig, '\n')
local s, e = 
stringy(mConfig,1,10),
stringy(mConfig,#mConfig-10,#mConfig)
io.stderr:write(s,'\n',e)
--]]
