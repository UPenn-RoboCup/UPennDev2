local function strip(str)
	return str:match( "^%s*(.-)%s*$" )
end

local function parse_file(filename)
	local objects = {}
	local current_object = ""
	local current_side = ""
	for line in io.lines(filename) do 
		if line:len() ~= 0 and line:sub(1,1) ~= '#' then
			if line == 'RIGHT' or line == 'LEFT' or line == 'CENTER' then
				current_side = line
				if not objects[current_object][current_side] then
					objects[current_object][current_side] = {}
				end
			elseif not line:find('=') then
				current_object = line
				current_side = nil
				if not objects[current_object] then
					objects[current_object] = {}
				end
			else
				-- Must be a variable
				equals,_ = line:find('=')
				name = strip(line:sub(1, equals-1))
				value = strip(line:sub(equals+1))
				if tonumber(value) then
					value = tonumber(value)
				end
				if current_side then
					objects[current_object][current_side][name] = value
				else
					objects[current_object][name] = value
				end
			end
		end
	end
	return objects
end

bodies = parse_file('original_values.txt')
trunnions = parse_file('trunnion_parameters.txt')

require('pl.pretty').dump(bodies)
print()
require('pl.pretty').dump(trunnions)
print()

for i,v in pairs(trunnions) do
	parent = v['Parent']
	print(parent)
	bodies[parent]['M'] = bodies[parent]['M'] - v['M']
end

require('pl.pretty').dump(bodies)
print()
