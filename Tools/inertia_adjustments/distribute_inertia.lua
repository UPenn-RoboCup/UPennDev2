local function strip(str)
	return str:match( "^%s*(.-)%s*$" )
end

local function parse_file(filename)
	local objects = {}
	local current_object = ""
	for line in io.lines(filename) do 
		if line:len() ~= 0 and line:sub(1,1) ~= '#' then
			if not line:find('=') then
				current_object = line
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
				objects[current_object][name] = value
			end
		end
	end
	return objects
end

local function move_mass(parent, trunnion)
	old_mass = parent['M']
	parent['M'] = old_mass - trunnion['M']
	parent['Xcbar'] = (parent['Xcbar']*old_mass - trunnion['Xcbar']*trunnion['M'])/parent['M']
	parent['Ycbar'] = (parent['Ycbar']*old_mass - trunnion['Ycbar']*trunnion['M'])/parent['M']
	parent['Zcbar'] = (parent['Zcbar']*old_mass - trunnion['Zcbar']*trunnion['M'])/parent['M']
end


bodies = parse_file('original_values.txt')
trunnions = parse_file('trunnion_parameters.txt')

require('pl.pretty').dump(bodies)
print()
require('pl.pretty').dump(trunnions)
print()

for i,trunnion in pairs(trunnions) do
	parent = bodies[trunnion['Parent']]
	parent_original = {}
	for i,x in pairs(parent) do
		parent_original[i] = x
	end
	
	move_mass(parent, trunnion)
	parent_new = {}
	for i,x in pairs(parent) do
		parent_new[i] = x
	end
	
	trunnion['M'] = -trunnion['M']
	move_mass(parent, trunnion)
	
	print(trunnion['Parent'])
	for i,x in pairs(parent) do
		print(i..':'..(x - parent_original[i]))
	end
	print()
end

require('pl.pretty').dump(bodies)
print()
