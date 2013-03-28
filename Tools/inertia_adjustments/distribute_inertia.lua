local pretty = require('pl.pretty')

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

local function perpendicular_distance(axis, a, b)
	local quadrance = 0
	for i, c in ipairs({'x', 'y', 'z'}) do
		if c ~= axis then
			quadrance = quadrance + (a[c..'cbar'] - b[c..'cbar'])^2
		end
	end
	return quadrance^(1/2)
end

local function cylinder_inertia(mass, radius, height)
	I_radial = 1/12*mass*(3*radius^2 + height^2)
	I_axial = 1/2*mass*radius^2
	return I_radial, I_axial
end

local function initialize_trunnion_inertia(trunnion)
	radial, axial = cylinder_inertia(trunnion['M'], trunnion['Radius'], trunnion['Height'])
	for i, c in ipairs({'x', 'y', 'z'}) do
		if c == trunnion['Axis'] then
			trunnion['I'..c..'c'] = axial
		else
			trunnion['I'..c..'c'] = radial
		end
	end
	for i, cs in ipairs({'xy', 'xz', 'yz'}) do
		trunnion['I'..cs..'c'] = 0
	end
end

local function adjust_com_component(component, parent, trunnion, new_parent)
	-- new_com = (old_com*old_mass - removed_com*removed_mass)/new_mass
	local c = component..'cbar'
	return (parent[c]*parent['M'] - trunnion[c]*trunnion['M'])/new_parent['M']
end

local function adjust_moi_component(component, parent, trunnion, new_parent)
	-- Parallel Axis Theorem:
	-- new_moi = old_moi + m*d^2
	
	local moi = 'I'..component..'c'
	local com = component..'cbar'
	
	trunnion_dist = perpendicular_distance(component, parent, trunnion)
	new_parent_dist = perpendicular_distance(component, parent, new_parent)
	
	-- new_moi = old_moi - (trunnion_moi + trunnion_mass*trunnion_dist^2) - new_mass*new_dist^2
	return parent[moi] - (trunnion[moi] + trunnion['M']*trunnion_dist^2) - new_parent['M']*new_parent_dist^2
end

local function adjust_poi_component(two_char_component, parent, trunnion, new_parent)
	-- Parallel Axis Theorem:
	-- new_poi_xy = old_poi_xy + m*dx*dy
	-- new_poi = old_poi - (trunnion_poi + trunnion_mass*trunnion_x*trunnion_y) - new_mass*new_x*new_y
	
	local poi = 'I'..two_char_component..'c'
	
	local trunnion_dist = 1
	local new_parent_dist = 1
	-- iterate over the two characters in "two_char_component"
	for i = 1, #two_char_component do
		c = two_char_component:sub(i,i)
		trunnion_dist = trunnion_dist * trunnion[c..'cbar']
		new_parent_dist = new_parent_dist * new_parent[c..'cbar']
	end
	
	return parent[poi] - (trunnion[poi] + trunnion['M']*trunnion_dist) - new_parent['M']*new_parent_dist
end

local function move_mass(parent, trunnion)
	local new_parent = {}
	for i,x in pairs(parent) do
		new_parent[i] = x
	end
	
	new_parent['M'] = parent['M'] - trunnion['M']
	for i, c in ipairs({'x', 'y', 'z'}) do
		new_parent[c..'cbar'] = adjust_com_component(c, parent, trunnion, new_parent)
		
		local inertia_c = 'I'..c..'c'
		new_parent[inertia_c] = adjust_moi_component(c, parent, trunnion, new_parent)
		if new_parent[inertia_c] < 0 then
			print('"'..trunnion['Parent']..'" inertia is negative! '..inertia_c..': '..new_parent[inertia_c])
		end
	end
	
	for i, cs in ipairs({'xy', 'xz', 'yz'}) do
		new_parent['I'..cs..'c'] = adjust_poi_component(cs, parent, trunnion, new_parent)
	end
	return new_parent
end



bodies = parse_file('original_values.txt')
trunnions = parse_file('trunnion_parameters.txt')

-- pretty.dump(bodies)
-- print()
-- pretty.dump(trunnions)
-- print()

for i,trunnion in pairs(trunnions) do
	initialize_trunnion_inertia(trunnion)
	
	local parent = bodies[trunnion['Parent']]
	
	local new_parent = move_mass(parent, trunnion)
	
	-- trunnion['M'] = -trunnion['M']
	-- local reverted_parent = move_mass(new_parent, trunnion)
	
	-- print(trunnion['Parent'])
	-- for i,x in pairs(reverted_parent) do
	-- 	print(i..':'..(x - parent[i]))
	-- end
	-- print()
end

-- pretty.dump(trunnions)
-- pretty.dump(bodies)
print()
