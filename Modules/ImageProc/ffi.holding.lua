
--[[
image: labeled image
m: width of labeled image
n: height of labeled image
mask: label index (color code)
-- NOTE: 256 is the maximum number of regions
-- TODO: Increase the max
--]]
-- NOTE: the following two are static to the connected regions function!!
local label_array = ffi.new('int[256][256]')
local equiv_table = {
	n_label = 0,
	m_table = {},
}
function equiv_table.ensureAllocated(self, label)
	local m_table = self.m_table
	for i=#m_table,label do table.insert(m_table, i) end
end
function equiv_table.addEquivalence(self, label1, label2)
	local m_table = self.m_table
  while label1 ~= m_table[label1] do label1 = m_table[label1] end
	while label2 ~= m_table[label2] do label2 = m_table[label2] end
  if label1 < label2 then
    m_table[label2] = label1
	else
    m_table[label1] = label2
	end
end
function equiv_table.traverseLinks(self)
	local m_table = self.m_table
	for i=1,#m_table do m_table[i] = m_table[ m_table[i] ] end
end
function equiv_table.removeGaps(self)
	local m_table = self.m_table
	local next = 0
	for i=1,#m_table do
    m_table[i] = (i == m_table[i]) and next or m_table[m_table[i]]
		next = (i == m_table[i]) and next + 1 or next
	end
	self.n_label = next - 1
end

--[[
function ImageProc.connected_regions(image, m, n, mask)
	-- Ensure we have enough room to compute
  if m > NMAX or n > NMAX then return -1 end
	-- Clear the Equivelence table
  equiv_table.m_table = {}
	local nlabel = 1
  equiv_table:ensureAllocated(nlabel)
  -- Iterate over pixels in image:
  local n_neighbor = 0
  local label_neighbor = {0, 0, 0, 0}
	-- Loop over the image
	local pixel
  for j=1,n-1 do
    for i=1,m-1 do
			-- Grab the pixel
      pixel = image[0]; image = image + 1
			-- See if the pixel includes our mask
      if band(pixel, mask)==0 then
        label_array[i][j] = 0
				-- TODO: Fix
			else
				-- See if we have any neighbors who are our same mask
	      n_neighbor = 0
	      -- Check 4-connected neighboring pixels:
	      if ((i > 0) and (label_array[i-1][j])) then
	        label_neighbor[n_neighbor++] = label_array[i-1][j]
				end
	      if ((j > 0) and (label_array[i][j-1])) then
	        label_neighbor[n_neighbor++] = label_array[i][j-1]
				end
	      -- Check 8-connected neighboring pixels:
	      if ((i > 0) and (j > 0) and (label_array[i-1][j-1])) then
	        label_neighbor[n_neighbor++] = label_array[i-1][j-1]
				end
	      if ((i < n-1) and (j > 0) and (label_array[i+1][j-1])) then
	        label_neighbor[n_neighbor++] = label_array[i+1][j-1]
				end
	      local label
	      if (n_neighbor > 0) then
	        label = nlabel
	        -- Determine minimum neighbor label
	        for i_neighbor = 0,n_neighbor-1 do
	          if (label_neighbor[i_neighbor] < label) then label = label_neighbor[i_neighbor] end
					end
	        -- Update equivalences
					for i_neighbor = 0,n_neighbor-1 do
	          if (label ~= label_neighbor[i_neighbor]) then
	            equiv_table.addEquivalence(label, label_neighbor[i_neighbor])
						end
					end
	      else
					-- TODO: check the order of operations on the ++
	        --label = nlabel++
					nlabel = nlabel + 1; label = nlabel
	        equiv_table.ensureAllocated(label)
				end
	      -- Set label of current pixel
	      label_array[i][j] = label
			end
			-- if
		end
	end
	-- double for
  -- Clean up equivalence table
  equiv_table:traverseLinks()
  equiv_table:removeGaps()
  --nlabel = equiv_table:numLabel()
	nlabel = equiv_table.numLabel
	-- Make the RegionProps array
	local props = {}
	for i=1,nlabel do
		props[i] = setmetatable({
			area = 0,
			minI = math.huge,
			maxI = -math.huge,
			minJ = math.huge,
			maxJ = -math.huge,
			sumI = 0,
			sumJ = 0,
		}, RegionProps_mt)
	end
	-- TODO: This can be a single loop, not double for
	local label, prop
  for i=0,m-1 do
		for j=0,n-1 do
			-- Note: TraverseLinks() must be called before grabbing the label
      label = equiv_table.m_table[ label_array[i][j] ]
      if (label > 0) then
				prop = props[label]
			  prop.area = prop.area + 1
			  prop.sumI = prop.sumI + i
			  prop.sumJ = prop.sumJ + j
			  --if (i < prop.minI) then prop.minI = i end
			  --if (i > prop.maxI) then prop.maxI = i end
			  --if (j < prop.minJ) then prop.minJ = j end
			  --if (j > prop.maxJ) then prop.maxJ = j end
				prop.minI = i < prop.minI and i or prop.minI
				prop.maxI = i > prop.maxI and i or prop.maxI
				prop.minJ = j < prop.minJ and i or prop.minJ
				prop.maxJ = j > prop.maxJ and i or prop.maxJ
			end
    end
  end

	-- double for
	-- Sort by area
	table.sort(props)
	-- Return in a better format
	local regions = {}
	for i, prop in ipairs(props) do
		if prop.area == 0 then break end
		table.insert(regions, {
			area = prop.area,
			centroid = {props.sumI/props.area, props.sumJ/props.area},
			boundingBox = {props.minI, props.maxI, props.minJ, props.maxJ}
		})
	end
	return #regions>0 and regions
end
--]]
