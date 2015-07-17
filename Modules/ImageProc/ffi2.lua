-- libDetect
-- (c) 2014 Stephen McGill
-- General Detection methods
local ImageProc = {}
local lshift = require'bit'.lshift
local rshift = require'bit'.rshift
local band = require'bit'.band
local bor = require'bit'.bor
local ffi = require'ffi'
local log2 = {[1] = 0, [2] = 1, [4] = 2, [8] = 3, [16] = 4, [32] = 5, [64] = 6}

-- Load Lookup Table for Color -> Label
--[[
filename: LUT file of 262144 bytes
--]]
local function load_lut(self, filename)
  local f_lut = io.open(filename)
  -- We know the size of the LUT, so load the storage
  local lut_s = f_lut:read('*a')
  f_lut:close()
	assert(#lut_s==262144, 'Bad LUT size')
  -- Form a tensor for us
  local lut_d = ffi.new('uint8_t[?]', 262144, lut_s)
  table.insert(self.luts, lut_d)
  -- Return LUT and the id of this LUT
	print('Loaded', filename)
  return lut_d, #self.luts
end
ImageProc.load_lut = load_lut

--[[
yuyv_ptr: YUYV image as string, lightuserdata, or cdata
lut_ptr: Lookup table as string, lightuserdata, or cdata
--
returns: labelA array
--]]
-- Assumes a subscale of 2 (i.e. drop every other column and row)
function yuyv_to_labelA(self, yuyv_ptr, lut_ptr)
  -- The yuyv pointer changes each time
  -- Cast the lightuserdata to cdata
  local yuyv_d = ffi.cast("uint32_t*", yuyv_ptr)
  -- Set the LUT Raw data
  local lut_d = ffi.cast("uint8_t*", lut_ptr or self.luts[1])
  -- Temporary variables for the loop
  -- NOTE: 4 bytes yields 2 pixels, so stride of (4/2)*w
  local a_ptr, stride = self.labelA_d, self.w / 2
  for j=1,self.ha do
    for i=1,self.wa do
      -- Set the label
			--[[
      a_ptr[0] = lut_d[bor(
        rshift(band(yuyv_d[0], 0xFC000000), 26),
        rshift(band(yuyv_d[0], 0x0000FC00), 4),
        lshift(band(yuyv_d[0], 0xFC), 10)
      )]
			--]]
      a_ptr[0] = lut_d[bor(
        rshift(yuyv_d[0], 26),
        rshift(band(yuyv_d[0], 0x0000FC00), 4),
        lshift(band(yuyv_d[0], 0xFC), 10)
      )]
      -- Move the labelA pointer
      a_ptr = a_ptr + 1
      -- Move the image pointer
      yuyv_d = yuyv_d + 1
    end
    -- stride to next line
    yuyv_d = yuyv_d + stride
  end
  --
  return self.labelA_d
end
ImageProc.yuyv_to_labelA = yuyv_to_labelA

function rgb_to_labelA(self, rgb_ptr, lut_ptr)
  -- The yuyv pointer changes each time
  -- Cast the lightuserdata to cdata
  local rgb_d = ffi.cast("uint8_t*", rgb_ptr)
  -- Set the LUT Raw data
  local lut_d = ffi.cast("uint8_t*", lut_ptr or self.luts[1])
  -- Temporary variables for the loop
  local a_ptr = self.labelA_d
	--local r, g, b
	--local y, u, v
  for j=1,self.ha do
    for i=1,self.wa do
			----[[
      -- Get Pixel
      r, g, b = rgb_d[0], rgb_d[1], rgb_d[2]
			-- Convert to YUV
      y = g
      u = 128 + (b - g)/2
      v = 128 + (r - g)/2
      a_ptr[0] = lut_d[bor(
        rshift(band(v, 0xFC), 2),
        lshift(band(u, 0xFC), 4),
        lshift(band(y, 0xFC), 10)
      )]
			--]]
			--[[
			-- Set the label
      a_ptr[0] = lut_d[bor(
        rshift(128 + (rgb_d[0] - rgb_d[1])/2, 0xFC),
        lshift(band(128 + (rgb_d[2] - rgb_d[1])/2, 0xFC), 4),
        lshift(band(rgb_d[1], 0xFC), 10)
      )]
			--]]
      -- Move the labelA pointer
      a_ptr = a_ptr + 1
      -- Move the image pointer
      rgb_d = rgb_d + 3
    end
  end
  --
  return self.labelA_d
end
ImageProc.rgb_to_labelA = rgb_to_labelA

-- Bit OR on blocks of NxN to get to labelB from labelA
local function block_bitorN(self)
  -- Zero the downsampled image
	local a_ptr, b_ptr = self.labelA_d, self.labelB_d
  ffi.fill(b_ptr, ffi.sizeof(b_ptr))
	-- Begin the loop
  local jy, iy, ind_b, off_j
  for jx=0,self.ha-1 do
    jy = rshift(jx, log2[self.scaleB])
    off_j = jy * self.wb
    for ix=0,self.wa-1 do
      iy = rshift(ix, log2[self.scaleB])
      ind_b = iy + off_j
      b_ptr[ind_b] = bor(b_ptr[ind_b], a_ptr[0])
      a_ptr = a_ptr + 1
    end
  end
  return self.labelB_d
end
-- Bit OR on blocks of 2x2 to get to labelB from labelA
local function block_bitor2(self)
  --print('bit2')
  -- Zero the downsampled image
  ffi.fill(self.labelB_d, ffi.sizeof(self.labelB_d))
  local a_ptr, b_ptr = self.labelA_d, self.labelB_d
  -- Offset a row
  local a_ptr1 = a_ptr + self.wa
  -- Start the loop
  for jb=1,self.hb do
    for ib=1,self.wb do
      b_ptr[0] = bor(a_ptr[0], a_ptr[1], a_ptr1[0], a_ptr1[1])
      -- Remove background white
      if b_ptr[0]==16 then b_ptr[0] = 0 end
      -- Move b
      b_ptr = b_ptr + 1
      -- Move to the next pixel
      a_ptr = a_ptr + 2
      a_ptr1 = a_ptr1 + 2
    end
    -- Move another row, too
    a_ptr = a_ptr + self.wa
    a_ptr1 = a_ptr1 + self.wa
  end
  return labelB_d
end
function block_bitor(self)
  --print(self.scaleB)
	if self.scaleB==2 then
	  return block_bitor2(self)
	else
	  return block_bitorN(self)
	end
end
ImageProc.block_bitor = block_bitor


-- Bit OR on blocks of NxN to get to labelB from labelA
local function block_bitorCN(self)
  -- Zero the downsampled image
	local a_ptr = self.labelA_d
  local c_ptr = self.labelC_d
  ffi.fill(c_ptr, ffi.sizeof(c_ptr))
	-- Begin the loop
  local jy, iy, ind_c, off_j
  for jx=0,self.ha-1 do
    jy = rshift(jx, log2[scaleC])
    off_j = jy * self.wc
    for ix=0,self.wa-1 do
      iy = rshift(ix, log2[scaleC])
      ind_c = iy + off_j
      --c_ptr[ind_c] = band(c_ptr[ind_c], a_ptr[0])
      c_ptr[ind_c] = bor(c_ptr[ind_c], a_ptr[0])
      c_ptr[ind_c] = a_ptr[0]
      a_ptr = a_ptr + 1
    end
  end

  return self.labelC_d
end
-- Bit OR on blocks of 2x2 to get to labelB from labelA
local function block_bitorC2(self)
  -- Zero the downsampled image
  ffi.fill(self.labelC_d, ffi.sizeof(self.labelC_d))
  local a_ptr = self.labelA_d
  local c_ptr = self.labelC_d
  -- Offset a row
  local a_ptr1 = a_ptr + self.wa
  -- Start the loop
  for jc=1,self.hc do
    for ic=1,self.wc do
      c_ptr[0] = bor(a_ptr[0], a_ptr[1], a_ptr1[0], a_ptr1[1])
      --c_ptr[0] = a_ptr[0] -- straight subsample
      -- Move c
      c_ptr = c_ptr + 1
      -- Move to the next pixel
      a_ptr = a_ptr + 2
      a_ptr1 = a_ptr1 + 2
    end
    -- Move another row, too
    a_ptr = a_ptr + self.wa
    a_ptr1 = a_ptr1 + self.wa
  end

  return labelC_d
end
function procC(self)
	if self.scaleC==2 then
	  block_bitorC2(self)
	else
	  block_bitorCN(self)
	end
  ----[[
  -- Erode
  for i=1,2 do
    local c_ptr = self.labelC_d
    local c_ptr1 = c_ptr + self.wc
    for jc=1,self.hc-1 do
      for ic=1,self.wc-1 do
        -- Erode
        c_ptr[0] = band(c_ptr[0], c_ptr1[0], c_ptr[1], c_ptr1[1])
        -- Move c
        c_ptr = c_ptr + 1
        c_ptr1 = c_ptr1 + 1
      end
    end
  end
  --]]
  ----[[
  -- Grow
  for i=1,3 do
    c_ptr = self.labelC_d
    c_ptr1 = c_ptr + self.wc
    local c_tptr = self.tmpC
    local c_tptr1 = c_tptr + self.wc
    for jc=1,self.hc-1 do
      for ic=1,self.wc-1 do
        -- Erode
        local together = bor(c_ptr[0], c_ptr1[0], c_ptr[1], c_ptr1[1])
        c_tptr[0] = together
        c_tptr[1] = together
        c_tptr1[0] = together
        c_tptr1[1] = together
        -- Move c
        c_tptr = c_tptr + 1
        c_tptr1 = c_tptr1 + 1
        c_ptr = c_ptr + 1
        c_ptr1 = c_ptr1 + 1
      end
    end
    for i=0,ffi.sizeof(self.tmpC)-1 do self.labelC_d[i] = self.tmpC[i] end
  end
  --]]
  return self.labelC_d
end
ImageProc.block_bitor = block_bitor



function color_countA(self)
  -- Reset the color count
  ffi.fill(self.ccA_d, ffi.sizeof(self.ccA_d))
  -- Loop variables
  local l_ptr = self.labelA_d
  for i=1,self.wa*self.ha do
    self.ccA_d[l_ptr[0]] = self.ccA_d[l_ptr[0]] + 1
    l_ptr = l_ptr + 1
  end
  return self.ccA_d
end
ImageProc.color_countA = color_countA
function color_countB(self)
  -- Reset the color count
  ffi.fill(ccB_d, ffi.sizeof(ccB_d))
  -- Loop variables
  local l_ptr = self.labelB_d
  for i=1,self.wb*self.hb do
    self.ccB_d[l_ptr[0]] = self.ccB_d[l_ptr[0]] + 1
    l_ptr = l_ptr + 1
  end
  return self.ccB_d
end
ImageProc.color_countB = color_countB

-- Field lines
----[[

local function radon2ij(props, ith, ir, cnt)
  local s, c = props.sin_d[ith], props.cos_d[ith]
  -- How far down the line
  local lMean = props.line_sum_d[ith][ir] / cnt
  local lMin = props.line_min_d[ith][ir]
  local lMax = props.line_max_d[ith][ir]

  if ith > props.NTH/2 then
    ir = -1*ir
    --iR = -iR
    --jR = -jR
  end

  -- Closest point
  local iR = props.RSCALE * ir * c -- + props.i0
  local jR = props.RSCALE * ir * s -- + props.j0

  --[[
  print()
  local r, th = props.RSCALE * ir, ith/props.NTH * 180
  print('r, th', r, th, cnt)
  local ct = r * math.cos(th*DEG_TO_RAD)
  local st = r * math.sin(th*DEG_TO_RAD)

  print('iR, jR       ', iR, jR)
  print('Closest point', ct, st)
  print('lMean', lMean)
  print('lMin', lMin)
  print('lMax', lMax)
  --print('s, c', s, c)

  --print('iMean', iR - lMean * s)
  print('iR', iR)
  print('jR', jR)
  print('cMean', lMean * c)
  print('sMean', lMean * s)
  print('cMin', lMin * c)
  print('sMin', lMin * s)
  print('cMax', lMax * c)
  print('sMax', lMax * s)
  --]]

  local lineProp = {
    ir = ir,
    ith = ith,
    count = cnt,
    iMean = iR - lMean * s,
    jMean = jR + lMean * c,
    --
    iMin  = iR - lMin * s,
    jMin  = jR + lMin * c,
    iMax  = iR - lMax * s,
    jMax  = jR + lMax * c,
  }

  lineProp.endpoint = {lineProp.iMin, lineProp.iMax, lineProp.jMin, lineProp.jMax}

  return lineProp
end

local ptable = require'util'.ptable
local RadonTransform = require'ImageProc.ffi.RadonTransform'
function ImageProc.field_lines(label, w, h)

  local props = RadonTransform.radon_lines_label(label, w, h)

  ----[[
  -- Find the global max
  local cmax = 0
  local irmax = 0
  local ithmax = 0
  for ith=0, props.NTH-1 do
    for ir=0, props.NR-1 do
      if props.count_d[ith][ir] > cmax then
        cmax = props.count_d[ith][ir]
        irmax = ir
        ithmax = ith
      end
    end
  end
  --]]

  --[[
  local irmaxes = {}
  local cmaxes = {}
  local min_width = 2
  local min_th = 2
  local ithMax, irMax1, irMax2
  local cntMax1, cntMax2 = 0, 0
  for ith=0, props.NTH-1 do
    local i_monotonic_max = 0
    local monotonic_max = 0
    local i_arr, c_arr = {}, {}
    if ith >= ithmax-min_th and ith<=ithmax+min_th and ith ~= ithmax then
      i_arr, c_arr = {0,0}, {0,0}
    else
      for ir=0, props.NR-1 do
        local val = props.count_d[ith][ir]
        if val > monotonic_max then
          monotonic_max = val
          i_monotonic_max = ir
        elseif #c_arr<2 then
          -- End of monotonicity
          table.insert(i_arr, i_monotonic_max)
          table.insert(c_arr, monotonic_max)
          i_monotonic_max = ir
          monotonic_max = 0
        elseif (ir - i_monotonic_max) > min_width then
          if monotonic_max>c_arr[1]  then
            c_arr[1] = monotonic_max
            i_arr[1] = i_monotonic_max
          elseif monotonic_max>c_arr[#c_arr] then
            c_arr[#c_arr] = monotonic_max
            i_arr[#i_arr] = i_monotonic_max
          end
          i_monotonic_max = ir
          monotonic_max = 0
        end
      end -- end the inner for
    end
    irmaxes[ith+1] = i_arr
    cmaxes[ith+1] = c_arr
    print('th:', ith+1, unpack(c_arr))
  end
--]]

  ----[[
  local irmaxes = {}
  local cmaxes = {}
  local min_width = 3
  local min_th = 3
  local ithMax, irMax1, irMax2
  local cntMax1, cntMax2 = 0, 0
  for ith=0, props.NTH-1 do
    local irmx, cmx = 0, 0
    if ith < ithmax-min_th or ith>ithmax+min_th then
      for ir=0, props.NR-1 do
        local cval = props.count_d[ith][ir]
        if cval > cmx then irmx = ir; cmx = cval; end
      end -- end the inner for
    elseif ith==ithmax then
      for ir=0, props.NR-1 do
        if ir < irmax-min_width or ir > irmax+min_width then
          local cval = props.count_d[ith][ir]
          if cval > cmx then irmx = ir; cmx = cval; end
        end
      end -- end the inner for
    end
    irmaxes[ith+1] = irmx
    cmaxes[ith+1] = cmx
  end

  -- How many extra?
  local nKeep = 2
  local maxN = {}
  for ith, c in ipairs(cmaxes) do
    --if c<100 then
    if c<150 then
    elseif #maxN<nKeep then
      table.insert(maxN, {ith-1, irmaxes[ith], c})
      -- check merge
      table.sort(maxN, function(a,b) return a[3]>b[3] end)
    elseif c>maxN[nKeep][3] then
      maxN[nKeep] = {ith-1, irmaxes[ith], c}
      -- check merge
      table.sort(maxN, function(a,b) return a[3]>b[3] end)
    end
  end
  table.insert(maxN, 1, {ithmax, irmax, cmax})
  local ijs = {}
  for i, v in ipairs(maxN) do
    ijs[i] = radon2ij(props, unpack(v))
  end
  --]]

  return ijs, props
  --[[

  -- Have a minimum width of the line (in pixel space)
  local min_width = 1
  local i_monotonic_max, monotonic_max, val
  local ithMax, irMax1, irMax2
  local cntMax1, cntMax2 = 0, 0
  local found = false

  local count_d = props.count_d
  local line_sum_d = props.line_sum_d
  local line_min_d = props.line_min_d
  local line_max_d = props.line_max_d
  local NTH = props.NTH
  local NR = props.NR
  local cos_d = props.cos_d
  local sin_d = props.sin_d
  local th_d = props.th_d
  local i0 = props.i0
  local j0 = props.j0
  local r0 = props.r0

  for ith=0, NTH-1 do
    for ir=0, NR-1 do

    end
  end
  --]]

end
--]]

local RegionProps_mt = {}
function RegionProps_mt.__lt(r1, r2)
	return r1.area > r2.area
end

-- Get the color stats for a bounding box
-- TODO: Add tilted color stats if needed
-- TODO: use the bbox in the for loop
local max = require'math'.max
local sqrt = require'math'.sqrt
local atan2 = require'math'.atan2
local min = require'math'.min
local max = require'math'.max
function ImageProc.color_stats(image, width, height, color, bbox)
  local i0 = 0
  local i1 = width - 1
  local j0 = 0
  local j1 = height - 1
  if bbox then
    i0 = max(i0, bbox[1])
    i1 = min(i1, bbox[2])
    j0 = max(j0, bbox[3])
    j1 = min(j1, bbox[4])
  end
	-- Move pointer to the bbox start
	image = image + j0 * width
	color = color or 1
	-- RegionProps based statistics
	local area = 0
	local minI, maxI = width - 1, 0
	local minJ, maxJ = height - 1, 0
	local sumI, sumJ = 0, 0
	local sumII, sumJJ, sumIJ = 0, 0, 0
  for j=j0,j1 do
    for i=i0,i1 do
			-- If our color, then update the RegionProps
			--if image[i] == color then
			if band(image[i], color)>0 then
				-- Increment area size
				area = area + 1
				-- Update min/max row/column values
				if i < minI then minI = i end
				if i > maxI then maxI = i end
				if j < minJ then minJ = j end
				if j > maxJ then maxJ = j end
				-- Update the sums
				sumI = sumI + i
				sumJ = sumJ + j
				sumII = sumII + i^2
				sumJJ = sumJJ + j^2
				sumIJ = sumIJ + i * j
      end
      -- If
    end
    image = image + width
  end
	if area==0 then return { area = area } end
	--
  local centroidI = sumI / area
  local centroidJ = sumJ / area
	--
	local covII = sumII/area - centroidI^2
	local covJJ = sumJJ/area - centroidJ^2
	local covIJ = sumIJ/area - centroidI*centroidJ
	local covTrace = covII + covJJ
	local covDet = covII*covJJ - covIJ^2
	local covFactor = sqrt(max(covTrace^2 - 4*covDet, 0))
	local covAdd = (covTrace + covFactor) / 2
	local covSubtract = max((covTrace - covFactor), 0) / 2
	--
	local axisMajor = sqrt(12 * covAdd) + 0.5
	local axisMinor = sqrt(12 * covSubtract) + 0.5
	local orientation = atan2(covJJ-covIJ-covSubtract, covII-covIJ-covSubtract)
	--
	return setmetatable({
		area = area,
		centroid = {centroidI, centroidJ},
		boundingBox = {minI,maxI,minJ,maxJ},
		axisMajor = axisMajor,
		axisMinor = axisMinor,
		orientation = orientation,
		}, RegionProps_mt)
end

-- Get the color stats for a bounding box
-- TODO: Add tilted color stats if needed
-- TODO: use the bbox in the for loop
local max = require'math'.max
local sqrt = require'math'.sqrt
local atan2 = require'math'.atan2
local min = require'math'.min
local max = require'math'.max
function ImageProc.color_stats_exact(image, width, height, color, bbox)
  local i0 = 0
  local i1 = width - 1
  local j0 = 0
  local j1 = height - 1
  if bbox then
    i0 = max(i0, bbox[1])
    i1 = min(i1, bbox[2])
    j0 = max(j0, bbox[3])
    j1 = min(j1, bbox[4])
  end
	-- Move pointer to the bbox start
	image = image + j0 * width
	color = color or 1
	-- RegionProps based statistics
	local area = 0
	local minI, maxI = width - 1, 0
	local minJ, maxJ = height - 1, 0
	local sumI, sumJ = 0, 0
	local sumII, sumJJ, sumIJ = 0, 0, 0
  for j=j0,j1 do
    for i=i0,i1 do
			-- If our color, then update the RegionProps
			if image[i] == color then
			--if band(image[i], color)>0 then
				-- Increment area size
				area = area + 1
				-- Update min/max row/column values
				if i < minI then minI = i end
				if i > maxI then maxI = i end
				if j < minJ then minJ = j end
				if j > maxJ then maxJ = j end
				-- Update the sums
				sumI = sumI + i
				sumJ = sumJ + j
				sumII = sumII + i^2
				sumJJ = sumJJ + j^2
				sumIJ = sumIJ + i * j
      end
      -- If
    end
    image = image + width
  end
	if area==0 then return { area = area } end
	--
  local centroidI = sumI / area
  local centroidJ = sumJ / area
	--
	local covII = sumII/area - centroidI^2
	local covJJ = sumJJ/area - centroidJ^2
	local covIJ = sumIJ/area - centroidI*centroidJ
	local covTrace = covII + covJJ
	local covDet = covII*covJJ - covIJ^2
	local covFactor = sqrt(max(covTrace^2 - 4*covDet, 0))
	local covAdd = (covTrace + covFactor) / 2
	local covSubtract = max((covTrace - covFactor), 0) / 2
	--
	local axisMajor = sqrt(12 * covAdd) + 0.5
	local axisMinor = sqrt(12 * covSubtract) + 0.5
	local orientation = atan2(covJJ-covIJ-covSubtract, covII-covIJ-covSubtract)
	--
	return setmetatable({
		area = area,
		centroid = {centroidI, centroidJ},
		boundingBox = {minI,maxI,minJ,maxJ},
		axisMajor = axisMajor,
		axisMinor = axisMinor,
		orientation = orientation,
		}, RegionProps_mt)
end

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

-- Setup should be able to quickly switch between cameras
-- i.e. not much overhead here.
-- Resize should be expensive at most n_cameras times (if all increase the sz)
function ImageProc.new(w, h, scaleA, scaleB)
  scaleA = scaleA or 2
  scaleB = scaleB or 2
  scaleC = scaleC or 4
  local wa, ha = w / scaleA, h / scaleA
  local wb, hb = wa / scaleB, ha / scaleB
  local wc, hc = wa / scaleC, ha / scaleC
	return {
		-- Widths and Heights of Image, LabelA, LabelB
		w = w,
		h = h,
		wa = wa,
		ha = ha,
		wb = wb,
		hb = hb,
    wc = wc,
		hc = hc,
		-- Downsampling scales
		scaleA = scaleA,
		scaleB = scaleB,
    scaleC = scaleC,
		-- Memory allocation of the labels
		labelA_d = ffi.new('uint8_t[?]', ha * wa),
		labelB_d = ffi.new('uint8_t[?]', hb * wb),
    labelC_d = ffi.new('uint8_t[?]', hc * wc),
    tmpC = ffi.new('uint8_t[?]', hc * wc),
		-- Color count allocations
    ccA_d = ffi.new('int[256]'),
		ccB_d = ffi.new('int[256]'),
		ccC_d = ffi.new('int[256]'),
		luts = {},
		-- TODO: Functions?
    load_lut = load_lut,
    yuyv_to_labelA = yuyv_to_labelA,
    block_bitor = block_bitor,
    procC = procC,
    color_countA = color_countA,
    color_countB = color_countB,
    color_countC = color_countC,
	}
end

return ImageProc
