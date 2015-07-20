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
local util = require'util'
local vector = require'vector'

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

-- Bit OR on blocks of 2x2 to get to labelB from labelA
local function block_bitor_bc(self)
  -- Zero the downsampled image
  ffi.fill(self.labelC_d, ffi.sizeof(self.labelC_d))
  local c_ptr = self.labelC_d
  -- Original and Offset a row
  local b_ptr = self.labelB_d
  local b_ptr1 = b_ptr + self.wb
  -- Start the loop
  for jb=1,self.hc do
    for ib=1,self.wc do
      c_ptr[0] = bor(b_ptr[0], b_ptr[1], b_ptr1[0], b_ptr1[1])
      -- Move to the next pixel
      c_ptr = c_ptr + 1
      b_ptr = b_ptr + 2
      b_ptr1 = b_ptr1 + 2
    end
    -- Move another row, too
    b_ptr = b_ptr + self.wb
    b_ptr1 = b_ptr1 + self.wb
  end
  return labelC_d
end
ImageProc.block_bitor_bc = block_bitor_bc

-- Bit OR on blocks of NxN to get to labelB from labelA
local function block_bitorN_ab(self)
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
      -- Remove background white
      if b_ptr[ind_b]==16 then b_ptr[ind_b] = 0 end
      -- if pure green
      --if b_ptr[ind_b]==8 then
        --b_ptr[ind_b] = band(a_ptr[0], a_ptr[1], a_ptr1[0], a_ptr1[1])
      --end
      a_ptr = a_ptr + 1
    end
  end
  return self.labelB_d
end
-- Bit OR on blocks of 2x2 to get to labelB from labelA
local function block_bitor2_ab(self)
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
      if b_ptr[0]==16 then
        -- Pure White: Remove background white
        b_ptr[0] = 0
      elseif b_ptr[0]==8 then
        -- Pure green: Remove background Green
        b_ptr[0] = band(a_ptr[0], a_ptr[1], a_ptr1[0], a_ptr1[1])
      end
      -- White alway on green...
      --if b_ptr[0] == 16 then b_ptr[0] = 24 end
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
function block_bitor_ab(self)
  --print(self.scaleB)
	if self.scaleB==2 then
	  block_bitor2_ab(self)
	else
	  block_bitorN_ab(self)
	end

  -- Grow2
  ----[[
  for i=1,3 do
    local b_ptr = self.labelB_d
    local b_ptr1 = b_ptr + self.wb
    local b_ptr2 = b_ptr + self.wb
    for jb=1,self.hb-2 do
      for ib=1,self.wb-2 do
        if b_ptr1[1]==16 then
          if band(b_ptr1[0],8)~=0 and b_ptr1[2]==0 then
            b_ptr1[0] = 24
            b_ptr1[2] = 24
          elseif b_ptr1[0]==0 and band(b_ptr1[2],8)~=0 then
            b_ptr1[0] = 24
            b_ptr1[2] = 24
          elseif b_ptr[1]==0 and band(b_ptr2[1], 8)~=0 then
            b_ptr[1] = 24
            b_ptr2[1] = 24
          elseif band(b_ptr[1], 8)~=0 and b_ptr2[1]==0 then
            b_ptr[1] = 24
            b_ptr2[1] = 24
          end
        end
        -- Move c
        b_ptr = b_ptr + 1
        b_ptr1 = b_ptr1 + 1
        b_ptr2 = b_ptr2 + 1
      end
    end
    -- TODO: Just run a copy
    --for i=0,ffi.sizeof(self.tmpB)-1 do self.labelB_d[i] = self.tmpB[i] end
  end
  --]]

  --[[
  -- Grow
  for i=1,2 do
    local b_ptr = self.labelB_d
    local b_ptr1 = b_ptr + self.wb
    local b_tptr = self.tmpB
    local b_tptr1 = b_tptr + self.wb
    for jb=1,self.hb-1 do
      for ib=1,self.wb-1 do
        local together = bor(b_ptr[0], b_ptr1[0], b_ptr[1], b_ptr1[1])
        b_tptr[0] = together
        b_tptr[1] = together
        b_tptr1[0] = together
        b_tptr1[1] = together
        -- Move c
        b_tptr = b_tptr + 1
        b_tptr1 = b_tptr1 + 1
        b_ptr = b_ptr + 1
        b_ptr1 = b_ptr1 + 1
      end
    end
    -- TODO: Just run a copy
    for i=0,ffi.sizeof(self.tmpB)-1 do self.labelB_d[i] = self.tmpB[i] end
  end
  --]]


  return self.labelB_d
end
ImageProc.block_bitor_ab = block_bitor_ab


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
	  block_bitor2(self)
	else
	  block_bitorN(self)
	end
end


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
function block_bitor_ac(self)
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

local function radon2ij(props, ith, ir)

  -- How far down the line
  local cnt = props.count_d[ith][ir]
  local lMean = props.line_sum_d[ith][ir] / cnt
  local lMin = props.line_min_d[ith][ir]
  local lMax = props.line_max_d[ith][ir]

  local flip = false
  if ith >= props.NTH then
    flip = true
    ith = ith - props.NTH
  end

  -- Closest point
  local s, c = props.sin_d[ith], props.cos_d[ith]
  if flip then
    --ir = -ir
    s = -s
    c = -c
  end

  local iR = ir * c * props.RSCALE
  local jR = ir * s * props.RSCALE
  local iMean = iR - lMean * s
  local jMean = jR + lMean * c

  local lineProp = {
    ir = ir,
    ith = ith,
    count = cnt,
    iMean = iMean,
    jMean = jMean,
    --
    iMin  = iR - lMin * s,
    jMin  = jR + lMin * c,
    --
    iMax  = iR - lMax * s,
    jMax  = jR + lMax * c,
  }

  return lineProp
end

local ptable = require'util'.ptable
local RadonTransform = require'ImageProc.ffi.RadonTransform'
function ImageProc.field_lines(label, w, h)
--print()
  local props = RadonTransform.radon_lines_label(label, w, h)

  ----[[
  -- Find the global max
  local cmax = 0
  local irmax = 0
  local ithmax = 0
  for ith=0, 2*props.NTH-1 do
  --for ith=0, props.NTH-1 do
    for ir=0, props.NR-1 do
      if props.count_d[ith][ir] > cmax then
        cmax = props.count_d[ith][ir]
        if ith >= props.NTH then
          irmax = -ir
          ithmax = ith - props.NTH
        else
          irmax = ir
          ithmax = ith
        end
      end
    end
  end
  --]]
  if cmax==0 then return end

  ----[[
  local irmaxes = {}
  local cmaxes = {}
  local min_width = 5
  local min_th = props.NTH / 6
  local ithMax, irMax1, irMax2
  local cntMax1, cntMax2 = 0, 0
  for ith=0, 2*props.NTH-1 do
    local irmx, cmx = 0, 0
    if ith < ithmax-min_th or ith>ithmax+min_th then
      for ir=0, props.NR-1 do
        local cval = props.count_d[ith][ir]
        if cval > cmx then
          cmx = cval
          if ith >= props.NTH then
            irmx = -ir
          else
            irmx = ir
          end
        end
      end -- end the inner for
    elseif ith==ithmax then
      for ir=0, props.NR-1 do
        if ir < irmax-min_width or ir > irmax+min_width then
          local cval = props.count_d[ith][ir]
          if cval > cmx then
            cmx = cval
            if ith >= props.NTH then
              irmx = -ir
            else
              irmx = ir
            end
          end
        end
      end -- end the inner for
    end
    cmaxes[ith+1] = cmx
    irmaxes[ith+1] = irmx
  end

  -- How many extra?
  local nKeep = 2
  local maxN = {}
  for ith, c in ipairs(cmaxes) do
    if #maxN<nKeep then
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

  local minCount = IS_WEBOTS and 32 or 64 --48
  local ijs = {}
  for i, v in ipairs(maxN) do
    --print('line (ith, ir, cnt)',unpack(v))
    if v[3]>=minCount then
      -- Check for flipping later...
      local ij = radon2ij(props, v[1], v[2])
      if ij then table.insert(ijs, ij) end
      --table.insert(ijs, radon2ij(props, v[1], v[2], true))
    end
  end
  --]]

  -- TODO: Line merge


  --print('ijs', #ijs)
  --util.ptable(props)
  return ijs, props

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

-- Setup should be able to quickly switch between cameras
-- i.e. not much overhead here.
-- Resize should be expensive at most n_cameras times (if all increase the sz)
function ImageProc.new(w, h, scaleA, scaleB, scaleC)
  scaleA = scaleA or 2
  scaleB = scaleB or 2
  scaleC = scaleC or 2
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
    tmpA = ffi.new('uint8_t[?]', ha * wa),
    tmpB = ffi.new('uint8_t[?]', hb * wb),
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
    block_bitor_ab = block_bitor_ab,
    block_bitor_ac = block_bitor_ac,
    block_bitor_bc = block_bitor_bc,
    --procC = procC,
    color_countA = color_countA,
    color_countB = color_countB,
    color_countC = color_countC,
	}
end

return ImageProc
