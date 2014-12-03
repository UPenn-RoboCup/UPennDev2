-- libDetect
-- (c) 2014 Stephen McGill
-- General Detection methods
local ImageProc = {}
local vector = require'vector'
local lshift = require'bit'.lshift
local rshift = require'bit'.rshift
local band = require'bit'.band
local bor = require'bit'.bor
local ffi = require'ffi'

-- Widths and Heights of Image, LabelA, LabelB
local w, h, wa, ha, wb, hb

-- Load Lookup Table for Color -> Label
--[[
filename: LUT file of 262144 bytes
--]]
local luts = {}
ImageProc.luts = luts
function ImageProc.load_lut(filename)
  local f_lut = io.open(filename)
  -- We know the size of the LUT, so load the storage
  local lut_s = f_lut:read('*a')
  f_lut:close()
	assert(#lut_s==262144, 'Bad LUT size')
  -- Form a tensor for us
  local lut_d = ffi.new('uint8_t[?]', 262144, lut_s)
  table.insert(luts, lut_d)
  -- Return LUT and the id of this LUT
	print('Loaded', filename)
  return lut_d, #luts
end

--[[
yuyv_ptr: YUYV image as string, lightuserdata, or cdata
lut_ptr: Lookup table as string, lightuserdata, or cdata
--
returns: labelA array
--]]
-- Assumes a subscale of 2 (i.e. drop every other column and row)
-- Define labelA array
local labelA_d, labelA_n
function ImageProc.yuyv_to_labelA(yuyv_ptr, lut_ptr)
  -- The yuyv pointer changes each time
  -- Cast the lightuserdata to cdata
  local yuyv_d = ffi.cast("uint32_t*", yuyv_ptr)
  -- Set the LUT Raw data
  local lut_d = ffi.cast("uint8_t*", lut_ptr or luts[1])
  -- Temporary variables for the loop
  -- NOTE: 4 bytes yields 2 pixels, so stride of (4/2)*w
  local a_ptr, stride = labelA_d, w / 2
  for j=1,ha do
    for i=1,wa do
      -- Set the label
      a_ptr[0] = lut_d[bor(
        rshift(band(yuyv_d[0], 0xFC000000), 26),
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
  return labelA_d
end

-- Bit OR on blocks of NxN to get to labelB from labelA
local labelB_d, labelB_n, log_sB
local function block_bitorN()
  -- Zero the downsampled image
  ffi.fill(labelB_d, labelB_n)
	-- Begin the loop
  local a_ptr, b_ptr = labelA_d, labelB_d
  local jy, iy, ind_b, off_j
  for jx=0,ha-1 do
    jy = rshift(jx, log_sB)
    off_j = jy * wb
    for ix=0,wa-1 do
      iy = rshift(ix, log_sB)
      ind_b = iy + off_j
      b_ptr[ind_b] = bor(b_ptr[ind_b], a_ptr[0])
      a_ptr = a_ptr + 1
    end
  end
  return labelB_d
end
-- Bit OR on blocks of 2x2 to get to labelB from labelA
local function block_bitor2()
  -- Zero the downsampled image
  ffi.fill(labelB_d, labelB_n)
  local a_ptr, b_ptr = labelA_d, labelB_d
  -- Offset a row
  local a_ptr1 = a_ptr + wa
  -- Start the loop
  for jb=1,hb do
    for ib=1,wb do
      b_ptr[0] = bor(a_ptr[0],a_ptr[1],a_ptr1[0],a_ptr1[1])
      -- Move b
      b_ptr = b_ptr + 1
      -- Move to the next pixel
      a_ptr = a_ptr + 2
      a_ptr1 = a_ptr1 + 2
    end
    -- Move another row, too
    a_ptr = a_ptr + wa
    a_ptr1 = a_ptr1 + wa
  end
  return labelB_d
end

local ccA_d = ffi.new('int[256]')
local ccB_d = ffi.new('int[256]')
local cc_n = ffi.sizeof(ccA_d)
function ImageProc.color_countA()
  -- Reset the color count
  ffi.fill(ccA_d, cc_n)
  -- Loop variables
  local l_ptr = labelA_d
  for i=1,np_a do
    ccA_d[l_ptr[0]] = ccA_d[l_ptr[0]] + 1
    l_ptr = l_ptr + 1
  end
  return ccA_d
end
function ImageProc.color_countB()
  -- Reset the color count
  ffi.fill(ccB_d, cc_n)
  -- Loop variables
  local l_ptr = labelB_d
  for i=1,np_b do
    ccB_d[l_ptr[0]] = ccB_d[l_ptr[0]] + 1
    l_ptr = l_ptr + 1
  end
  return ccB_d
end

-- Get the color stats for a bounding box
-- TODO: Add tilted color stats if needed
-- TODO: use the bbox in the for loop
local max = require'math'.max
local sqrt = require'math'.sqrt
local atan2 = require'math'.atan2
function ImageProc.color_stats(label, color, bbox)
	local l_ptr = label=='a' and labelA_d or labelB_d
	local ni = label=='a' and wa or wb
	local nj = label=='a' and ha or hb
	--
  local i0 = 0
  local i1 = ni - 1
  local j0 = 0
  local j1 = nj - 1
  if bbox then
    -- TODO: bounds check
    i0 = bbox[1]
    i1 = bbox[2]
    j0 = bbox[3]
    j1 = bbox[4]
  end
  color = color or 1
	-- Initialize statistics
	local area = 0
	local minI, maxI = ni - 1, 0
	local minJ, maxJ = nj - 1, 0
	local sumI, sumJ = 0, 0
	local sumII, sumJJ, sumIJ = 0, 0, 0
  for j=j0,j1 do
    for i=i0,i1 do
			if l_ptr[i] == color then
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
    l_ptr = l_ptr + ni
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
	local covFactor = sqrt(max(covTrace*covTrace-4*covDet, 0))
	local covAdd = (covTrace + covFactor) / 2
	local covSubtract = max((covTrace - covFactor), 0) / 2
	--
	local axisMajor = sqrt(12*covAdd) + 0.5
	local axisMinor = sqrt(12*covSubtract) + 0.5
	local orientation = atan2(covJJ-covIJ-covSubtract, covII-covIJ-covSubtract)
	--
	return {
		area = area,
		centroid = {centroidI, centroidJ},
		boundingBox = {minI,maxI,minJ,maxJ},
		axisMajor = axisMajor,
		axisMinor = axisMinor,
		orientation = orientation,
	}
end

local equivalence_mt = {}
function equivalence_mt.addEquivalence(self, label1, label2)
	local m_table = self.m_table
  while label1 ~= m_table[label1] do label1 = m_table[label1] end
	while label2 ~= m_table[label2] do label2 = m_table[label2] end
  if label1 < label2 then
    m_table[label2] = label1
	else
    m_table[label1] = label2
	end
end
function equivalence_mt.traverseLinks(self)
	local m_table = self.m_table
	for i=1,#m_table do m_table[i] = m_table[ m_table[i] ] end
end
function equivalence_mt.removeGaps(self)
	local m_table = self.m_table
	local next = 0
	for i=1,#m_table do
    m_table[i] = (i == m_table[i]) and next or m_table[m_table[i]]
		next = (i == m_table[i]) and next + 1 or next
	end
	self.n_label = next - 1
end
function equivalence_mt.getEquivalentLabel(self, label)
	-- Note: TraverseLinks() must be called before this function
	return m_table[label]
end
function equivalence_mt.ensureAllocated(self, label)
	local m_table = self.m_table
	for i=#m_table,label do table.insert(m_table, i) end
end
function equivalence_mt.clear(self)
	self.m_table = {}
end
function equivalence_mt.numLabel(self)
  -- Note: RemoveGaps() must be called before this function
  return self.n_label
end
function gen_equivalence_table()
	return setmetatable({
		n_label = 0,
		m_table = {}
	}, equivalence_mt)
end

-- TODO: ConnectRegions

-- Setup should be able to quickly switch between cameras
-- i.e. not much overhead here.
-- Resize should be expensive at most n_cameras times (if all increase the sz)
-- Downscaling
local scaleA, scaleB
local log2 = {[1] = 0, [2] = 1, [4] = 2, [8] = 3, [16] = 4, [32] = 5, [64] = 6}
function ImageProc.setup(w0, h0, sA, sB)
  -- Save the scale paramter
  scaleA = sA or 2
  scaleB = sB or 2
  --log_sA = log2[scaleA]
	log_sB = log2[scaleB]
  -- Recompute the width and height of the images
  w, h = w0, h0
  wa, ha = w / scaleA, h / scaleA
  wb, hb = wa / scaleB, ha / scaleB
  -- Save the number of pixels
  np_a, np_b = wa * ha, wb * hb
	labelA_d = ffi.new('uint8_t[?]', ha*wa)
	labelB_d = ffi.new('uint8_t[?]', hb*wb)
	labelA_n = ffi.sizeof(labelA_d)
	labelB_n = ffi.sizeof(labelB_d)
	-- Allow access
	ImageProc.labelA_d, ImageProc.labelA_n = labelA_d, labelA_n
	ImageProc.labelB_d, ImageProc.labelB_n = labelB_d, labelB_n
  -- Select faster bit_or
  if scaleB==2 then
    ImageProc.block_bitor = block_bitor2
  else
    ImageProc.block_bitor = block_bitorN
  end
end

return ImageProc
