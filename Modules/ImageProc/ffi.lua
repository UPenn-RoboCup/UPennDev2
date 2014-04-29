-- libDetect
-- (c) 2014 Stephen McGill
-- General Detection methods
local ImageProc = {}
local torch  = require'torch'
local bit    = require'bit'
local lshift = bit.lshift
local rshift = bit.rshift
local band   = bit.band
local bor    = bit.bor
local RadonTransform = require'ImageProc.ffi.RadonTransform'

-- Widths and Heights of Image, LabelA, LabelB
local w, h, wa, ha, wb, hb
-- Form the labelA and labelB tensors
local labelA_t, labelB_t = torch.ByteTensor(), torch.ByteTensor()
-- Color Count always the same, as 8 bits for 8 colors means 256 color combos
local cc_t = torch.IntTensor(256)
-- The pointer will not change for this one
local cc_d = cc_t:data()
-- The Current Lookup table (Can be swapped dynamically)
local luts = {}
-- Downscaling
local scaleA, scaleB, log_sA, log_sB
local log2 = {[1] = 0, [2] = 1, [4] = 2, [8] = 3,}
-- For the edge system
local edge_t, edge_bin_t = torch.ByteTensor(), torch.ByteTensor()
local grey_t = torch.IntTensor()
local THRESH = 500

-- Load LookUp Table for Color -> Label
function ImageProc.load_lut (filename)
  local f_lut = torch.DiskFile( filename , 'r')
  f_lut.binary(f_lut)
  -- We know the size of the LUT, so load the storage
  local lut_s = f_lut:readByte(262144)
  f_lut:close()
  -- Form a tensor for us
  lut_t = torch.ByteTensor(lut_s)
  table.insert(luts,lut_t)
  -- Return the id of this LUT
  return #luts
end
-- Return the pointer to the LUT
function ImageProc.get_lut (lut_id)
  local lut_t = luts[lut_id]
  if not lut_t then return end
  return lut_t
end

-- Take in a pointer (or string) to the image
-- Take in the lookup table, too
-- Return labelA and the color count
-- Assumes a subscale of 2 (i.e. drop every other column and row)
-- Should be dropin for previous method
function ImageProc.yuyv_to_label (yuyv_ptr, lut_ptr)
  -- The yuyv pointer changes each time
  -- Cast the lightuserdata to cdata
  local yuyv_d = ffi.cast("uint32_t*", yuyv_ptr)
  -- Set the LUT Raw data
  local lut_d = ffi.cast("uint8_t*", lut_ptr)
  -- Temporary variables for the loop
  -- NOTE: 4 bytes yields 2 pixels, so stride of (4/2)*w
  local a_ptr, stride, yuyv = labelA_t:data(), w / 2
  for j=0,ha-1 do
    for i=0,wa-1 do
      yuyv = yuyv_d[0]
      -- Set the label 
      a_ptr[0] = lut_d[bor(
        rshift(band(yuyv, 0xFC000000), 26),
        rshift(band(yuyv, 0x0000FC00), 4),
        lshift(band(yuyv, 0xFC), 10)
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
  return labelA_t
end

function ImageProc.color_count (label_t)
  -- Reset the color count
  cc_t:zero()
  -- Loop variables
  local l_ptr, color = label_t:data()
  for i=0,np_a-1 do
    color = l_ptr[0]
    cc_d[color] = cc_d[color] + 1
    l_ptr = l_ptr + 1
  end
  return cc_t
end

-- Bit OR on blocks of NxN to get to labelB from labelA
local function block_bitorN (label_t)
  -- Zero the downsampled image
  labelB_t:zero()
  local a_ptr, b_ptr = label_t:data(), labelB_t:data()

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

  return labelB_t
end

-- Bit OR on blocks of 2x2 to get to labelB from labelA
local function block_bitor2 (label_t)
  -- Zero the downsampled image
  labelB_t:zero()
  local a_ptr, b_ptr = label_t:data(), labelB_t:data()
  
  -- Offset a row
  local a_ptr1 = a_ptr + wa
  -- Start the loop
  for jb=0,hb-1 do
    for ib=0,wb-1 do
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

  return labelB_t
end

-- Get the color stats for a bounding box
-- TODO: Add tilted color stats if needed
function ImageProc.color_stats ()
  
	-- Initialize statistics
	local area = 0
	local minI, maxI = width-1, 0
	local minJ, maxJ = height-1, 0
	local sumI, sumJ = 0, 0
	local sumII, sumJJ, sumIJ = 0, 0, 0
  
  for j=0,ha-1 do
    for i=0,wa-1 do
    end
  end
end

-- TODO: Add the line state machine
function ImageProc.line_stats (edge_t, threshold)
  threshold = threshold or 0
  local j, i, label0, label1
  -- Clear out any old transform
  RadonTransform.init()
  
  -- Scan for vertical field line pixels
  local e_ptr = edge_t:data()
  for j=0, edge_t:size(1)-1 do
    -- Use -2 and not -1 since we do not go to the edge
    for i=0, edge_t:size(2)-2 do
      label0 = e_ptr[0]
      e_ptr = e_ptr + 1
      label1 = e_ptr[0]
      if label0>threshold and label1>threshold then
        RadonTransform.addVerticalPixel(i, j)
      end
    end
  end
  
  -- Scan for horizontal field line pixels
  local e_ptr_l = edge_t:data()
  local e_ptr_r = e_ptr_l
  for j=0, edge_t:size(1)-2 do
    -- Use -2 and not -1 since we do not go to the edge
    for i=0, edge_t:size(2)-1 do
      label0 = e_ptr_l[0]
      e_ptr_l = e_ptr_l + 1
      label1 = e_ptr_r[0]
      e_ptr_r = e_ptr_r + 1
      if label0>threshold and label1>threshold then
        RadonTransform.addHorizontalPixel(i, j)
      end
    end
  end
  return RadonTransform.get_line_stats()
end

function ImageProc.label_to_edge (edge_t, threshold)
  
end

function ImageProc.yuyv_to_edge (yuyv_ptr, threshold, label)
  -- Wrap the pointer into a tensor
  local yuyv_t = torch.ByteTensor(ffi.cast('uint8_t*',yuyv_ptr),w*h*4)
  -- Reshape for a subsample.
  -- TODO: Should be resize onto the same storage, else a malloc!
	local yuyv_sub = r:reshape(h/2,w,4):sub(1,-1,1,w/2)
	-- Get the y-plane
	local y_plane = yuyv_sub:select(3,1)
	local u_plane = yuyv_sub:select(3,2)
	local v_plane = yuyv_sub:select(3,3)
	--local y1_plane = yuyv_sub:select(3,4)
	-- Perform the convolution on the Int y-plane
  -- TODO: Mix this
  -- This typecast is required
	y_int:resize(y_plane:size()):copy(y_plane)
  -- Resize the edge map as necessary
	edge:resize(
		y_plane:size(1)+kern:size(1)/2,
		y_plane:size(2)+kern:size(2)/2
	)
  -- Perform the convolution
	torch.conv2(edge,y_int,kern)
	-- Threshold (Somewhat not working...)
  label = label or 255
	edge_bin:resize(edge:size())
	edge_bin[edge:lt(0)] = 0
	edge_bin[edge:gt(THRESH)] = label
  -- TODO: Some other dynamic range compression
  return edge, edge_bin
end

-- Setup should be able to quickly switch between cameras
-- i.e. not much overhead here.
-- Resize should be expensive at most n_cameras times (if all increase the sz)
function ImageProc.setup (w0, h0, sA, sB)
  -- Save the scale paramter
  scaleA = sA or 2
  scaleB = sB or 2
  log_sA, log_sB = log2[scaleA], log2[scaleB]
  -- Recompute the width and height of the images
  w, h = w0, h0
  wa, ha = w / scaleA, h / scaleA
  wb, hb = wa / scaleB, ha / scaleB
  -- Save the number of pixels
  np_a, np_b = wa * ha, wb * hb
  -- Resize as needed
  labelA_t:resize(ha, wa)
  labelB_t:resize(hb, wb)
  -- Select faster bit_or
  if scaleB==2 then
    ImageProc.block_bitor = block_bitor2
  else
    ImageProc.block_bitor = block_bitorN
  end
end

return ImageProc
