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
-- Externally defined libraries
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

local kernel = {
	{0,  0,   3,  0,  0,},
	{0,  3,   8,  3,  0,},
	{3,  8, -30,  8,  3,},
	{0,  3,   8,  3,  0,},
	{0,  0,   3,  0,  0,}
}

----[[
local kernel = {
	{0, 0,   0,  0, 0,},
	{0, 1,   0,  0, 0,},
	{4, 10, -30,  10, 4,},
	{0, 0,   0,  1, 0,},
	{0, 0,   0,  0, 0,}
}
--]]

--[[
local kernel = {
	{5, 10, -30, 10, 5,},
}
--]]

local edge_t = torch.IntTensor()
local grey_t = torch.IntTensor()
local kernel_t = torch.IntTensor(kernel)--:t():clone()
--
local edge_char_t = torch.CharTensor()
local grey_char_t = torch.CharTensor()
local kernel_char_t = torch.CharTensor(kernel)
--
local yuv_samples_t = torch.DoubleTensor()
local grey_transformed_t = torch.DoubleTensor()

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
-- TODO: use the bbox in the for loop
function ImageProc.color_stats (label_t, bbox, color)
  local ni, nj = label_t:size(1), label_t:size(2)
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
	local minI, maxI = width-1, 0
	local minJ, maxJ = height-1, 0
	local sumI, sumJ = 0, 0
	local sumII, sumJJ, sumIJ = 0, 0, 0
  
  local l_ptr, label = label_t:data()
  for j=j0,j1 do
    for i=i0,i1 do
      label = l_ptr[i]
			if label == color then
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
				sumII = sumII + i * i
				sumJJ = sumJJ + j * j
				sumIJ = sumIJ + i * j
      end
      -- If      
    end
    l_ptr = l_ptr + ni
  end
  
end


local aH = RadonTransform.addHorizontalPixel
local aV = RadonTransform.addVerticalPixel

-- TODO: Add the line state machine
-- TODO: Use can shift by 90 degrees to make the search for line match reliable
-- because no edge spill over
function ImageProc.line_stats_old (edge_t, threshold, shift90)
  threshold = threshold or 2000
  local j, i, label0, label1
  -- Clear out any old transform
  RadonTransform.init(edge_t:size(1), edge_t:size(2))
  
  local e_ptr = edge_t:data()
  for j=0, edge_t:size(1)-1 do
    -- Use -2 and not -1 since we do not go to the edge
    for i=0, edge_t:size(2)-2 do
      label0 = e_ptr[0]
      e_ptr = e_ptr + 1
      label1 = e_ptr[0]
      if label0>threshold and label1>threshold then
        aH(i, j)
        --[[
        if shift90 then
          -- 90 degree shift
          aV(j, i)
        else
          -- 0 degree shift
          aH(i, j)
        end
        --]]
      end
    end
  end
  
  
  
  local aH = RadonTransform.addVerticalPixel
  local e_ptr_l = edge_t:data()
  local e_ptr_r = e_ptr_l + edge_t:size(2)
  for j=0, edge_t:size(1)-2 do
    -- Use -2 and not -1 since we do not go to the edge
    for i=0, edge_t:size(2)-1 do
      label0 = e_ptr_l[0]
      e_ptr_l = e_ptr_l + 1
      label1 = e_ptr_r[0]
      e_ptr_r = e_ptr_r + 1
      if label0>threshold and label1>threshold then
        aV(i, j)
        --[[
        if shift90 then
          -- 90 degree shift
          RadonTransform.addHorizontalPixel(j, i)
        else
          -- 0 degree shift
          RadonTransform.addVerticalPixel(i, j)
        end
        --]]
      end
    end
  end

  --return RadonTransform.get_line_stats()

  return
end




function ImageProc.line_stats (edge_t, threshold, shift90)
  threshold = threshold or 2000
  -- TODO: Use SouthEast pixel so we can have diagonal pixel resolution
  local j, i, label_nw, label_ne, label_sw, label_se
  -- Clear out any old transform
  RadonTransform.init(edge_t:size(1), edge_t:size(2))
    
  local e_ptr_l = edge_t:data()
  local e_ptr_r = e_ptr_l + edge_t:size(2)
  for j=0, edge_t:size(1)-2 do
    -- Use -2 and not -1 since we do not go to the edge
    for i=0, edge_t:size(2)-1 do
      label_nw = e_ptr_l[0]
      e_ptr_l = e_ptr_l + 1
      label_ne = e_ptr_l[0]
      label_sw = e_ptr_r[0]
      e_ptr_r = e_ptr_r + 1
      label_se = e_ptr_r[0]
      if label_nw>threshold then
        if label_sw>threshold then aV(i, j) end
        if label_ne>threshold then aH(i, j) end
      end
    end
  end

  RadonTransform.get_parallel_lines()

  --return RadonTransform.get_line_stats()
end





-- TODO: Remove these functions
util = require'util'
ImageProc.get_radon = function()
  return RadonTransform
end

local function pca(x)
    -- From: https://github.com/koraykv/unsup
   local mean = torch.mean(x,1)
   local xm = x - torch.ger(torch.ones(x:size(1)),mean:squeeze())
   xm:div(math.sqrt(x:size(1)-1))
   local w,s,v = torch.svd(xm:t())
   s:cmul(s)
   return s, w, mean, xm
end

-- NOTE: This is just a test funciton, really. Makes a greyscaleimage
function ImageProc.yuyv_color_stats (yuyv_ptr, bbox)
  local yuyv_s = torch.ByteStorage(
  w*h*4,
  tonumber(ffi.cast("intptr_t",yuyv_ptr))
  )
  local yuyv_t = torch.ByteTensor(yuyv_s, 1, torch.LongStorage{h/2,w,4})
  local yuyv_sub = yuyv_t:sub(1,-1,1,w/2)
	local y_plane = yuyv_sub:select(3,1)
	local u_plane = yuyv_sub:select(3,2)
  --local y1_plane = yuyv_sub:select(3,3)
	local v_plane = yuyv_sub:select(3,4)
  -- Select just the bbox pixels
  local ys, us, vs
  if bbox then
    -- Remember the coordinates and memory layout of a yuyv camera image
    local c, d, a, b = unpack(bbox)
    ys = y_plane:sub(a,b,c,d)
    us = u_plane:sub(a,b,c,d)
    vs = v_plane:sub(a,b,c,d)
  else
    ys = y_plane
    us = u_plane
    vs = v_plane
  end
  -- Copy into our samples, with a precision change, too
  local yuv_samples = torch.DoubleTensor(ys:nElement(),3)
  yuv_samples:select(2,1):copy(ys)
  yuv_samples:select(2,2):copy(us)
  yuv_samples:select(2,3):copy(vs)
  --util.ptorch(yuv_samples)
  -- Run PCA
  local eigenvalues, eigenvectors, mean, yuv_samples_0_mean = pca(yuv_samples)
  print('evecs')
  util.ptorch(eigenvectors)
  print('evals')
  util.ptorch(eigenvalues)
  --print('STUFF')
  --util.ptorch(eigenvalues:sub(1,1,1,1))
  --print('STUFF2')
  -- Convert samples to a nice space
  --print('make into a row',eigenvalues:sub(1,3,1,1) * yuv_samples_0_mean)
  --local transformed = torch.mv(yuv_samples_0_mean,eigenvectors[2])
  local transformed = torch.mv(yuv_samples_0_mean,eigenvectors:select(2,1))
  --util.ptorch(eigenvalues[1])
  --util.ptorch(eigenvalues:select(2,1))
  --util.ptorch(transformed)
  transformed:resize(ys:size(1),ys:size(2))
  --util.ptorch(transformed)
  return eigenvalues, eigenvectors, mean, ys, us, vs, transformed
end


-- TODO: Might not be the smartest approach...
-- That is why the state machine system is used
function ImageProc.label_to_edge (label_t, label)
  -- Resize as necessary. First time gets the hit
  -- Fill our map correctly
  grey_char_t:resize(label_t:size()):map(label_t, function(g, l)
    if l==label then return 1 else return 0 end
    end)
  -- Do the convolution in byte space
  -- TODO: Test any overflow issues!
  -- TODO: Do I even need to resize edge_char_t ?
  -- Or does that happen automagically?
  edge_char_t:conv2(grey_char_t, kernel_char_t)
  return edge_char_t
end

function ImageProc.yuyv_to_edge (yuyv_ptr, bbox)
  -- Wrap the pointer into a tensor
  local yuyv_s = torch.ByteStorage(
  w*h*4,
  tonumber(ffi.cast("intptr_t",yuyv_ptr))
  )
  local yuyv_sz = torch.LongStorage{h/2,w,4}
  local yuyv_t = torch.ByteTensor(yuyv_s, 1, yuyv_sz)
  local yuyv_sub = yuyv_t:sub(1,-1,1,w/2)
  -- TODO: Can modify the strides easily - wow!
  -- Reshape for a subsample.
  -- TODO: Should be resize onto the same storage, else a malloc!
  -- TODO: Could finagle the strides via the ffi ;)
  -- TODO: Be careful, since not contiguous, so reshape
  -- may help with type cast copy? or no?
  -- TODO: Support more subsample levels, too. This may work:
  -- yuyv_sub = yuyv_t:reshape(h/4,w,4):sub(1,-1,1,w/4)
  -- yuyv_sub = yuyv_t:reshape(h/8,w,4):sub(1,-1,1,w/8)
	-- Get the y-plane
	local y_plane = yuyv_sub:select(3,1)
	local u_plane = yuyv_sub:select(3,2)
  --local y1_plane = yuyv_sub:select(3,3)
	local v_plane = yuyv_sub:select(3,4)
  -- Form the greyscale image
  local ys, us, vs
  if bbox then
    -- Remember the coordinates and memory layout of a yuyv camera image
    local c, d, a, b = unpack(bbox)
    ys = y_plane:sub(a,b,c,d)
    us = u_plane:sub(a,b,c,d)
    vs = v_plane:sub(a,b,c,d)
  else
    ys = y_plane
    us = u_plane
    vs = v_plane
  end
  -- Copy into our samples, with a precision change, too
  -- TODO: Just use resize instead of making a new tensor
  yuv_samples_t:resize(ys:nElement(), 3)
  yuv_samples_t:select(2,1):copy(ys)
  yuv_samples_t:select(2,2):copy(us)
  yuv_samples_t:select(2,3):copy(vs)
  grey_transformed_t:resize(ys:nElement())
  -- Run PCA
  local eigenvalues, eigenvectors, mean, yuv_samples_0_mean = pca(yuv_samples_t)
  -- Transform into the grey space
  -- TODO: Need to figure out which eigenvector to use...
  torch.mv(grey_transformed_t,yuv_samples_0_mean,eigenvectors:select(2,1))
  -- Resize as an image
  grey_transformed_t:resize(ys:size(1),ys:size(2))
  -- Scale to the integer plane
  -- TODO: Just use the double space? I wonder how much of a speed hit...?
  grey_t:resize(grey_transformed_t:size()):copy(grey_transformed_t:mul(255))
  -- Perform the convolution
	edge_t:conv2(grey_t, kernel_t)
  return edge_t, grey_t
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
