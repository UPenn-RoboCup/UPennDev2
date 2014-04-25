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

-- Widths and Heights of Image, LabelA, LabelB
local w, h, wa, ha, wb, hb
-- Form the labelA and labelB tensors
local labelA_t, labelB_t = torch.ByteTensor(), torch.ByteTensor()
-- torch FFI raw data access
local lA_d, lB_d
-- Color Count always the same, as 8 bits for 8 colors means 256 color combos
local cc_t = torch.IntTensor(256)
-- The pointer will not change for this one
local cc_d = cc_t:data()
-- The Current Lookup table (Can be swapped dynamically)
local luts, lut_ns, lut_t, lut_d = {}, {}
-- Downscaling
local scaleA, scaleB

-- Setup should be able to quickly switch between cameras
-- i.e. not much overhead here.
-- Resize should be expensive at most n_cameras times (if all increase the sz)
function ImageProc.setup (w0, h0, lut, sA, sB)
  -- Save the scale paramter
  scaleA = sA or 2
  scaleB = sB or 2
  -- Recompute the width and height of the images
  w, h = w0, h0
  wa, ha = w/scaleA, h/scaleA
  wb, hb = wa/scaleB, ha/scaleB
  -- Resize as needed
  labelA_t:resize(ha, wa)
  labelB_t:resize(hb, wb)
  -- Raw data access
  lA_d, lB_d = labelA_t:data(), labelB_t:data()
  -- Lookup Table for Color -> Label
  if type(lut)=='number' then
    lut_t = luts[lut]
  else
    -- Check if already loaded
    local loaded
    for i,name in ipairs(lut_ns) do
      if lut==name then
        loaded = i
        break
      end
    end
    if loaded then
      lut_t = luts[i]
    else
      local f_lut = torch.DiskFile( fname , 'r')
      f_lut.binary(f_lut)
      -- We know the size of the LUT
      local lut_s = f_lut:readByte(262144)
      f_lut:close()
      -- Form a tensor for us
      lut_t = torch.ByteTensor(lut_s)
      table.insert(luts,lut_t)
      table.insert(lut_ns,lut)
    end
  end  
  -- Set the LUT Raw data
  lut_d = lut_t:data()
end

-- Take in a pointer (or string) to the image
-- Return labelA and the color count
function ImageProc.yuyv_to_label (yuyv_ptr)
  -- The yuyv pointer changes each time
  -- Cast the lightuserdata to cdata
  local yuyv_d = ffi.cast("uint32_t*", yuyv_ptr)
  -- Reset the color count
  cc_t:zero()
  -- Temporary variables for the loop
  -- NOTE: 4 bytes yields 2 pixels, so stride of (4/2)*w
  local a_ptr, yuyv_ptr, stride, index, cdt = lA_d, yuyv_d, w / 2
  for j=0,ha-1 do
    for i=0,wa-1 do
      index = bor(
        rshift(band(yuyv_ptr[0], 0xFC000000), 26),
        rshift(band(yuyv_ptr[0], 0x0000FC00), 4),
        lshift(band(yuyv_ptr[0], 0xFC), 10)
      )
      cdt = lut_d[index]
      --if cdt~=0 then print('cdt',cdt,index) end
      -- Increment the color count
      cc_d[cdt] = cc_d[cdt] + 1
      -- Move the labelA pointer
      a_ptr[0] = cdt
      a_ptr = a_ptr + 1
      -- Move the image pointer
      yuyv_ptr = yuyv_ptr + 1
    end
    -- stride to next
    yuyv_ptr = yuyv_ptr + stride
  end
  --
  return labelA_t, cc_t
end

-- Bit OR on blocks of 2x2 to get to labelB from labelA
-- TODO: could do 4x4 blocks, too, if we add a parameter
function ImageProc.block_bitor ()
  -- Zero the downsampled image
  labelB_t:zero()
  local a_ptr, b_ptr = lA_d, lB_d
  -- Offset a row
  local a_ptr1 = a_ptr + wa
  -- Start the loop
  for jb=0,hb-1 do
    for ib=0,wb-1 do
      b_ptr[0] = bor(a_ptr[0],a_ptr[1],a_ptr1[0],a_ptr1[1])
      -- Move to the next pixel
      a_ptr = a_ptr + 2
      a_ptr1 = a_ptr1 + 2
      -- Move b
      b_ptr = b_ptr + 1
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
	local sumI 0, sumJ = 0, 0
	local sumII, sumJJ, sumIJ = 0, 0, 0
  
  for j=0,ha-1 do
    for i=0,wa-1 do
    end
  end
end

return ImageProc
