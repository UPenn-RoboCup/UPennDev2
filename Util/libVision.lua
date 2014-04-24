-- libDetect
-- (c) 2014 Stephen McGill
-- General Detection methods
local libVision = {}
local torch  = require'torch'
local bit = require'bit'
local lshift = bit.lshift
local rshift = bit.rshift
local bor = bit.bor
local band = bit.band

local w, h, wa, ha, wb, hb
-- Form the labelA and labelB tensors
local labelA_t, labelB_t, lut_t, cc_t
-- torch raw data
local lut_d, lA_d, lB_d, cc_d

function libVision.get_labels()
  return labelA_t, labelB_t
end

function libVision.setup (w0, h0)
  -- Take in the width and height of the original YUYV image
  w, h = w0, h0
  wa, ha = w/2, h/2
  wb, hb = wa/2, ha/2
  labelA_t, labelB_t = torch.ByteTensor(ha,wa), torch.ByteTensor(hb,wb)
  -- Raw data access
  lA_d, lB_d = labelA_t:data(), labelB_t:data()
  -- Color count tensor
  cc_t = torch.IntTensor(256):zero()
  cc_d = cc_t:data()
end

function libVision.load_lut (fname)
  local f_lut = torch.DiskFile( fname , 'r')
  f_lut.binary(f_lut)
  -- We know the size of the LUT
  local lut_s = f_lut:readByte(262144)
  f_lut:close()
  -- Form a tensor for us
  lut_t = torch.ByteTensor(lut_s)
  -- Raw data
  lut_d = lut_t:data()
end

-- Take in a pointer (or string) to the image
-- Return nothing, since given the tensors
function libVision.yuyv_to_labelA (yuyv_ptr)
  -- The yuyv pointer changes each time
  -- Cast the lightuserdata to cdata
  local yuyv_d = ffi.cast("uint8_t*", yuyv_ptr)
  -- Reset the color count
  cc_t:zero()
  -- Temporary variables for the loop
  -- NOTE: 4 bytes yields 2 pixels, so stride of (4/2)*w
  local a_ptr, yuyv_ptr, stride, index, cdt = lA_d, yuyv_d, 2 * w
  for j=0,ha-1 do
    for i=0,wa-1 do
      index = bor(
        rshift(yuyv_ptr[0], 2),
        lshift(band(yuyv_ptr[1], 0xFC),4),
        lshift(band(yuyv_ptr[2], 0xFC),10)
      )
      cdt = lut_d[index]
      -- Increment the color count
      cc_d[cdt] = cc_d[cdt] + 1
      -- Move the labelA pointer
      a_ptr[0] = cdt
      a_ptr = a_ptr + 1
      -- Move the image pointer
      yuyv_ptr = yuyv_ptr + 4
    end
    -- stride to next
    yuyv_ptr = yuyv_ptr + stride
  end
  --
end

-- Bit OR on blocks of 2x2 to get to labelB
-- NOTE: could do 4x4 blocks, too, if we add a parameter
function libVision.form_labelB()
  -- Zero the downsampled image
  labelB_t:zero()
  -- Assumes a subsample of 2 only
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
end

return libVision
