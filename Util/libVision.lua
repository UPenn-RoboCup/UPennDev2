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
function libVision.yuyv_to_labelA (yuyv_ptr)
  -- Correct dimensions
  -- NOTE: 4 bytes yields 2 pixels, so stride of (4/2)*w
  local stride, idx, la_i = 2 * w, 0, 0
  -- The yuyv pointer changes each time
  -- Cast the lightuserdata to cdata
  local yuyv_d = ffi.cast("uint8_t*", yuyv_ptr)
  -- Reset the color count
  cc_t:zero()
  -- Temporary variables for the loop
  local y6, u6, v6, index, cdt
  for j=0,ha-1 do
    for i=0,wa-1 do
      y6, u6, v6 = 
      rshift(yuyv_d[idx], 2),
      band(yuyv_d[idx+1], 0xFC),
      band(yuyv_d[idx+2], 0xFC)
      index = bor(y6, lshift(u6,4), lshift(v6,10))
      cdt = lut_d[index]
      lA_d[la_i] = cdt
      -- Increment the color count
      cc_d[cdt] = cc_d[cdt] + 1
      -- Stride to next, assume 4
      idx = idx + 4
      la_i = la_i + 1
    end
    -- stride to next
    idx = idx + stride
  end
  -- Return nothing, since given the tensors
end

-- Bit OR on blocks of 2x2 to get to labelB
-- NOTE: could do 4x4 blocks, too, if we add a parameter
function libVision.form_labelB()
  -- Zero the downsmapled image
  labelB_t:zero()
  -- Grab pointer to labelA
  local a_ptr, sub = lA_d, 2
  local jb, ib, b_idx
  for ja=0,ha-1 do
    jb = math.floor(ja / sub)
    for ia=0,wa-1 do
      ib = math.floor(ia / sub)
      --b_idx = jb * wb + ib
      --print(jb,ib)
      labelB_t[jb+1][ib+1] = bor(labelB_t[jb+1][ib+1], a_ptr[0])
      a_ptr = a_ptr + 1
    end
  end
  -- Let's see if we can make this faster
end

return libVision
